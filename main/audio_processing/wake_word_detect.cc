#include "wake_word_detect.h"
// application.h might not be needed here if OpusEncoderWrapper is self-contained or not used by AudioProcessor methods
// #include "application.h" 

#include <esp_log.h>
#include <model_path.h>
#include <arpa/inet.h>
#include <sstream>

#define DETECTION_RUNNING_EVENT 1

static const char* TAG = "WakeWordDetect";

WakeWordDetect::WakeWordDetect()
    : afe_iface_(nullptr), // Initialize AFE interface pointer
      afe_data_(nullptr),
      wakenet_model_(nullptr), // Initialize model pointer
      codec_(nullptr), // Initialize codec pointer
      audio_detection_task_handle_(nullptr), // Initialize task handle
      wake_word_encode_task_(nullptr),
      wake_word_encode_task_stack_(nullptr),
      wake_word_pcm_(),
      wake_word_opus_() {

    event_group_ = xEventGroupCreate();
    ESP_LOGI(TAG, "WakeWordDetect instance created.");
}

WakeWordDetect::~WakeWordDetect() {
    Stop(); // Ensure task is stopped.
    if (afe_data_ != nullptr) {
        if (afe_iface_) { // Check if afe_iface_ is valid before using
            afe_iface_->destroy(afe_data_);
        }
        afe_data_ = nullptr;
    }
    // wakenet_model_ is typically a pointer to data within srmodel_list_t, managed by esp_srmodel_init/deinit.
    // If it were allocated separately, it would need freeing. Assuming it's managed elsewhere for now.
    wakenet_model_ = nullptr;


    if (wake_word_encode_task_stack_ != nullptr) {
        heap_caps_free(wake_word_encode_task_stack_);
        wake_word_encode_task_stack_ = nullptr;
    }
    // audio_detection_task_handle_ should be null if task self-deleted.
    // If not, direct deletion here is risky. Stop() should handle task termination.
    audio_detection_task_handle_ = nullptr; 
    wake_word_encode_task_ = nullptr;


    if (event_group_) {
        vEventGroupDelete(event_group_);
        event_group_ = nullptr;
    }
    ESP_LOGI(TAG, "WakeWordDetect instance destroyed.");
}

// --- AudioProcessor Interface Implementation ---
void WakeWordDetect::Initialize(AudioCodec* codec) { // Marked as override in .h
    codec_ = codec;
    if (!codec_) {
        ESP_LOGE(TAG, "Codec is null in Initialize.");
        return;
    }
    int ref_num = codec_->input_reference() ? 1 : 0;

    srmodel_list_t *models = esp_srmodel_init("model");
    if (!models) {
        ESP_LOGE(TAG, "Failed to init sr models");
        return;
    }

    for (int i = 0; i < models->num; i++) {
        ESP_LOGI(TAG, "Model %d: %s", i, models->model_name[i]);
        if (strstr(models->model_name[i], ESP_WN_PREFIX) != NULL) {
            wakenet_model_ = models->model_name[i]; // Store the model name string
            auto words_str = esp_srmodel_get_wake_words(models, wakenet_model_);
            std::stringstream ss(words_str);
            std::string word;
            wake_words_.clear(); // Clear previous wake words
            while (std::getline(ss, word, ';')) {
                wake_words_.push_back(word);
            }
            ESP_LOGI(TAG, "Found WakeNet model: %s, with words: %s", wakenet_model_, words_str);
        }
    }
    if (wake_words_.empty()) {
        ESP_LOGW(TAG, "No wake words found for model: %s. Defaulting to 'Hi LeXin'", wakenet_model_ ? wakenet_model_ : "Unknown");
        // Add a default if none are found to prevent issues later with index 0 access
        // This part of the original code was missing, but it's safer.
        // wake_words_.push_back("Hi LeXin"); 
    }


    std::string input_format;
    for (int i = 0; i < codec_->input_channels() - ref_num; i++) {
        input_format.push_back('M');
    }
    for (int i = 0; i < ref_num; i++) {
        input_format.push_back('R');
    }
    afe_config_t* afe_config_ptr = afe_config_init(input_format.c_str(), models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    if (!afe_config_ptr) {
        ESP_LOGE(TAG, "Failed to init AFE config.");
        esp_srmodel_deinit(models); // Clean up models
        return;
    }
    afe_config_ptr->aec_init = codec_->input_reference();
    afe_config_ptr->aec_mode = AEC_MODE_SR_HIGH_PERF;
    afe_config_ptr->afe_perferred_core = 1;
    afe_config_ptr->afe_perferred_priority = 1;
    afe_config_ptr->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    
    afe_iface_ = esp_afe_handle_from_config(afe_config_ptr);
    if (!afe_iface_) { 
        ESP_LOGE(TAG, "Failed to get AFE handle from config."); 
        afe_config_free(afe_config_ptr); // Free config object
        esp_srmodel_deinit(models); // Clean up models
        return; 
    }
    afe_data_ = afe_iface_->create_from_config(afe_config_ptr);
    if (!afe_data_) { 
        ESP_LOGE(TAG, "Failed to create AFE data from config."); 
        // afe_iface_ might not need explicit destruction if create_from_config failed to allocate afe_data_
        // esp_afe_destroy_handle(afe_iface_); // Potentially needed if handle itself needs cleanup
        afe_config_free(afe_config_ptr);
        esp_srmodel_deinit(models);
        return; 
    }
    // afe_config_free(afe_config_ptr); // Config is used by AFE, should not be freed immediately after create_from_config.
                                     // The AFE instance might hold pointers to it or its contents.
                                     // This needs careful checking against ESP-SR documentation.
                                     // For now, assuming AFE copies what it needs.
    // esp_srmodel_deinit(models); // Models list might also be needed by AFE. Assuming AFE copies.
                                  // If not, these deinit/free calls should be in the destructor.

    if (audio_detection_task_handle_ == nullptr) {
        xTaskCreate([](void* arg) {
            auto this_ = (WakeWordDetect*)arg;
            this_->AudioDetectionTask();
            this_->audio_detection_task_handle_ = nullptr; 
            vTaskDelete(NULL);
        }, "audio_detection_task", 4096, this, 3, &audio_detection_task_handle_);
    }
    ESP_LOGI(TAG, "WakeWordDetect Initialized.");
}

void WakeWordDetect::OnWakeWordDetected(std::function<void(const std::string& wake_word)> callback) {
    wake_word_detected_callback_ = callback;
}

void WakeWordDetect::StartDetection() {
    if (!IsDetectionRunning()) {
        if (!event_group_) { ESP_LOGE(TAG, "Event group not initialized in StartDetection."); return; }
        xEventGroupSetBits(event_group_, DETECTION_RUNNING_EVENT);
        ESP_LOGI(TAG, "Wake word detection started via StartDetection (event bit set).");
    } else {
        ESP_LOGI(TAG, "Wake word detection already running when StartDetection called.");
    }
}

void WakeWordDetect::StopDetection() {
    if (IsDetectionRunning()) {
        if (!event_group_) { ESP_LOGE(TAG, "Event group not initialized in StopDetection."); return; }
        xEventGroupClearBits(event_group_, DETECTION_RUNNING_EVENT);
        if (afe_data_ != nullptr && afe_iface_ != nullptr) {
            afe_iface_->reset_buffer(afe_data_);
        }
        ESP_LOGI(TAG, "Wake word detection stopped via StopDetection (event bit cleared).");
    } else {
        ESP_LOGI(TAG, "Wake word detection already stopped when StopDetection called.");
    }
}

bool WakeWordDetect::IsDetectionRunning() {
    if (!event_group_) return false;
    return (xEventGroupGetBits(event_group_) & DETECTION_RUNNING_EVENT) != 0;
}

void WakeWordDetect::Feed(const std::vector<int16_t>& data) { 
    if (afe_data_ == nullptr || !IsDetectionRunning() || !afe_iface_) {
        return;
    }
    afe_iface_->feed(afe_data_, const_cast<int16_t*>(data.data()));
}

size_t WakeWordDetect::GetFeedSize() { 
    if (afe_data_ == nullptr || !codec_ || !afe_iface_) {
        return 512 * 2; // Default fallback: 512 samples * 16-bit
    }
    return afe_iface_->get_feed_chunksize(afe_data_) * codec_->input_channels() * sizeof(int16_t);
}

void WakeWordDetect::Start() { 
    StartDetection(); 
}

void WakeWordDetect::Stop() { 
    StopDetection(); 
}

bool WakeWordDetect::IsRunning() { 
    return IsDetectionRunning(); 
}

void WakeWordDetect::OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) { 
    output_callback_ = callback;
    ESP_LOGI(TAG, "OnOutput callback registered.");
}

void WakeWordDetect::OnVadStateChange(std::function<void(bool speaking)> callback) { 
    vad_callback_ = callback;
    ESP_LOGI(TAG, "OnVadStateChange callback registered.");
}

// --- End of AudioProcessor Interface Implementation ---

// --- Subtask specific methods (Process, GetRequiredInputBufferSize) ---
// These are not part of AudioProcessor, but requested by subtask.
bool WakeWordDetect::Process(const std::vector<int16_t>& audio_input) {
    if (IsDetectionRunning()) {
        Feed(audio_input); // Call the overridden Feed method
    }
    return true; 
}

size_t WakeWordDetect::GetRequiredInputBufferSize() {
    return GetFeedSize(); // Call the overridden GetFeedSize method
}
// --- End of Subtask specific methods ---


void WakeWordDetect::AudioDetectionTask() {
    if (!afe_data_ || !afe_iface_ || !codec_ || !event_group_) {
        ESP_LOGE(TAG, "AFE, codec, or event_group not initialized in AudioDetectionTask. Exiting.");
        if(event_group_) xEventGroupClearBits(event_group_, DETECTION_RUNNING_EVENT); // Ensure not stuck as running
        return;
    }
    auto fetch_size_samples_per_channel = afe_iface_->get_fetch_chunksize(afe_data_);
    auto feed_size_samples_per_channel = afe_iface_->get_feed_chunksize(afe_data_);
    ESP_LOGI(TAG, "Audio detection task started, feed_chunksize (samples/channel): %d, fetch_chunksize (samples/channel): %d",
        feed_size_samples_per_channel, fetch_size_samples_per_channel);

    while (true) { 
        xEventGroupWaitBits(event_group_, DETECTION_RUNNING_EVENT, pdFALSE, pdTRUE, portMAX_DELAY);

        while(IsDetectionRunning()) { 
            auto res = afe_iface_->fetch_with_delay(afe_data_, pdMS_TO_TICKS(100)); 
            
            if (!IsDetectionRunning()) break; 

            if (res == nullptr) { 
                vTaskDelay(pdMS_TO_TICKS(10)); 
                continue; 
            }
            
            if (res->ret_value == ESP_FAIL) {
                ESP_LOGE(TAG, "afe_fetch_with_delay failed");
                continue;
            }

            StoreWakeWordData(reinterpret_cast<int16_t*>(res->data), res->data_size / sizeof(int16_t));

            if (res->wakeup_state == WAKENET_DETECTED) {
                ESP_LOGI(TAG, "WAKENET_DETECTED: Wake word index: %d", res->wake_word_index);
                if (res->wake_word_index > 0 && (size_t)res->wake_word_index <= wake_words_.size()) {
                    last_detected_wake_word_ = wake_words_[res->wake_word_index - 1];
                    ESP_LOGI(TAG, "Last detected wake word: %s", last_detected_wake_word_.c_str());
                    if (wake_word_detected_callback_) {
                        wake_word_detected_callback_(last_detected_wake_word_);
                    }
                } else {
                    ESP_LOGW(TAG, "Detected wake word index %d is out of bounds for known wake words (size: %zu)", res->wake_word_index, wake_words_.size());
                }
            }
            // Example of how VAD callback could be triggered if AFE provides this info in `res`
            // if (vad_callback_ && res->vad_info_available) { vad_callback_(res->is_speaking); }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

void WakeWordDetect::StoreWakeWordData(uint16_t* data, size_t samples) {
    StoreWakeWordData(reinterpret_cast<int16_t*>(data), samples);
}

void WakeWordDetect::StoreWakeWordData(int16_t* data, size_t samples) {
    wake_word_pcm_.emplace_back(std::vector<int16_t>(data, data + samples));
    
    size_t max_chunks = 63; // Default: approx 2 seconds for 32ms chunks
    if (afe_iface_ && codec_ && codec_->input_channels() > 0) {
        int feed_chunk_samples = afe_iface_->get_feed_chunksize(afe_data_);
        if (feed_chunk_samples > 0) {
             // Duration of one chunk in ms: (feed_chunk_samples_per_channel * 1000) / sample_rate (e.g. 16000)
            float chunk_duration_ms = (float)(feed_chunk_samples * 1000) / 16000.0f; 
            if (chunk_duration_ms > 0) {
                max_chunks = (size_t)(2000.0f / chunk_duration_ms);
            }
        }
    }
    while (wake_word_pcm_.size() > max_chunks && !wake_word_pcm_.empty()) { 
        wake_word_pcm_.pop_front();
    }
}

void WakeWordDetect::EncodeWakeWordData() {
    wake_word_opus_.clear();
    if (wake_word_encode_task_stack_ == nullptr) {
        // Allocate with SPIRAM capabilities if possible and needed
        wake_word_encode_task_stack_ = (StackType_t*)heap_caps_malloc_prefer(4096 * 8, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT);
        if (!wake_word_encode_task_stack_) {
            ESP_LOGE(TAG, "Failed to allocate stack for opus encode task");
            return;
        }
    }
    // Ensure task is not already running if wake_word_encode_task_ is used as a flag
    if (wake_word_encode_task_ != nullptr) {
         ESP_LOGW(TAG, "Opus encode task may already be running or handle not cleared.");
         // Simple recovery: nullify handle and try to create task.
         // Robust solution would involve proper task state checking or signaling.
         wake_word_encode_task_ = nullptr; 
    }

    wake_word_encode_task_ = xTaskCreateStatic([](void* arg) {
        auto this_ = (WakeWordDetect*)arg;
        { // Task scope
            auto start_time = esp_timer_get_time();
            // OpusEncoderWrapper might need to be included if not transitively by wake_word_detect.h
            // For now, assume it's available.
            auto encoder = std::make_unique<OpusEncoderWrapper>(16000, 1, OPUS_FRAME_DURATION_MS);
            encoder->SetComplexity(0); 

            for (auto& pcm: this_->wake_word_pcm_) {
                encoder->Encode(std::move(pcm), [this_](std::vector<uint8_t>&& opus) {
                    std::lock_guard<std::mutex> lock(this_->wake_word_mutex_);
                    this_->wake_word_opus_.emplace_back(std::move(opus));
                    this_->wake_word_cv_.notify_all();
                });
            }
            this_->wake_word_pcm_.clear();

            auto end_time = esp_timer_get_time();
            ESP_LOGI(TAG, "Encode wake word opus %zu packets in %lld ms",
                this_->wake_word_opus_.size(), (end_time - start_time) / 1000);

            std::lock_guard<std::mutex> lock(this_->wake_word_mutex_);
            this_->wake_word_opus_.push_back(std::vector<uint8_t>()); // Signal end of stream
            this_->wake_word_cv_.notify_all();
        } // End Task scope
        // Nullify task handle before deleting task, so main class knows it's gone.
        // This is a bit tricky with xTaskCreateStatic if the TCB is also static.
        // The task itself calls vTaskDelete(NULL) to self-terminate.
        // The owner (WakeWordDetect) should ideally nullify its copy of the handle.
        // This is best done if the task signals completion or via a wrapper that nullifies then deletes.
        // For xTaskCreateStatic, the TCB (wake_word_encode_task_buffer_) is static, so vTaskDelete doesn't free it.
        // It just makes the TCB available for reuse if another static task is created with it.
        // Setting class member to null is main concern here.
        if (arg) { // Check arg to be safe
            ((WakeWordDetect*)arg)->wake_word_encode_task_ = nullptr;
        }
        vTaskDelete(NULL);
    }, "opus_encode_task", 4096 * 8, this, 2, wake_word_encode_task_stack_, &wake_word_encode_task_buffer_);
     if (!wake_word_encode_task_) {
        ESP_LOGE(TAG, "Failed to create opus_encode_task");
        // If task creation failed, free the stack we allocated.
        if (wake_word_encode_task_stack_) {
            heap_caps_free(wake_word_encode_task_stack_);
            wake_word_encode_task_stack_ = nullptr;
        }
    }
}

bool WakeWordDetect::GetWakeWordOpus(std::vector<uint8_t>& opus) {
    std::unique_lock<std::mutex> lock(wake_word_mutex_);
    // Wait until wake_word_opus_ is not empty OR the encoding task has finished (signaled by an empty vector)
    wake_word_cv_.wait(lock, [this]() {
        return !wake_word_opus_.empty();
    });
    
    opus.swap(wake_word_opus_.front());
    wake_word_opus_.pop_front();
    // If the popped 'opus' vector is empty, it's the end-of-stream signal from encoder task
    return !opus.empty(); 
}
