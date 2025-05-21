#ifndef WAKE_WORD_DETECT_H
#define WAKE_WORD_DETECT_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_afe_sr_models.h>
#include <esp_nsn_models.h>

#include <list>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <condition_variable>

#include "audio_codec.h"
#include "audio_processing/audio_processor.h" // Include AudioProcessor

class WakeWordDetect : public AudioProcessor { // Inherit from AudioProcessor
public:
    WakeWordDetect();
    ~WakeWordDetect();

    // AudioProcessor interface methods
    void Initialize(AudioCodec* codec) override; // Already exists, make it override
    void Feed(const std::vector<int16_t>& data) override; // Already exists, make it override
    void Start() override; // To be implemented, maps to StartDetection
    void Stop() override; // To be implemented, maps to StopDetection
    bool IsRunning() override; // To be implemented, maps to IsDetectionRunning
    void OnOutput(std::function<void(std::vector<int16_t>&& data)> callback) override;
    void OnVadStateChange(std::function<void(bool speaking)> callback) override;
    size_t GetFeedSize() override; // Already exists, make it override. Note: problem description used GetRequiredInputBufferSize, using existing GetFeedSize as it matches the purpose.

    // WakeWordDetect specific methods
    void OnWakeWordDetected(std::function<void(const std::string& wake_word)> callback); // Already exists
    void StartDetection(); // Already exists
    void StopDetection(); // Already exists
    bool IsDetectionRunning(); // Already exists
    void EncodeWakeWordData(); // Already exists
    bool GetWakeWordOpus(std::vector<uint8_t>& opus); // Already exists
    const std::string& GetLastDetectedWakeWord() const { return last_detected_wake_word_; }

private:
    esp_afe_sr_iface_t* afe_iface_ = nullptr;
    esp_afe_sr_data_t* afe_data_ = nullptr;
    char* wakenet_model_ = NULL;
    std::vector<std::string> wake_words_;
    EventGroupHandle_t event_group_;
    std::function<void(const std::string& wake_word)> wake_word_detected_callback_;
    AudioCodec* codec_ = nullptr; // Already a member, used by Initialize
    std::string last_detected_wake_word_;
    std::function<void(std::vector<int16_t>&& data)> output_callback_; // For AudioProcessor
    std::function<void(bool speaking)> vad_callback_; // For AudioProcessor

    TaskHandle_t wake_word_encode_task_ = nullptr;
    StaticTask_t wake_word_encode_task_buffer_;
    StackType_t* wake_word_encode_task_stack_ = nullptr;
    std::list<std::vector<int16_t>> wake_word_pcm_;
    std::list<std::vector<uint8_t>> wake_word_opus_;
    std::mutex wake_word_mutex_;
    std::condition_variable wake_word_cv_;

    void StoreWakeWordData(uint16_t* data, size_t size);
    void AudioDetectionTask();
};

#endif
