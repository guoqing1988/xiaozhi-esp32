#include "wifi_board.h"
#include "audio_codecs/es8388_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "boards/common/servo_control.h" // Include ServoControl header
#include "iot/thing_manager.h"
#include "iot/things/servo_thing.h" // Include ServoThing header
#include "led/single_led.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <string> // Required for std::string
#include "audio_processing/wake_word_detect.h" // Required for WakeWordDetect
#include "application.h" // Required for Application::GetInstance()
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif
 
#define TAG "CompactVoiceServoBoard"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

class CompactVoiceServoBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_handle_ = nullptr;
    ES8388AudioCodec* audio_codec_ = nullptr;
    ServoControl* servo_ = nullptr; // Add ServoControl member
    WakeWordDetect* wake_word_detector_ = nullptr; // Add WakeWordDetect member
 
    Button boot_button_;
    LcdDisplay* display_;

    void InitializeI2CMaster() {
        i2c_master_bus_config_t i2c_mst_config = {
            .i2c_port = I2C_NUM_0, // Assuming I2C_NUM_0, change if needed
            .sda_io_num = I2C_MASTER_SDA_IO,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_filter_ns = 0,
            .intr_priority = 0,
            .trans_queue_depth = 0, // Set to 0 for noisr mode
            .flags = {
                .extern_sda_pullup = 0, // External pullups are used
                .extern_scl_pullup = 0, // External pullups are used
            }
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };        
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        
        esp_lcd_panel_reset(panel);
 

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_16_4,
                                        .icon_font = &font_awesome_16_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
#endif
                                    });
    }


 
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Lamp"));
        thing_manager.AddThing(iot::CreateThing("ServoThing")); // Add ServoThing
    }

public:
    CompactVoiceServoBoard() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2CMaster(); // Initialize I2C master first
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeIot();

        // Instantiate audio_codec_ after I2C is initialized
        audio_codec_ = new Es8388AudioCodec(i2c_bus_handle_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            PA_OUTPUT_GPIO, ES8388_I2C_ADDR);
        
        // Instantiate ServoControl
        servo_ = new ServoControl(SERVO_CONTROL_GPIO, LEDC_TIMER_0, LEDC_CHANNEL_0);
        if (servo_) {
            servo_->set_angle(90); // Set initial angle to 90 degrees
            iot::ServoThing::SetServoInstance(servo_); // Set the static instance for ServoThing
        }

        // Initialize WakeWordDetector
        wake_word_detector_ = new WakeWordDetect();
        if (audio_codec_) { // Ensure audio_codec_ is valid before using
            wake_word_detector_->Initialize(audio_codec_);
            wake_word_detector_->OnWakeWordDetected([this](const std::string& wake_word) {
                ESP_LOGI(TAG, "Wake word detected: %s", wake_word.c_str());
                static int command_idx = 0;
                std::string mock_command;
                if (command_idx == 0) mock_command = "servo_left";
                else if (command_idx == 1) mock_command = "servo_center";
                else mock_command = "servo_right";
                command_idx = (command_idx + 1) % 3;

                ESP_LOGI(TAG, "Executing mock command: %s", mock_command.c_str());
                this->ProcessVoiceCommand(mock_command); 
                
                // Important: Restart wake word detection after processing a command
                if (this->wake_word_detector_ && !this->wake_word_detector_->IsDetectionRunning()) {
                     this->wake_word_detector_->StartDetection();
                }
            });
            wake_word_detector_->StartDetection();
        } else {
            ESP_LOGE(TAG, "Audio codec is null, cannot initialize wake word detector.");
        }
        
        Application::GetInstance().SetAudioProcessor(wake_word_detector_);

        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    ~CompactVoiceServoBoard() {
        if (wake_word_detector_) { 
            wake_word_detector_->StopDetection(); 
            delete wake_word_detector_; 
            wake_word_detector_ = nullptr; 
        }
        if (servo_) { 
            delete servo_; 
            servo_ = nullptr; 
        }
        // audio_codec_ is managed by this class as per previous steps, so delete it.
        if (audio_codec_) { 
            delete audio_codec_; 
            audio_codec_ = nullptr; 
        }
        // Note: i2c_bus_handle_ might need cleanup if i2c_del_master_bus is the counterpart to i2c_new_master_bus
        if (i2c_bus_handle_){
            i2c_del_master_bus(i2c_bus_handle_);
            i2c_bus_handle_ = nullptr;
        }
    }

private: // Ensure ProcessVoiceCommand is private
    void ProcessVoiceCommand(const std::string& command_text) {
        ESP_LOGI(TAG, "Processing voice command: %s", command_text.c_str());
        if (servo_) {
            if (command_text == "servo_left") {
                servo_->set_angle(0);
                ESP_LOGI(TAG, "Setting servo angle to 0 (left)");
            } else if (command_text == "servo_center") {
                servo_->set_angle(90);
                ESP_LOGI(TAG, "Setting servo angle to 90 (center)");
            } else if (command_text == "servo_right") {
                servo_->set_angle(180);
                ESP_LOGI(TAG, "Setting servo angle to 180 (right)");
            } else {
                ESP_LOGW(TAG, "Unknown voice command: %s", command_text.c_str());
            }
        } else {
            ESP_LOGE(TAG, "Servo not initialized, cannot process voice command.");
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        return audio_codec_;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactVoiceServoBoard);
