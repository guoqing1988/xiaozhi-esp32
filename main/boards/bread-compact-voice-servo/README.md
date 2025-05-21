# Bread Compact Voice Servo Board

## Description
Board based on `bread-compact-wifi-lcd` with added voice control for a servo motor. It utilizes an ESP32-S3 microcontroller.

## Key Features
- Voice control capabilities (wake-word based)
- Servo motor support
- ES8388 audio codec for audio input and output
- Wi-Fi connectivity
- LCD display (inherited from `bread-compact-wifi-lcd`)

## Pinout for Critical New Connections

### Servo Control
- **Servo Control Pin**: `GPIO_NUM_2` (as defined in `config.h`)

### ES8388 Audio Codec (I2S Interface)
- **MCLK**: `GPIO_NUM_0`
- **BCLK**: `GPIO_NUM_5`
- **WS (LRCK)**: `GPIO_NUM_4`
- **DOUT (DAC Data)**: `GPIO_NUM_7`
- **DIN (ADC Data)**: `GPIO_NUM_6`

### ES8388 Audio Codec (I2C Interface)
- **SCL (Clock)**: `GPIO_NUM_18`
- **SDA (Data)**: `GPIO_NUM_17`
- **I2C Address**: `0x10`

*(Note: GPIO pin numbers are based on definitions in `main/boards/bread-compact-voice-servo/config.h`)*

## Basic Voice Commands Supported
Currently, after the wake word "Hi LeXin" (or other configured wake word) is detected, the system cycles through the following mock commands:
- "servo_left" (sets servo to 0 degrees)
- "servo_center" (sets servo to 90 degrees)
- "servo_right" (sets servo to 180 degrees)

**Important Note**: Actual speech-to-text for varied voice commands is not yet implemented. The commands above are triggered sequentially as a demonstration of the voice-to-action pipeline.

## Building
To build for this board, ensure your ESP-IDF environment is set up, then use a command similar to:
```bash
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;boards/bread-compact-voice-servo/sdkconfig.defaults" build
```
(Note: A specific `boards/bread-compact-voice-servo/sdkconfig.defaults` may not be strictly necessary if the main `sdkconfig.defaults` and Kconfig options are sufficient.)

Select `BOARD_TYPE_BREAD_COMPACT_VOICE_SERVO` in `menuconfig` if prompted or to customize the build.
