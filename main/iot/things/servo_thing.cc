#include "iot/things/servo_thing.h"
#include "boards/common/servo_control.h" // For ServoControl definition
#include <cJSON.h>
#include <esp_log.h>

namespace iot {

// Define static members
ServoControl* ServoThing::servo_instance = nullptr;

void ServoThing::SetServoInstance(ServoControl* servo) {
    servo_instance = servo;
}

ServoThing::ServoThing() : Thing("Servo", "A controllable servo motor") {
    // Define a method "setAngle"
    iot::ParameterList params;
    params.AddParameter({"angle", "Target angle for the servo (0-180)", iot::kValueTypeNumber, true});
    
    methods_.AddMethod("setAngle", "Sets the servo angle", params, [this](const iot::ParameterList& actual_params) {
        if (ServoThing::servo_instance) {
            // Assuming "angle" parameter exists and is a number, as defined.
            // The ParameterList access might need error checking in a real scenario.
            double angle_double = actual_params["angle"].number();
            int angle = static_cast<int>(angle_double);

            if (angle < 0) angle = 0;
            if (angle > 180) angle = 180;
            
            ServoThing::servo_instance->set_angle(angle);
            ESP_LOGI("ServoThing", "Set angle to %d via IoT", angle);
        } else {
            ESP_LOGE("ServoThing", "Servo instance not set!");
        }
    });

    // "currentAngle" property is omitted as ServoControl does not support reading the angle.
}

// DECLARE_THING macro to allow factory creation
DECLARE_THING(ServoThing);

} // namespace iot
