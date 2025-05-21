#ifndef _SERVO_THING_H_
#define _SERVO_THING_H_

#include "iot/thing.h"

// Forward declare ServoControl to avoid including its header here
class ServoControl; 

namespace iot {

class ServoThing : public Thing {
public:
    ServoThing(); // Constructor will use static servo_instance
    
    // Static members to allow ServoControl instance to be set from board initialization
    static ServoControl* servo_instance;
    static void SetServoInstance(ServoControl* servo);

    // DECLARE_THING macro is typically used in the .cc file
};

} // namespace iot

#endif // _SERVO_THING_H_
