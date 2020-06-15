This folder contains messages to communicate with six wheeled car's motor controller.

In SixWheelCommand.msg You can Choose Control Types 

1. If you choose 0 you can set 'right_speed' and 'left_speed'.
2. If you choose 1 you can set 'linearspeed' and 'angle'. Note that this control method was the default method and wasn't working well.
3. If you choose 2 then you can set 'motor_number'(1 to 6) and 'individual_motor_speed'

In SixWheelCommand.msg you can read linearspeed  'motor(n) _ speed' 'motor(n)_current' 'voltage'  and 'temperature' (I am sorry for syntax error)

This message is published by sixwd_msgs pakage and please note that all the unit's are in bytes therefore it should be converted later on. 