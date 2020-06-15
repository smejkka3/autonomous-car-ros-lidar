;; Auto-generated. Do not edit!


(when (boundp 'sixwd_msgs::SixWheelInfo)
  (if (not (find-package "SIXWD_MSGS"))
    (make-package "SIXWD_MSGS"))
  (shadow 'SixWheelInfo (find-package "SIXWD_MSGS")))
(unless (find-package "SIXWD_MSGS::SIXWHEELINFO")
  (make-package "SIXWD_MSGS::SIXWHEELINFO"))

(in-package "ROS")
;;//! \htmlinclude SixWheelInfo.msg.html


(defclass sixwd_msgs::SixWheelInfo
  :super ros::object
  :slots (_linearspeed _motor1_speed _motor2_speed _motor3_speed _motor4_speed _motor5_speed _motor6_speed _motor1_current _motor2_current _motor3_current _motor4_current _motor5_current _motor6_current _voltage _temperature ))

(defmethod sixwd_msgs::SixWheelInfo
  (:init
   (&key
    ((:linearspeed __linearspeed) 0)
    ((:motor1_speed __motor1_speed) 0)
    ((:motor2_speed __motor2_speed) 0)
    ((:motor3_speed __motor3_speed) 0)
    ((:motor4_speed __motor4_speed) 0)
    ((:motor5_speed __motor5_speed) 0)
    ((:motor6_speed __motor6_speed) 0)
    ((:motor1_current __motor1_current) 0)
    ((:motor2_current __motor2_current) 0)
    ((:motor3_current __motor3_current) 0)
    ((:motor4_current __motor4_current) 0)
    ((:motor5_current __motor5_current) 0)
    ((:motor6_current __motor6_current) 0)
    ((:voltage __voltage) 0)
    ((:temperature __temperature) 0)
    )
   (send-super :init)
   (setq _linearspeed (round __linearspeed))
   (setq _motor1_speed (round __motor1_speed))
   (setq _motor2_speed (round __motor2_speed))
   (setq _motor3_speed (round __motor3_speed))
   (setq _motor4_speed (round __motor4_speed))
   (setq _motor5_speed (round __motor5_speed))
   (setq _motor6_speed (round __motor6_speed))
   (setq _motor1_current (round __motor1_current))
   (setq _motor2_current (round __motor2_current))
   (setq _motor3_current (round __motor3_current))
   (setq _motor4_current (round __motor4_current))
   (setq _motor5_current (round __motor5_current))
   (setq _motor6_current (round __motor6_current))
   (setq _voltage (round __voltage))
   (setq _temperature (round __temperature))
   self)
  (:linearspeed
   (&optional __linearspeed)
   (if __linearspeed (setq _linearspeed __linearspeed)) _linearspeed)
  (:motor1_speed
   (&optional __motor1_speed)
   (if __motor1_speed (setq _motor1_speed __motor1_speed)) _motor1_speed)
  (:motor2_speed
   (&optional __motor2_speed)
   (if __motor2_speed (setq _motor2_speed __motor2_speed)) _motor2_speed)
  (:motor3_speed
   (&optional __motor3_speed)
   (if __motor3_speed (setq _motor3_speed __motor3_speed)) _motor3_speed)
  (:motor4_speed
   (&optional __motor4_speed)
   (if __motor4_speed (setq _motor4_speed __motor4_speed)) _motor4_speed)
  (:motor5_speed
   (&optional __motor5_speed)
   (if __motor5_speed (setq _motor5_speed __motor5_speed)) _motor5_speed)
  (:motor6_speed
   (&optional __motor6_speed)
   (if __motor6_speed (setq _motor6_speed __motor6_speed)) _motor6_speed)
  (:motor1_current
   (&optional __motor1_current)
   (if __motor1_current (setq _motor1_current __motor1_current)) _motor1_current)
  (:motor2_current
   (&optional __motor2_current)
   (if __motor2_current (setq _motor2_current __motor2_current)) _motor2_current)
  (:motor3_current
   (&optional __motor3_current)
   (if __motor3_current (setq _motor3_current __motor3_current)) _motor3_current)
  (:motor4_current
   (&optional __motor4_current)
   (if __motor4_current (setq _motor4_current __motor4_current)) _motor4_current)
  (:motor5_current
   (&optional __motor5_current)
   (if __motor5_current (setq _motor5_current __motor5_current)) _motor5_current)
  (:motor6_current
   (&optional __motor6_current)
   (if __motor6_current (setq _motor6_current __motor6_current)) _motor6_current)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:serialization-length
   ()
   (+
    ;; int16 _linearspeed
    2
    ;; int16 _motor1_speed
    2
    ;; int16 _motor2_speed
    2
    ;; int16 _motor3_speed
    2
    ;; int16 _motor4_speed
    2
    ;; int16 _motor5_speed
    2
    ;; int16 _motor6_speed
    2
    ;; int16 _motor1_current
    2
    ;; int16 _motor2_current
    2
    ;; int16 _motor3_current
    2
    ;; int16 _motor4_current
    2
    ;; int16 _motor5_current
    2
    ;; int16 _motor6_current
    2
    ;; int16 _voltage
    2
    ;; int16 _temperature
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _linearspeed
       (write-word _linearspeed s)
     ;; int16 _motor1_speed
       (write-word _motor1_speed s)
     ;; int16 _motor2_speed
       (write-word _motor2_speed s)
     ;; int16 _motor3_speed
       (write-word _motor3_speed s)
     ;; int16 _motor4_speed
       (write-word _motor4_speed s)
     ;; int16 _motor5_speed
       (write-word _motor5_speed s)
     ;; int16 _motor6_speed
       (write-word _motor6_speed s)
     ;; int16 _motor1_current
       (write-word _motor1_current s)
     ;; int16 _motor2_current
       (write-word _motor2_current s)
     ;; int16 _motor3_current
       (write-word _motor3_current s)
     ;; int16 _motor4_current
       (write-word _motor4_current s)
     ;; int16 _motor5_current
       (write-word _motor5_current s)
     ;; int16 _motor6_current
       (write-word _motor6_current s)
     ;; int16 _voltage
       (write-word _voltage s)
     ;; int16 _temperature
       (write-word _temperature s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _linearspeed
     (setq _linearspeed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor1_speed
     (setq _motor1_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor2_speed
     (setq _motor2_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor3_speed
     (setq _motor3_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor4_speed
     (setq _motor4_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor5_speed
     (setq _motor5_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor6_speed
     (setq _motor6_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor1_current
     (setq _motor1_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor2_current
     (setq _motor2_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor3_current
     (setq _motor3_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor4_current
     (setq _motor4_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor5_current
     (setq _motor5_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _motor6_current
     (setq _motor6_current (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _voltage
     (setq _voltage (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _temperature
     (setq _temperature (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get sixwd_msgs::SixWheelInfo :md5sum-) "c4dc9fd6e27eb023af6f7e522d9e61ed")
(setf (get sixwd_msgs::SixWheelInfo :datatype-) "sixwd_msgs/SixWheelInfo")
(setf (get sixwd_msgs::SixWheelInfo :definition-)
      "
int16 linearspeed   #Linear Speed or right or left speed with angle
int16 motor1_speed #Individual Speed info from each motor
int16 motor2_speed
int16 motor3_speed
int16 motor4_speed
int16 motor5_speed
int16 motor6_speed
int16 motor1_current
int16 motor2_current
int16 motor3_current
int16 motor4_current
int16 motor5_current
int16 motor6_current
int16 voltage  #Battery voltage reading and temperature
int16 temperature
")



(provide :sixwd_msgs/SixWheelInfo "c4dc9fd6e27eb023af6f7e522d9e61ed")


