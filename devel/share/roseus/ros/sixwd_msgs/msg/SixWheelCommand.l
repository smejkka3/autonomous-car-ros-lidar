;; Auto-generated. Do not edit!


(when (boundp 'sixwd_msgs::SixWheelCommand)
  (if (not (find-package "SIXWD_MSGS"))
    (make-package "SIXWD_MSGS"))
  (shadow 'SixWheelCommand (find-package "SIXWD_MSGS")))
(unless (find-package "SIXWD_MSGS::SIXWHEELCOMMAND")
  (make-package "SIXWD_MSGS::SIXWHEELCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude SixWheelCommand.msg.html


(defclass sixwd_msgs::SixWheelCommand
  :super ros::object
  :slots (_controltype _linearspeed _angle _motor_number _individual_motors_speed _right_speed _left_speed ))

(defmethod sixwd_msgs::SixWheelCommand
  (:init
   (&key
    ((:controltype __controltype) 0)
    ((:linearspeed __linearspeed) 0)
    ((:angle __angle) 0)
    ((:motor_number __motor_number) 0)
    ((:individual_motors_speed __individual_motors_speed) 0)
    ((:right_speed __right_speed) 0)
    ((:left_speed __left_speed) 0)
    )
   (send-super :init)
   (setq _controltype (round __controltype))
   (setq _linearspeed (round __linearspeed))
   (setq _angle (round __angle))
   (setq _motor_number (round __motor_number))
   (setq _individual_motors_speed (round __individual_motors_speed))
   (setq _right_speed (round __right_speed))
   (setq _left_speed (round __left_speed))
   self)
  (:controltype
   (&optional __controltype)
   (if __controltype (setq _controltype __controltype)) _controltype)
  (:linearspeed
   (&optional __linearspeed)
   (if __linearspeed (setq _linearspeed __linearspeed)) _linearspeed)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:motor_number
   (&optional __motor_number)
   (if __motor_number (setq _motor_number __motor_number)) _motor_number)
  (:individual_motors_speed
   (&optional __individual_motors_speed)
   (if __individual_motors_speed (setq _individual_motors_speed __individual_motors_speed)) _individual_motors_speed)
  (:right_speed
   (&optional __right_speed)
   (if __right_speed (setq _right_speed __right_speed)) _right_speed)
  (:left_speed
   (&optional __left_speed)
   (if __left_speed (setq _left_speed __left_speed)) _left_speed)
  (:serialization-length
   ()
   (+
    ;; int16 _controltype
    2
    ;; int16 _linearspeed
    2
    ;; int16 _angle
    2
    ;; uint8 _motor_number
    1
    ;; int16 _individual_motors_speed
    2
    ;; int16 _right_speed
    2
    ;; int16 _left_speed
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _controltype
       (write-word _controltype s)
     ;; int16 _linearspeed
       (write-word _linearspeed s)
     ;; int16 _angle
       (write-word _angle s)
     ;; uint8 _motor_number
       (write-byte _motor_number s)
     ;; int16 _individual_motors_speed
       (write-word _individual_motors_speed s)
     ;; int16 _right_speed
       (write-word _right_speed s)
     ;; int16 _left_speed
       (write-word _left_speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _controltype
     (setq _controltype (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _linearspeed
     (setq _linearspeed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _angle
     (setq _angle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _motor_number
     (setq _motor_number (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _individual_motors_speed
     (setq _individual_motors_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _right_speed
     (setq _right_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _left_speed
     (setq _left_speed (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get sixwd_msgs::SixWheelCommand :md5sum-) "8eb7be4689f84a603224726e6da73ba6")
(setf (get sixwd_msgs::SixWheelCommand :datatype-) "sixwd_msgs/SixWheelCommand")
(setf (get sixwd_msgs::SixWheelCommand :definition-)
      "int16 controltype  #For individual control send 0 for linear control send 1
int16 linearspeed  #Linear Speed or right or left speed with angle and what to command
int16 angle
uint8 motor_number #Select motor 
int16 individual_motors_speed #İndividual speed commands for each motor
int16 right_speed
int16 left_speed


")



(provide :sixwd_msgs/SixWheelCommand "8eb7be4689f84a603224726e6da73ba6")


