;; Auto-generated. Do not edit!


(when (boundp 'rc_msgs::RCControlMsg)
  (if (not (find-package "RC_MSGS"))
    (make-package "RC_MSGS"))
  (shadow 'RCControlMsg (find-package "RC_MSGS")))
(unless (find-package "RC_MSGS::RCCONTROLMSG")
  (make-package "RC_MSGS::RCCONTROLMSG"))

(in-package "ROS")
;;//! \htmlinclude RCControlMsg.msg.html


(defclass rc_msgs::RCControlMsg
  :super ros::object
  :slots (_throttle_cmd _steering_cmd ))

(defmethod rc_msgs::RCControlMsg
  (:init
   (&key
    ((:throttle_cmd __throttle_cmd) 0)
    ((:steering_cmd __steering_cmd) 0)
    )
   (send-super :init)
   (setq _throttle_cmd (round __throttle_cmd))
   (setq _steering_cmd (round __steering_cmd))
   self)
  (:throttle_cmd
   (&optional __throttle_cmd)
   (if __throttle_cmd (setq _throttle_cmd __throttle_cmd)) _throttle_cmd)
  (:steering_cmd
   (&optional __steering_cmd)
   (if __steering_cmd (setq _steering_cmd __steering_cmd)) _steering_cmd)
  (:serialization-length
   ()
   (+
    ;; int32 _throttle_cmd
    4
    ;; int32 _steering_cmd
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _throttle_cmd
       (write-long _throttle_cmd s)
     ;; int32 _steering_cmd
       (write-long _steering_cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _throttle_cmd
     (setq _throttle_cmd (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _steering_cmd
     (setq _steering_cmd (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rc_msgs::RCControlMsg :md5sum-) "dc4cb0d59f80caaf2e977b6feae39f15")
(setf (get rc_msgs::RCControlMsg :datatype-) "rc_msgs/RCControlMsg")
(setf (get rc_msgs::RCControlMsg :definition-)
      "int32 throttle_cmd
int32 steering_cmd
")



(provide :rc_msgs/RCControlMsg "dc4cb0d59f80caaf2e977b6feae39f15")


