; Auto-generated. Do not edit!


(cl:in-package rc_msgs-msg)


;//! \htmlinclude RCControlMsg.msg.html

(cl:defclass <RCControlMsg> (roslisp-msg-protocol:ros-message)
  ((throttle_cmd
    :reader throttle_cmd
    :initarg :throttle_cmd
    :type cl:integer
    :initform 0)
   (steering_cmd
    :reader steering_cmd
    :initarg :steering_cmd
    :type cl:integer
    :initform 0))
)

(cl:defclass RCControlMsg (<RCControlMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RCControlMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RCControlMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rc_msgs-msg:<RCControlMsg> is deprecated: use rc_msgs-msg:RCControlMsg instead.")))

(cl:ensure-generic-function 'throttle_cmd-val :lambda-list '(m))
(cl:defmethod throttle_cmd-val ((m <RCControlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rc_msgs-msg:throttle_cmd-val is deprecated.  Use rc_msgs-msg:throttle_cmd instead.")
  (throttle_cmd m))

(cl:ensure-generic-function 'steering_cmd-val :lambda-list '(m))
(cl:defmethod steering_cmd-val ((m <RCControlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rc_msgs-msg:steering_cmd-val is deprecated.  Use rc_msgs-msg:steering_cmd instead.")
  (steering_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RCControlMsg>) ostream)
  "Serializes a message object of type '<RCControlMsg>"
  (cl:let* ((signed (cl:slot-value msg 'throttle_cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'steering_cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RCControlMsg>) istream)
  "Deserializes a message object of type '<RCControlMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'throttle_cmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering_cmd) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RCControlMsg>)))
  "Returns string type for a message object of type '<RCControlMsg>"
  "rc_msgs/RCControlMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RCControlMsg)))
  "Returns string type for a message object of type 'RCControlMsg"
  "rc_msgs/RCControlMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RCControlMsg>)))
  "Returns md5sum for a message object of type '<RCControlMsg>"
  "dc4cb0d59f80caaf2e977b6feae39f15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RCControlMsg)))
  "Returns md5sum for a message object of type 'RCControlMsg"
  "dc4cb0d59f80caaf2e977b6feae39f15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RCControlMsg>)))
  "Returns full string definition for message of type '<RCControlMsg>"
  (cl:format cl:nil "int32 throttle_cmd~%int32 steering_cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RCControlMsg)))
  "Returns full string definition for message of type 'RCControlMsg"
  (cl:format cl:nil "int32 throttle_cmd~%int32 steering_cmd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RCControlMsg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RCControlMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'RCControlMsg
    (cl:cons ':throttle_cmd (throttle_cmd msg))
    (cl:cons ':steering_cmd (steering_cmd msg))
))
