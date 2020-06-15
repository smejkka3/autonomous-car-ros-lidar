; Auto-generated. Do not edit!


(cl:in-package sixwd_msgs-msg)


;//! \htmlinclude SixWheelCommand.msg.html

(cl:defclass <SixWheelCommand> (roslisp-msg-protocol:ros-message)
  ((controltype
    :reader controltype
    :initarg :controltype
    :type cl:fixnum
    :initform 0)
   (linearspeed
    :reader linearspeed
    :initarg :linearspeed
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:fixnum
    :initform 0)
   (motor_number
    :reader motor_number
    :initarg :motor_number
    :type cl:fixnum
    :initform 0)
   (individual_motors_speed
    :reader individual_motors_speed
    :initarg :individual_motors_speed
    :type cl:fixnum
    :initform 0)
   (right_speed
    :reader right_speed
    :initarg :right_speed
    :type cl:fixnum
    :initform 0)
   (left_speed
    :reader left_speed
    :initarg :left_speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SixWheelCommand (<SixWheelCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SixWheelCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SixWheelCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sixwd_msgs-msg:<SixWheelCommand> is deprecated: use sixwd_msgs-msg:SixWheelCommand instead.")))

(cl:ensure-generic-function 'controltype-val :lambda-list '(m))
(cl:defmethod controltype-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:controltype-val is deprecated.  Use sixwd_msgs-msg:controltype instead.")
  (controltype m))

(cl:ensure-generic-function 'linearspeed-val :lambda-list '(m))
(cl:defmethod linearspeed-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:linearspeed-val is deprecated.  Use sixwd_msgs-msg:linearspeed instead.")
  (linearspeed m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:angle-val is deprecated.  Use sixwd_msgs-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'motor_number-val :lambda-list '(m))
(cl:defmethod motor_number-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor_number-val is deprecated.  Use sixwd_msgs-msg:motor_number instead.")
  (motor_number m))

(cl:ensure-generic-function 'individual_motors_speed-val :lambda-list '(m))
(cl:defmethod individual_motors_speed-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:individual_motors_speed-val is deprecated.  Use sixwd_msgs-msg:individual_motors_speed instead.")
  (individual_motors_speed m))

(cl:ensure-generic-function 'right_speed-val :lambda-list '(m))
(cl:defmethod right_speed-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:right_speed-val is deprecated.  Use sixwd_msgs-msg:right_speed instead.")
  (right_speed m))

(cl:ensure-generic-function 'left_speed-val :lambda-list '(m))
(cl:defmethod left_speed-val ((m <SixWheelCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:left_speed-val is deprecated.  Use sixwd_msgs-msg:left_speed instead.")
  (left_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SixWheelCommand>) ostream)
  "Serializes a message object of type '<SixWheelCommand>"
  (cl:let* ((signed (cl:slot-value msg 'controltype)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'linearspeed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_number)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'individual_motors_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'left_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SixWheelCommand>) istream)
  "Deserializes a message object of type '<SixWheelCommand>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controltype) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'linearspeed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motor_number)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'individual_motors_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SixWheelCommand>)))
  "Returns string type for a message object of type '<SixWheelCommand>"
  "sixwd_msgs/SixWheelCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SixWheelCommand)))
  "Returns string type for a message object of type 'SixWheelCommand"
  "sixwd_msgs/SixWheelCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SixWheelCommand>)))
  "Returns md5sum for a message object of type '<SixWheelCommand>"
  "8eb7be4689f84a603224726e6da73ba6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SixWheelCommand)))
  "Returns md5sum for a message object of type 'SixWheelCommand"
  "8eb7be4689f84a603224726e6da73ba6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SixWheelCommand>)))
  "Returns full string definition for message of type '<SixWheelCommand>"
  (cl:format cl:nil "int16 controltype  #For individual control send 0 for linear control send 1~%int16 linearspeed  #Linear Speed or right or left speed with angle and what to command~%int16 angle~%uint8 motor_number #Select motor ~%int16 individual_motors_speed #İndividual speed commands for each motor~%int16 right_speed~%int16 left_speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SixWheelCommand)))
  "Returns full string definition for message of type 'SixWheelCommand"
  (cl:format cl:nil "int16 controltype  #For individual control send 0 for linear control send 1~%int16 linearspeed  #Linear Speed or right or left speed with angle and what to command~%int16 angle~%uint8 motor_number #Select motor ~%int16 individual_motors_speed #İndividual speed commands for each motor~%int16 right_speed~%int16 left_speed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SixWheelCommand>))
  (cl:+ 0
     2
     2
     2
     1
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SixWheelCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'SixWheelCommand
    (cl:cons ':controltype (controltype msg))
    (cl:cons ':linearspeed (linearspeed msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':motor_number (motor_number msg))
    (cl:cons ':individual_motors_speed (individual_motors_speed msg))
    (cl:cons ':right_speed (right_speed msg))
    (cl:cons ':left_speed (left_speed msg))
))
