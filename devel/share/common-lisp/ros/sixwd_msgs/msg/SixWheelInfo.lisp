; Auto-generated. Do not edit!


(cl:in-package sixwd_msgs-msg)


;//! \htmlinclude SixWheelInfo.msg.html

(cl:defclass <SixWheelInfo> (roslisp-msg-protocol:ros-message)
  ((linearspeed
    :reader linearspeed
    :initarg :linearspeed
    :type cl:fixnum
    :initform 0)
   (motor1_speed
    :reader motor1_speed
    :initarg :motor1_speed
    :type cl:fixnum
    :initform 0)
   (motor2_speed
    :reader motor2_speed
    :initarg :motor2_speed
    :type cl:fixnum
    :initform 0)
   (motor3_speed
    :reader motor3_speed
    :initarg :motor3_speed
    :type cl:fixnum
    :initform 0)
   (motor4_speed
    :reader motor4_speed
    :initarg :motor4_speed
    :type cl:fixnum
    :initform 0)
   (motor5_speed
    :reader motor5_speed
    :initarg :motor5_speed
    :type cl:fixnum
    :initform 0)
   (motor6_speed
    :reader motor6_speed
    :initarg :motor6_speed
    :type cl:fixnum
    :initform 0)
   (motor1_current
    :reader motor1_current
    :initarg :motor1_current
    :type cl:fixnum
    :initform 0)
   (motor2_current
    :reader motor2_current
    :initarg :motor2_current
    :type cl:fixnum
    :initform 0)
   (motor3_current
    :reader motor3_current
    :initarg :motor3_current
    :type cl:fixnum
    :initform 0)
   (motor4_current
    :reader motor4_current
    :initarg :motor4_current
    :type cl:fixnum
    :initform 0)
   (motor5_current
    :reader motor5_current
    :initarg :motor5_current
    :type cl:fixnum
    :initform 0)
   (motor6_current
    :reader motor6_current
    :initarg :motor6_current
    :type cl:fixnum
    :initform 0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:fixnum
    :initform 0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SixWheelInfo (<SixWheelInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SixWheelInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SixWheelInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sixwd_msgs-msg:<SixWheelInfo> is deprecated: use sixwd_msgs-msg:SixWheelInfo instead.")))

(cl:ensure-generic-function 'linearspeed-val :lambda-list '(m))
(cl:defmethod linearspeed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:linearspeed-val is deprecated.  Use sixwd_msgs-msg:linearspeed instead.")
  (linearspeed m))

(cl:ensure-generic-function 'motor1_speed-val :lambda-list '(m))
(cl:defmethod motor1_speed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor1_speed-val is deprecated.  Use sixwd_msgs-msg:motor1_speed instead.")
  (motor1_speed m))

(cl:ensure-generic-function 'motor2_speed-val :lambda-list '(m))
(cl:defmethod motor2_speed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor2_speed-val is deprecated.  Use sixwd_msgs-msg:motor2_speed instead.")
  (motor2_speed m))

(cl:ensure-generic-function 'motor3_speed-val :lambda-list '(m))
(cl:defmethod motor3_speed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor3_speed-val is deprecated.  Use sixwd_msgs-msg:motor3_speed instead.")
  (motor3_speed m))

(cl:ensure-generic-function 'motor4_speed-val :lambda-list '(m))
(cl:defmethod motor4_speed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor4_speed-val is deprecated.  Use sixwd_msgs-msg:motor4_speed instead.")
  (motor4_speed m))

(cl:ensure-generic-function 'motor5_speed-val :lambda-list '(m))
(cl:defmethod motor5_speed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor5_speed-val is deprecated.  Use sixwd_msgs-msg:motor5_speed instead.")
  (motor5_speed m))

(cl:ensure-generic-function 'motor6_speed-val :lambda-list '(m))
(cl:defmethod motor6_speed-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor6_speed-val is deprecated.  Use sixwd_msgs-msg:motor6_speed instead.")
  (motor6_speed m))

(cl:ensure-generic-function 'motor1_current-val :lambda-list '(m))
(cl:defmethod motor1_current-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor1_current-val is deprecated.  Use sixwd_msgs-msg:motor1_current instead.")
  (motor1_current m))

(cl:ensure-generic-function 'motor2_current-val :lambda-list '(m))
(cl:defmethod motor2_current-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor2_current-val is deprecated.  Use sixwd_msgs-msg:motor2_current instead.")
  (motor2_current m))

(cl:ensure-generic-function 'motor3_current-val :lambda-list '(m))
(cl:defmethod motor3_current-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor3_current-val is deprecated.  Use sixwd_msgs-msg:motor3_current instead.")
  (motor3_current m))

(cl:ensure-generic-function 'motor4_current-val :lambda-list '(m))
(cl:defmethod motor4_current-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor4_current-val is deprecated.  Use sixwd_msgs-msg:motor4_current instead.")
  (motor4_current m))

(cl:ensure-generic-function 'motor5_current-val :lambda-list '(m))
(cl:defmethod motor5_current-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor5_current-val is deprecated.  Use sixwd_msgs-msg:motor5_current instead.")
  (motor5_current m))

(cl:ensure-generic-function 'motor6_current-val :lambda-list '(m))
(cl:defmethod motor6_current-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:motor6_current-val is deprecated.  Use sixwd_msgs-msg:motor6_current instead.")
  (motor6_current m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:voltage-val is deprecated.  Use sixwd_msgs-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <SixWheelInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sixwd_msgs-msg:temperature-val is deprecated.  Use sixwd_msgs-msg:temperature instead.")
  (temperature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SixWheelInfo>) ostream)
  "Serializes a message object of type '<SixWheelInfo>"
  (cl:let* ((signed (cl:slot-value msg 'linearspeed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor1_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor2_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor3_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor4_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor5_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor6_speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor1_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor2_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor3_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor4_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor5_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'motor6_current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'voltage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'temperature)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SixWheelInfo>) istream)
  "Deserializes a message object of type '<SixWheelInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'linearspeed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor1_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor2_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor3_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor4_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor5_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor6_speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor1_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor2_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor3_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor4_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor5_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motor6_current) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'voltage) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temperature) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SixWheelInfo>)))
  "Returns string type for a message object of type '<SixWheelInfo>"
  "sixwd_msgs/SixWheelInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SixWheelInfo)))
  "Returns string type for a message object of type 'SixWheelInfo"
  "sixwd_msgs/SixWheelInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SixWheelInfo>)))
  "Returns md5sum for a message object of type '<SixWheelInfo>"
  "c4dc9fd6e27eb023af6f7e522d9e61ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SixWheelInfo)))
  "Returns md5sum for a message object of type 'SixWheelInfo"
  "c4dc9fd6e27eb023af6f7e522d9e61ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SixWheelInfo>)))
  "Returns full string definition for message of type '<SixWheelInfo>"
  (cl:format cl:nil "~%int16 linearspeed   #Linear Speed or right or left speed with angle~%int16 motor1_speed #Individual Speed info from each motor~%int16 motor2_speed~%int16 motor3_speed~%int16 motor4_speed~%int16 motor5_speed~%int16 motor6_speed~%int16 motor1_current~%int16 motor2_current~%int16 motor3_current~%int16 motor4_current~%int16 motor5_current~%int16 motor6_current~%int16 voltage  #Battery voltage reading and temperature~%int16 temperature~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SixWheelInfo)))
  "Returns full string definition for message of type 'SixWheelInfo"
  (cl:format cl:nil "~%int16 linearspeed   #Linear Speed or right or left speed with angle~%int16 motor1_speed #Individual Speed info from each motor~%int16 motor2_speed~%int16 motor3_speed~%int16 motor4_speed~%int16 motor5_speed~%int16 motor6_speed~%int16 motor1_current~%int16 motor2_current~%int16 motor3_current~%int16 motor4_current~%int16 motor5_current~%int16 motor6_current~%int16 voltage  #Battery voltage reading and temperature~%int16 temperature~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SixWheelInfo>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SixWheelInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'SixWheelInfo
    (cl:cons ':linearspeed (linearspeed msg))
    (cl:cons ':motor1_speed (motor1_speed msg))
    (cl:cons ':motor2_speed (motor2_speed msg))
    (cl:cons ':motor3_speed (motor3_speed msg))
    (cl:cons ':motor4_speed (motor4_speed msg))
    (cl:cons ':motor5_speed (motor5_speed msg))
    (cl:cons ':motor6_speed (motor6_speed msg))
    (cl:cons ':motor1_current (motor1_current msg))
    (cl:cons ':motor2_current (motor2_current msg))
    (cl:cons ':motor3_current (motor3_current msg))
    (cl:cons ':motor4_current (motor4_current msg))
    (cl:cons ':motor5_current (motor5_current msg))
    (cl:cons ':motor6_current (motor6_current msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':temperature (temperature msg))
))
