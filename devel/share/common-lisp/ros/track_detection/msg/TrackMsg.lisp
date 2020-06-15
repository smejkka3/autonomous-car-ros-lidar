; Auto-generated. Do not edit!


(cl:in-package track_detection-msg)


;//! \htmlinclude TrackMsg.msg.html

(cl:defclass <TrackMsg> (roslisp-msg-protocol:ros-message)
  ((leftLength
    :reader leftLength
    :initarg :leftLength
    :type cl:integer
    :initform 0)
   (left
    :reader left
    :initarg :left
    :type (cl:vector track_detection-msg:PointMsg)
   :initform (cl:make-array 0 :element-type 'track_detection-msg:PointMsg :initial-element (cl:make-instance 'track_detection-msg:PointMsg)))
   (rightLength
    :reader rightLength
    :initarg :rightLength
    :type cl:integer
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type (cl:vector track_detection-msg:PointMsg)
   :initform (cl:make-array 0 :element-type 'track_detection-msg:PointMsg :initial-element (cl:make-instance 'track_detection-msg:PointMsg))))
)

(cl:defclass TrackMsg (<TrackMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrackMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrackMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name track_detection-msg:<TrackMsg> is deprecated: use track_detection-msg:TrackMsg instead.")))

(cl:ensure-generic-function 'leftLength-val :lambda-list '(m))
(cl:defmethod leftLength-val ((m <TrackMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_detection-msg:leftLength-val is deprecated.  Use track_detection-msg:leftLength instead.")
  (leftLength m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <TrackMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_detection-msg:left-val is deprecated.  Use track_detection-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'rightLength-val :lambda-list '(m))
(cl:defmethod rightLength-val ((m <TrackMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_detection-msg:rightLength-val is deprecated.  Use track_detection-msg:rightLength instead.")
  (rightLength m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <TrackMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader track_detection-msg:right-val is deprecated.  Use track_detection-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrackMsg>) ostream)
  "Serializes a message object of type '<TrackMsg>"
  (cl:let* ((signed (cl:slot-value msg 'leftLength)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'left))
  (cl:let* ((signed (cl:slot-value msg 'rightLength)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'right))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrackMsg>) istream)
  "Deserializes a message object of type '<TrackMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leftLength) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'left) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'left)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'track_detection-msg:PointMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rightLength) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'right) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'right)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'track_detection-msg:PointMsg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrackMsg>)))
  "Returns string type for a message object of type '<TrackMsg>"
  "track_detection/TrackMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrackMsg)))
  "Returns string type for a message object of type 'TrackMsg"
  "track_detection/TrackMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrackMsg>)))
  "Returns md5sum for a message object of type '<TrackMsg>"
  "cdb50cb692e6a3e0b32b69782a762116")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrackMsg)))
  "Returns md5sum for a message object of type 'TrackMsg"
  "cdb50cb692e6a3e0b32b69782a762116")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrackMsg>)))
  "Returns full string definition for message of type '<TrackMsg>"
  (cl:format cl:nil "# TODO Add track direction~%~%int32 leftLength~%PointMsg[] left~%~%int32 rightLength~%PointMsg[] right~%================================================================================~%MSG: track_detection/PointMsg~%float64 x~%float64 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrackMsg)))
  "Returns full string definition for message of type 'TrackMsg"
  (cl:format cl:nil "# TODO Add track direction~%~%int32 leftLength~%PointMsg[] left~%~%int32 rightLength~%PointMsg[] right~%================================================================================~%MSG: track_detection/PointMsg~%float64 x~%float64 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrackMsg>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'left) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'right) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrackMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'TrackMsg
    (cl:cons ':leftLength (leftLength msg))
    (cl:cons ':left (left msg))
    (cl:cons ':rightLength (rightLength msg))
    (cl:cons ':right (right msg))
))
