; Auto-generated. Do not edit!


(cl:in-package reinforcement_learning-msg)


;//! \htmlinclude EpisodeResult.msg.html

(cl:defclass <EpisodeResult> (roslisp-msg-protocol:ros-message)
  ((reward
    :reader reward
    :initarg :reward
    :type cl:float
    :initform 0.0)
   (length
    :reader length
    :initarg :length
    :type cl:integer
    :initform 0))
)

(cl:defclass EpisodeResult (<EpisodeResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EpisodeResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EpisodeResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name reinforcement_learning-msg:<EpisodeResult> is deprecated: use reinforcement_learning-msg:EpisodeResult instead.")))

(cl:ensure-generic-function 'reward-val :lambda-list '(m))
(cl:defmethod reward-val ((m <EpisodeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reinforcement_learning-msg:reward-val is deprecated.  Use reinforcement_learning-msg:reward instead.")
  (reward m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <EpisodeResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader reinforcement_learning-msg:length-val is deprecated.  Use reinforcement_learning-msg:length instead.")
  (length m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EpisodeResult>) ostream)
  "Serializes a message object of type '<EpisodeResult>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'reward))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'length)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EpisodeResult>) istream)
  "Deserializes a message object of type '<EpisodeResult>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'reward) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'length) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EpisodeResult>)))
  "Returns string type for a message object of type '<EpisodeResult>"
  "reinforcement_learning/EpisodeResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EpisodeResult)))
  "Returns string type for a message object of type 'EpisodeResult"
  "reinforcement_learning/EpisodeResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EpisodeResult>)))
  "Returns md5sum for a message object of type '<EpisodeResult>"
  "fa4f4b21e1c3537a9bf4795f8c325893")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EpisodeResult)))
  "Returns md5sum for a message object of type 'EpisodeResult"
  "fa4f4b21e1c3537a9bf4795f8c325893")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EpisodeResult>)))
  "Returns full string definition for message of type '<EpisodeResult>"
  (cl:format cl:nil "float32 reward~%int32 length~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EpisodeResult)))
  "Returns full string definition for message of type 'EpisodeResult"
  (cl:format cl:nil "float32 reward~%int32 length~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EpisodeResult>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EpisodeResult>))
  "Converts a ROS message object to a list"
  (cl:list 'EpisodeResult
    (cl:cons ':reward (reward msg))
    (cl:cons ':length (length msg))
))
