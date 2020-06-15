;; Auto-generated. Do not edit!


(when (boundp 'track_detection::TrackMsg)
  (if (not (find-package "TRACK_DETECTION"))
    (make-package "TRACK_DETECTION"))
  (shadow 'TrackMsg (find-package "TRACK_DETECTION")))
(unless (find-package "TRACK_DETECTION::TRACKMSG")
  (make-package "TRACK_DETECTION::TRACKMSG"))

(in-package "ROS")
;;//! \htmlinclude TrackMsg.msg.html


(defclass track_detection::TrackMsg
  :super ros::object
  :slots (_leftLength _left _rightLength _right ))

(defmethod track_detection::TrackMsg
  (:init
   (&key
    ((:leftLength __leftLength) 0)
    ((:left __left) (let (r) (dotimes (i 0) (push (instance track_detection::PointMsg :init) r)) r))
    ((:rightLength __rightLength) 0)
    ((:right __right) (let (r) (dotimes (i 0) (push (instance track_detection::PointMsg :init) r)) r))
    )
   (send-super :init)
   (setq _leftLength (round __leftLength))
   (setq _left __left)
   (setq _rightLength (round __rightLength))
   (setq _right __right)
   self)
  (:leftLength
   (&optional __leftLength)
   (if __leftLength (setq _leftLength __leftLength)) _leftLength)
  (:left
   (&rest __left)
   (if (keywordp (car __left))
       (send* _left __left)
     (progn
       (if __left (setq _left (car __left)))
       _left)))
  (:rightLength
   (&optional __rightLength)
   (if __rightLength (setq _rightLength __rightLength)) _rightLength)
  (:right
   (&rest __right)
   (if (keywordp (car __right))
       (send* _right __right)
     (progn
       (if __right (setq _right (car __right)))
       _right)))
  (:serialization-length
   ()
   (+
    ;; int32 _leftLength
    4
    ;; track_detection/PointMsg[] _left
    (apply #'+ (send-all _left :serialization-length)) 4
    ;; int32 _rightLength
    4
    ;; track_detection/PointMsg[] _right
    (apply #'+ (send-all _right :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _leftLength
       (write-long _leftLength s)
     ;; track_detection/PointMsg[] _left
     (write-long (length _left) s)
     (dolist (elem _left)
       (send elem :serialize s)
       )
     ;; int32 _rightLength
       (write-long _rightLength s)
     ;; track_detection/PointMsg[] _right
     (write-long (length _right) s)
     (dolist (elem _right)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _leftLength
     (setq _leftLength (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; track_detection/PointMsg[] _left
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left (let (r) (dotimes (i n) (push (instance track_detection::PointMsg :init) r)) r))
     (dolist (elem- _left)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int32 _rightLength
     (setq _rightLength (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; track_detection/PointMsg[] _right
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right (let (r) (dotimes (i n) (push (instance track_detection::PointMsg :init) r)) r))
     (dolist (elem- _right)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get track_detection::TrackMsg :md5sum-) "cdb50cb692e6a3e0b32b69782a762116")
(setf (get track_detection::TrackMsg :datatype-) "track_detection/TrackMsg")
(setf (get track_detection::TrackMsg :definition-)
      "# TODO Add track direction

int32 leftLength
PointMsg[] left

int32 rightLength
PointMsg[] right
================================================================================
MSG: track_detection/PointMsg
float64 x
float64 y
")



(provide :track_detection/TrackMsg "cdb50cb692e6a3e0b32b69782a762116")


