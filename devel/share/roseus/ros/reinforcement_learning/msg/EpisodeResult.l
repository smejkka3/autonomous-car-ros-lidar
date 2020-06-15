;; Auto-generated. Do not edit!


(when (boundp 'reinforcement_learning::EpisodeResult)
  (if (not (find-package "REINFORCEMENT_LEARNING"))
    (make-package "REINFORCEMENT_LEARNING"))
  (shadow 'EpisodeResult (find-package "REINFORCEMENT_LEARNING")))
(unless (find-package "REINFORCEMENT_LEARNING::EPISODERESULT")
  (make-package "REINFORCEMENT_LEARNING::EPISODERESULT"))

(in-package "ROS")
;;//! \htmlinclude EpisodeResult.msg.html


(defclass reinforcement_learning::EpisodeResult
  :super ros::object
  :slots (_reward _length ))

(defmethod reinforcement_learning::EpisodeResult
  (:init
   (&key
    ((:reward __reward) 0.0)
    ((:length __length) 0)
    )
   (send-super :init)
   (setq _reward (float __reward))
   (setq _length (round __length))
   self)
  (:reward
   (&optional __reward)
   (if __reward (setq _reward __reward)) _reward)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:serialization-length
   ()
   (+
    ;; float32 _reward
    4
    ;; int32 _length
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _reward
       (sys::poke _reward (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _length
       (write-long _length s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _reward
     (setq _reward (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _length
     (setq _length (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get reinforcement_learning::EpisodeResult :md5sum-) "fa4f4b21e1c3537a9bf4795f8c325893")
(setf (get reinforcement_learning::EpisodeResult :datatype-) "reinforcement_learning/EpisodeResult")
(setf (get reinforcement_learning::EpisodeResult :definition-)
      "float32 reward
int32 length
")



(provide :reinforcement_learning/EpisodeResult "fa4f4b21e1c3537a9bf4795f8c325893")


