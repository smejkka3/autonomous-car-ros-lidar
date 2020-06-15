
(cl:in-package :asdf)

(defsystem "track_detection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PointMsg" :depends-on ("_package_PointMsg"))
    (:file "_package_PointMsg" :depends-on ("_package"))
    (:file "TrackMsg" :depends-on ("_package_TrackMsg"))
    (:file "_package_TrackMsg" :depends-on ("_package"))
  ))