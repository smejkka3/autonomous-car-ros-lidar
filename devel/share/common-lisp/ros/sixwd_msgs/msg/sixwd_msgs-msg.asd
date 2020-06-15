
(cl:in-package :asdf)

(defsystem "sixwd_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SixWheelCommand" :depends-on ("_package_SixWheelCommand"))
    (:file "_package_SixWheelCommand" :depends-on ("_package"))
    (:file "SixWheelInfo" :depends-on ("_package_SixWheelInfo"))
    (:file "_package_SixWheelInfo" :depends-on ("_package"))
  ))