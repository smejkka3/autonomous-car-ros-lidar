
(cl:in-package :asdf)

(defsystem "rc_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "RCControlMsg" :depends-on ("_package_RCControlMsg"))
    (:file "_package_RCControlMsg" :depends-on ("_package"))
  ))