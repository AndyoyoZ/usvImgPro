
(cl:in-package :asdf)

(defsystem "xx_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Flag_xx" :depends-on ("_package_Flag_xx"))
    (:file "_package_Flag_xx" :depends-on ("_package"))
  ))