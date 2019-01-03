
(cl:in-package :asdf)

(defsystem "xx_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Flag" :depends-on ("_package_Flag"))
    (:file "_package_Flag" :depends-on ("_package"))
    (:file "Res" :depends-on ("_package_Res"))
    (:file "_package_Res" :depends-on ("_package"))
  ))