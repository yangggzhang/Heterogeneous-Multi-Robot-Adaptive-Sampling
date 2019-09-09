
(cl:in-package :asdf)

(defsystem "temp-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "temp_odom" :depends-on ("_package_temp_odom"))
    (:file "_package_temp_odom" :depends-on ("_package"))
  ))