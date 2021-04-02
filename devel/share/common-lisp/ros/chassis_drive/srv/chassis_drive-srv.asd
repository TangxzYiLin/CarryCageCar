
(cl:in-package :asdf)

(defsystem "chassis_drive-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmd_lift" :depends-on ("_package_cmd_lift"))
    (:file "_package_cmd_lift" :depends-on ("_package"))
  ))