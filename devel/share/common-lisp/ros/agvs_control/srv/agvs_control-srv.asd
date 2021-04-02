
(cl:in-package :asdf)

(defsystem "agvs_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmd_control_mode" :depends-on ("_package_cmd_control_mode"))
    (:file "_package_cmd_control_mode" :depends-on ("_package"))
    (:file "cmd_control_task" :depends-on ("_package_cmd_control_task"))
    (:file "_package_cmd_control_task" :depends-on ("_package"))
  ))