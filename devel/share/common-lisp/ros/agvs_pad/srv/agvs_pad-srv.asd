
(cl:in-package :asdf)

(defsystem "agvs_pad-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "cmd_lift" :depends-on ("_package_cmd_lift"))
    (:file "_package_cmd_lift" :depends-on ("_package"))
  ))