
(cl:in-package :asdf)

(defsystem "agvs_task-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "route_target" :depends-on ("_package_route_target"))
    (:file "_package_route_target" :depends-on ("_package"))
  ))