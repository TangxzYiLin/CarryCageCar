
(cl:in-package :asdf)

(defsystem "agvs_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "date_pads_cmd" :depends-on ("_package_date_pads_cmd"))
    (:file "_package_date_pads_cmd" :depends-on ("_package"))
    (:file "date_realtime_feedback" :depends-on ("_package_date_realtime_feedback"))
    (:file "_package_date_realtime_feedback" :depends-on ("_package"))
  ))