
(cl:in-package :asdf)

(defsystem "agvs_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "date_pads_cmd" :depends-on ("_package_date_pads_cmd"))
    (:file "_package_date_pads_cmd" :depends-on ("_package"))
    (:file "slam_data" :depends-on ("_package_slam_data"))
    (:file "_package_slam_data" :depends-on ("_package"))
  ))