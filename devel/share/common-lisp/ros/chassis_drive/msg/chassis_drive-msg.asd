
(cl:in-package :asdf)

(defsystem "chassis_drive-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :agvs_task-msg
)
  :components ((:file "_package")
    (:file "agvs_led_state" :depends-on ("_package_agvs_led_state"))
    (:file "_package_agvs_led_state" :depends-on ("_package"))
    (:file "agvs_mode" :depends-on ("_package_agvs_mode"))
    (:file "_package_agvs_mode" :depends-on ("_package"))
    (:file "agvs_test" :depends-on ("_package_agvs_test"))
    (:file "_package_agvs_test" :depends-on ("_package"))
    (:file "chassis_alarm" :depends-on ("_package_chassis_alarm"))
    (:file "_package_chassis_alarm" :depends-on ("_package"))
    (:file "chassis_bat" :depends-on ("_package_chassis_bat"))
    (:file "_package_chassis_bat" :depends-on ("_package"))
    (:file "chassis_cmd" :depends-on ("_package_chassis_cmd"))
    (:file "_package_chassis_cmd" :depends-on ("_package"))
    (:file "chassis_state" :depends-on ("_package_chassis_state"))
    (:file "_package_chassis_state" :depends-on ("_package"))
  ))