;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::agvs_led_state)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'agvs_led_state (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::AGVS_LED_STATE")
  (make-package "CHASSIS_DRIVE::AGVS_LED_STATE"))

(in-package "ROS")
;;//! \htmlinclude agvs_led_state.msg.html


(intern "*CHASSIS_LED_STATE_ALL_OFF*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_ALL_OFF* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_ALL_OFF* 0)
(intern "*CHASSIS_LED_STATE_RED_ON*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_RED_ON* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_RED_ON* 1)
(intern "*CHASSIS_LED_STATE_BULE_ON*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_BULE_ON* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_BULE_ON* 2)
(intern "*CHASSIS_LED_STATE_GREEN_ON*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_GREEN_ON* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_GREEN_ON* 3)
(intern "*CHASSIS_LED_STATE_RED_FLASH*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_RED_FLASH* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_RED_FLASH* 4)
(intern "*CHASSIS_LED_STATE_GREEN_FLASH*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_GREEN_FLASH* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_GREEN_FLASH* 5)
(intern "*CHASSIS_LED_STATE_BLUE_FLASH*" (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(shadow '*CHASSIS_LED_STATE_BLUE_FLASH* (find-package "CHASSIS_DRIVE::AGVS_LED_STATE"))
(defconstant chassis_drive::agvs_led_state::*CHASSIS_LED_STATE_BLUE_FLASH* 6)
(defclass chassis_drive::agvs_led_state
  :super ros::object
  :slots (_update_state ))

(defmethod chassis_drive::agvs_led_state
  (:init
   (&key
    ((:update_state __update_state) 0)
    )
   (send-super :init)
   (setq _update_state (round __update_state))
   self)
  (:update_state
   (&optional __update_state)
   (if __update_state (setq _update_state __update_state)) _update_state)
  (:serialization-length
   ()
   (+
    ;; uint8 _update_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _update_state
       (write-byte _update_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _update_state
     (setq _update_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get chassis_drive::agvs_led_state :md5sum-) "82295a76d34cf26c9e6b43410588f16c")
(setf (get chassis_drive::agvs_led_state :datatype-) "chassis_drive/agvs_led_state")
(setf (get chassis_drive::agvs_led_state :definition-)
      "uint8 update_state
       
uint8 chassis_led_state_all_off = 0
uint8 chassis_led_state_red_on  = 1
uint8 chassis_led_state_bule_on = 2
uint8 chassis_led_state_green_on = 3
uint8 chassis_led_state_red_flash = 4
uint8 chassis_led_state_green_flash = 5
uint8 chassis_led_state_blue_flash  = 6
")



(provide :chassis_drive/agvs_led_state "82295a76d34cf26c9e6b43410588f16c")


