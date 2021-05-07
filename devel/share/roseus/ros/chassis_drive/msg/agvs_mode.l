;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::agvs_mode)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'agvs_mode (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::AGVS_MODE")
  (make-package "CHASSIS_DRIVE::AGVS_MODE"))

(in-package "ROS")
;;//! \htmlinclude agvs_mode.msg.html


(intern "*AGVS_MODE_MANUAL*" (find-package "CHASSIS_DRIVE::AGVS_MODE"))
(shadow '*AGVS_MODE_MANUAL* (find-package "CHASSIS_DRIVE::AGVS_MODE"))
(defconstant chassis_drive::agvs_mode::*AGVS_MODE_MANUAL* 0)
(intern "*AGVS_MODE_AUTO_FLOOR_1*" (find-package "CHASSIS_DRIVE::AGVS_MODE"))
(shadow '*AGVS_MODE_AUTO_FLOOR_1* (find-package "CHASSIS_DRIVE::AGVS_MODE"))
(defconstant chassis_drive::agvs_mode::*AGVS_MODE_AUTO_FLOOR_1* 1)
(intern "*AGVS_MODE_AUTO_FLOOR_2*" (find-package "CHASSIS_DRIVE::AGVS_MODE"))
(shadow '*AGVS_MODE_AUTO_FLOOR_2* (find-package "CHASSIS_DRIVE::AGVS_MODE"))
(defconstant chassis_drive::agvs_mode::*AGVS_MODE_AUTO_FLOOR_2* 2)
(defclass chassis_drive::agvs_mode
  :super ros::object
  :slots (_current_mode ))

(defmethod chassis_drive::agvs_mode
  (:init
   (&key
    ((:current_mode __current_mode) 0)
    )
   (send-super :init)
   (setq _current_mode (round __current_mode))
   self)
  (:current_mode
   (&optional __current_mode)
   (if __current_mode (setq _current_mode __current_mode)) _current_mode)
  (:serialization-length
   ()
   (+
    ;; uint8 _current_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _current_mode
       (write-byte _current_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _current_mode
     (setq _current_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get chassis_drive::agvs_mode :md5sum-) "334b5039384f49ff4aa9ee0483feb402")
(setf (get chassis_drive::agvs_mode :datatype-) "chassis_drive/agvs_mode")
(setf (get chassis_drive::agvs_mode :definition-)
      "uint8 current_mode

uint8 agvs_mode_manual =0
uint8 agvs_mode_auto_floor_1 = 1
uint8 agvs_mode_auto_floor_2 = 2
")



(provide :chassis_drive/agvs_mode "334b5039384f49ff4aa9ee0483feb402")


