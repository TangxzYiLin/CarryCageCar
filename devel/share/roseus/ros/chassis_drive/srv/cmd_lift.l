;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::cmd_lift)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'cmd_lift (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::CMD_LIFT")
  (make-package "CHASSIS_DRIVE::CMD_LIFT"))
(unless (find-package "CHASSIS_DRIVE::CMD_LIFTREQUEST")
  (make-package "CHASSIS_DRIVE::CMD_LIFTREQUEST"))
(unless (find-package "CHASSIS_DRIVE::CMD_LIFTRESPONSE")
  (make-package "CHASSIS_DRIVE::CMD_LIFTRESPONSE"))

(in-package "ROS")





(intern "*MODE_UP*" (find-package "CHASSIS_DRIVE::CMD_LIFTREQUEST"))
(shadow '*MODE_UP* (find-package "CHASSIS_DRIVE::CMD_LIFTREQUEST"))
(defconstant chassis_drive::cmd_liftRequest::*MODE_UP* 1)
(intern "*MODE_DOWN*" (find-package "CHASSIS_DRIVE::CMD_LIFTREQUEST"))
(shadow '*MODE_DOWN* (find-package "CHASSIS_DRIVE::CMD_LIFTREQUEST"))
(defconstant chassis_drive::cmd_liftRequest::*MODE_DOWN* 0)
(defclass chassis_drive::cmd_liftRequest
  :super ros::object
  :slots (_lift_active ))

(defmethod chassis_drive::cmd_liftRequest
  (:init
   (&key
    ((:lift_active __lift_active) 0)
    )
   (send-super :init)
   (setq _lift_active (round __lift_active))
   self)
  (:lift_active
   (&optional __lift_active)
   (if __lift_active (setq _lift_active __lift_active)) _lift_active)
  (:serialization-length
   ()
   (+
    ;; uint16 _lift_active
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _lift_active
       (write-word _lift_active s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _lift_active
     (setq _lift_active (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass chassis_drive::cmd_liftResponse
  :super ros::object
  :slots ())

(defmethod chassis_drive::cmd_liftResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass chassis_drive::cmd_lift
  :super ros::object
  :slots ())

(setf (get chassis_drive::cmd_lift :md5sum-) "3ebadb64f1f7323f82c99c510cfe6f5c")
(setf (get chassis_drive::cmd_lift :datatype-) "chassis_drive/cmd_lift")
(setf (get chassis_drive::cmd_lift :request) chassis_drive::cmd_liftRequest)
(setf (get chassis_drive::cmd_lift :response) chassis_drive::cmd_liftResponse)

(defmethod chassis_drive::cmd_liftRequest
  (:response () (instance chassis_drive::cmd_liftResponse :init)))

(setf (get chassis_drive::cmd_liftRequest :md5sum-) "3ebadb64f1f7323f82c99c510cfe6f5c")
(setf (get chassis_drive::cmd_liftRequest :datatype-) "chassis_drive/cmd_liftRequest")
(setf (get chassis_drive::cmd_liftRequest :definition-)
      "uint16 lift_active
uint16 mode_up=1
uint16 mode_down=0
---
")

(setf (get chassis_drive::cmd_liftResponse :md5sum-) "3ebadb64f1f7323f82c99c510cfe6f5c")
(setf (get chassis_drive::cmd_liftResponse :datatype-) "chassis_drive/cmd_liftResponse")
(setf (get chassis_drive::cmd_liftResponse :definition-)
      "uint16 lift_active
uint16 mode_up=1
uint16 mode_down=0
---
")



(provide :chassis_drive/cmd_lift "3ebadb64f1f7323f82c99c510cfe6f5c")


