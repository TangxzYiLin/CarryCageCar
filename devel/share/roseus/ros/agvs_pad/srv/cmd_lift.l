;; Auto-generated. Do not edit!


(when (boundp 'agvs_pad::cmd_lift)
  (if (not (find-package "AGVS_PAD"))
    (make-package "AGVS_PAD"))
  (shadow 'cmd_lift (find-package "AGVS_PAD")))
(unless (find-package "AGVS_PAD::CMD_LIFT")
  (make-package "AGVS_PAD::CMD_LIFT"))
(unless (find-package "AGVS_PAD::CMD_LIFTREQUEST")
  (make-package "AGVS_PAD::CMD_LIFTREQUEST"))
(unless (find-package "AGVS_PAD::CMD_LIFTRESPONSE")
  (make-package "AGVS_PAD::CMD_LIFTRESPONSE"))

(in-package "ROS")





(intern "*MODE_UP*" (find-package "AGVS_PAD::CMD_LIFTREQUEST"))
(shadow '*MODE_UP* (find-package "AGVS_PAD::CMD_LIFTREQUEST"))
(defconstant agvs_pad::cmd_liftRequest::*MODE_UP* 1)
(intern "*MODE_DOWN*" (find-package "AGVS_PAD::CMD_LIFTREQUEST"))
(shadow '*MODE_DOWN* (find-package "AGVS_PAD::CMD_LIFTREQUEST"))
(defconstant agvs_pad::cmd_liftRequest::*MODE_DOWN* 0)
(defclass agvs_pad::cmd_liftRequest
  :super ros::object
  :slots (_lift_active ))

(defmethod agvs_pad::cmd_liftRequest
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

(defclass agvs_pad::cmd_liftResponse
  :super ros::object
  :slots ())

(defmethod agvs_pad::cmd_liftResponse
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

(defclass agvs_pad::cmd_lift
  :super ros::object
  :slots ())

(setf (get agvs_pad::cmd_lift :md5sum-) "3ebadb64f1f7323f82c99c510cfe6f5c")
(setf (get agvs_pad::cmd_lift :datatype-) "agvs_pad/cmd_lift")
(setf (get agvs_pad::cmd_lift :request) agvs_pad::cmd_liftRequest)
(setf (get agvs_pad::cmd_lift :response) agvs_pad::cmd_liftResponse)

(defmethod agvs_pad::cmd_liftRequest
  (:response () (instance agvs_pad::cmd_liftResponse :init)))

(setf (get agvs_pad::cmd_liftRequest :md5sum-) "3ebadb64f1f7323f82c99c510cfe6f5c")
(setf (get agvs_pad::cmd_liftRequest :datatype-) "agvs_pad/cmd_liftRequest")
(setf (get agvs_pad::cmd_liftRequest :definition-)
      "uint16 lift_active
uint16 mode_up=1
uint16 mode_down=0
---
")

(setf (get agvs_pad::cmd_liftResponse :md5sum-) "3ebadb64f1f7323f82c99c510cfe6f5c")
(setf (get agvs_pad::cmd_liftResponse :datatype-) "agvs_pad/cmd_liftResponse")
(setf (get agvs_pad::cmd_liftResponse :definition-)
      "uint16 lift_active
uint16 mode_up=1
uint16 mode_down=0
---
")



(provide :agvs_pad/cmd_lift "3ebadb64f1f7323f82c99c510cfe6f5c")


