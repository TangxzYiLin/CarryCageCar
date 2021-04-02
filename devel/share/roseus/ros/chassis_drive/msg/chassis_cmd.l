;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::chassis_cmd)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'chassis_cmd (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::CHASSIS_CMD")
  (make-package "CHASSIS_DRIVE::CHASSIS_CMD"))

(in-package "ROS")
;;//! \htmlinclude chassis_cmd.msg.html


(defclass chassis_drive::chassis_cmd
  :super ros::object
  :slots (_chassis_vel_cmd_ _chassis_angle_cmd_ _chassis_indicator_cmd_ _chassis_brake_cmd_ ))

(defmethod chassis_drive::chassis_cmd
  (:init
   (&key
    ((:chassis_vel_cmd_ __chassis_vel_cmd_) 0)
    ((:chassis_angle_cmd_ __chassis_angle_cmd_) 0)
    ((:chassis_indicator_cmd_ __chassis_indicator_cmd_) 0)
    ((:chassis_brake_cmd_ __chassis_brake_cmd_) 0)
    )
   (send-super :init)
   (setq _chassis_vel_cmd_ (round __chassis_vel_cmd_))
   (setq _chassis_angle_cmd_ (round __chassis_angle_cmd_))
   (setq _chassis_indicator_cmd_ (round __chassis_indicator_cmd_))
   (setq _chassis_brake_cmd_ (round __chassis_brake_cmd_))
   self)
  (:chassis_vel_cmd_
   (&optional __chassis_vel_cmd_)
   (if __chassis_vel_cmd_ (setq _chassis_vel_cmd_ __chassis_vel_cmd_)) _chassis_vel_cmd_)
  (:chassis_angle_cmd_
   (&optional __chassis_angle_cmd_)
   (if __chassis_angle_cmd_ (setq _chassis_angle_cmd_ __chassis_angle_cmd_)) _chassis_angle_cmd_)
  (:chassis_indicator_cmd_
   (&optional __chassis_indicator_cmd_)
   (if __chassis_indicator_cmd_ (setq _chassis_indicator_cmd_ __chassis_indicator_cmd_)) _chassis_indicator_cmd_)
  (:chassis_brake_cmd_
   (&optional __chassis_brake_cmd_)
   (if __chassis_brake_cmd_ (setq _chassis_brake_cmd_ __chassis_brake_cmd_)) _chassis_brake_cmd_)
  (:serialization-length
   ()
   (+
    ;; int16 _chassis_vel_cmd_
    2
    ;; int16 _chassis_angle_cmd_
    2
    ;; int16 _chassis_indicator_cmd_
    2
    ;; int16 _chassis_brake_cmd_
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _chassis_vel_cmd_
       (write-word _chassis_vel_cmd_ s)
     ;; int16 _chassis_angle_cmd_
       (write-word _chassis_angle_cmd_ s)
     ;; int16 _chassis_indicator_cmd_
       (write-word _chassis_indicator_cmd_ s)
     ;; int16 _chassis_brake_cmd_
       (write-word _chassis_brake_cmd_ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _chassis_vel_cmd_
     (setq _chassis_vel_cmd_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_angle_cmd_
     (setq _chassis_angle_cmd_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_indicator_cmd_
     (setq _chassis_indicator_cmd_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_brake_cmd_
     (setq _chassis_brake_cmd_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get chassis_drive::chassis_cmd :md5sum-) "8c01b3f6487ca68f987b16e7e5933bc6")
(setf (get chassis_drive::chassis_cmd :datatype-) "chassis_drive/chassis_cmd")
(setf (get chassis_drive::chassis_cmd :definition-)
      "int16 chassis_vel_cmd_  
int16 chassis_angle_cmd_
int16 chassis_indicator_cmd_
int16 chassis_brake_cmd_

")



(provide :chassis_drive/chassis_cmd "8c01b3f6487ca68f987b16e7e5933bc6")


