;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::chassis_bat)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'chassis_bat (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::CHASSIS_BAT")
  (make-package "CHASSIS_DRIVE::CHASSIS_BAT"))

(in-package "ROS")
;;//! \htmlinclude chassis_bat.msg.html


(defclass chassis_drive::chassis_bat
  :super ros::object
  :slots (_bat_cap_ _bat_vol_ _bat_cur_ _bat_error_code_ ))

(defmethod chassis_drive::chassis_bat
  (:init
   (&key
    ((:bat_cap_ __bat_cap_) 0)
    ((:bat_vol_ __bat_vol_) 0)
    ((:bat_cur_ __bat_cur_) 0)
    ((:bat_error_code_ __bat_error_code_) 0)
    )
   (send-super :init)
   (setq _bat_cap_ (round __bat_cap_))
   (setq _bat_vol_ (round __bat_vol_))
   (setq _bat_cur_ (round __bat_cur_))
   (setq _bat_error_code_ (round __bat_error_code_))
   self)
  (:bat_cap_
   (&optional __bat_cap_)
   (if __bat_cap_ (setq _bat_cap_ __bat_cap_)) _bat_cap_)
  (:bat_vol_
   (&optional __bat_vol_)
   (if __bat_vol_ (setq _bat_vol_ __bat_vol_)) _bat_vol_)
  (:bat_cur_
   (&optional __bat_cur_)
   (if __bat_cur_ (setq _bat_cur_ __bat_cur_)) _bat_cur_)
  (:bat_error_code_
   (&optional __bat_error_code_)
   (if __bat_error_code_ (setq _bat_error_code_ __bat_error_code_)) _bat_error_code_)
  (:serialization-length
   ()
   (+
    ;; int16 _bat_cap_
    2
    ;; int16 _bat_vol_
    2
    ;; int16 _bat_cur_
    2
    ;; int16 _bat_error_code_
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _bat_cap_
       (write-word _bat_cap_ s)
     ;; int16 _bat_vol_
       (write-word _bat_vol_ s)
     ;; int16 _bat_cur_
       (write-word _bat_cur_ s)
     ;; int16 _bat_error_code_
       (write-word _bat_error_code_ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _bat_cap_
     (setq _bat_cap_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _bat_vol_
     (setq _bat_vol_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _bat_cur_
     (setq _bat_cur_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _bat_error_code_
     (setq _bat_error_code_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get chassis_drive::chassis_bat :md5sum-) "1d7bde1a0bdfdf9227bac0927d779d8f")
(setf (get chassis_drive::chassis_bat :datatype-) "chassis_drive/chassis_bat")
(setf (get chassis_drive::chassis_bat :definition-)
      "int16 bat_cap_
int16 bat_vol_
int16 bat_cur_
int16 bat_error_code_
")



(provide :chassis_drive/chassis_bat "1d7bde1a0bdfdf9227bac0927d779d8f")


