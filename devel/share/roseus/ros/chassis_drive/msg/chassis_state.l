;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::chassis_state)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'chassis_state (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::CHASSIS_STATE")
  (make-package "CHASSIS_DRIVE::CHASSIS_STATE"))

(in-package "ROS")
;;//! \htmlinclude chassis_state.msg.html


(defclass chassis_drive::chassis_state
  :super ros::object
  :slots (_chassis_mileage_variable_ _chassis_mileage_record_ _chassis_drive_speed_ _chassis_drive_angle_ _chassis_drivemotor_error_code_ _chassis_whirlmotor_erro_code_ _chassis_liftmotor_erro_code_ _chassis_task_id_ _chassis_task_state_ _chassis_selfcheck_error_code_ ))

(defmethod chassis_drive::chassis_state
  (:init
   (&key
    ((:chassis_mileage_variable_ __chassis_mileage_variable_) 0)
    ((:chassis_mileage_record_ __chassis_mileage_record_) 0)
    ((:chassis_drive_speed_ __chassis_drive_speed_) 0)
    ((:chassis_drive_angle_ __chassis_drive_angle_) 0)
    ((:chassis_drivemotor_error_code_ __chassis_drivemotor_error_code_) 0)
    ((:chassis_whirlmotor_erro_code_ __chassis_whirlmotor_erro_code_) 0)
    ((:chassis_liftmotor_erro_code_ __chassis_liftmotor_erro_code_) 0)
    ((:chassis_task_id_ __chassis_task_id_) 0)
    ((:chassis_task_state_ __chassis_task_state_) 0)
    ((:chassis_selfcheck_error_code_ __chassis_selfcheck_error_code_) 0)
    )
   (send-super :init)
   (setq _chassis_mileage_variable_ (round __chassis_mileage_variable_))
   (setq _chassis_mileage_record_ (round __chassis_mileage_record_))
   (setq _chassis_drive_speed_ (round __chassis_drive_speed_))
   (setq _chassis_drive_angle_ (round __chassis_drive_angle_))
   (setq _chassis_drivemotor_error_code_ (round __chassis_drivemotor_error_code_))
   (setq _chassis_whirlmotor_erro_code_ (round __chassis_whirlmotor_erro_code_))
   (setq _chassis_liftmotor_erro_code_ (round __chassis_liftmotor_erro_code_))
   (setq _chassis_task_id_ (round __chassis_task_id_))
   (setq _chassis_task_state_ (round __chassis_task_state_))
   (setq _chassis_selfcheck_error_code_ (round __chassis_selfcheck_error_code_))
   self)
  (:chassis_mileage_variable_
   (&optional __chassis_mileage_variable_)
   (if __chassis_mileage_variable_ (setq _chassis_mileage_variable_ __chassis_mileage_variable_)) _chassis_mileage_variable_)
  (:chassis_mileage_record_
   (&optional __chassis_mileage_record_)
   (if __chassis_mileage_record_ (setq _chassis_mileage_record_ __chassis_mileage_record_)) _chassis_mileage_record_)
  (:chassis_drive_speed_
   (&optional __chassis_drive_speed_)
   (if __chassis_drive_speed_ (setq _chassis_drive_speed_ __chassis_drive_speed_)) _chassis_drive_speed_)
  (:chassis_drive_angle_
   (&optional __chassis_drive_angle_)
   (if __chassis_drive_angle_ (setq _chassis_drive_angle_ __chassis_drive_angle_)) _chassis_drive_angle_)
  (:chassis_drivemotor_error_code_
   (&optional __chassis_drivemotor_error_code_)
   (if __chassis_drivemotor_error_code_ (setq _chassis_drivemotor_error_code_ __chassis_drivemotor_error_code_)) _chassis_drivemotor_error_code_)
  (:chassis_whirlmotor_erro_code_
   (&optional __chassis_whirlmotor_erro_code_)
   (if __chassis_whirlmotor_erro_code_ (setq _chassis_whirlmotor_erro_code_ __chassis_whirlmotor_erro_code_)) _chassis_whirlmotor_erro_code_)
  (:chassis_liftmotor_erro_code_
   (&optional __chassis_liftmotor_erro_code_)
   (if __chassis_liftmotor_erro_code_ (setq _chassis_liftmotor_erro_code_ __chassis_liftmotor_erro_code_)) _chassis_liftmotor_erro_code_)
  (:chassis_task_id_
   (&optional __chassis_task_id_)
   (if __chassis_task_id_ (setq _chassis_task_id_ __chassis_task_id_)) _chassis_task_id_)
  (:chassis_task_state_
   (&optional __chassis_task_state_)
   (if __chassis_task_state_ (setq _chassis_task_state_ __chassis_task_state_)) _chassis_task_state_)
  (:chassis_selfcheck_error_code_
   (&optional __chassis_selfcheck_error_code_)
   (if __chassis_selfcheck_error_code_ (setq _chassis_selfcheck_error_code_ __chassis_selfcheck_error_code_)) _chassis_selfcheck_error_code_)
  (:serialization-length
   ()
   (+
    ;; int32 _chassis_mileage_variable_
    4
    ;; int32 _chassis_mileage_record_
    4
    ;; int16 _chassis_drive_speed_
    2
    ;; int16 _chassis_drive_angle_
    2
    ;; int16 _chassis_drivemotor_error_code_
    2
    ;; int16 _chassis_whirlmotor_erro_code_
    2
    ;; int16 _chassis_liftmotor_erro_code_
    2
    ;; int16 _chassis_task_id_
    2
    ;; int16 _chassis_task_state_
    2
    ;; int16 _chassis_selfcheck_error_code_
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _chassis_mileage_variable_
       (write-long _chassis_mileage_variable_ s)
     ;; int32 _chassis_mileage_record_
       (write-long _chassis_mileage_record_ s)
     ;; int16 _chassis_drive_speed_
       (write-word _chassis_drive_speed_ s)
     ;; int16 _chassis_drive_angle_
       (write-word _chassis_drive_angle_ s)
     ;; int16 _chassis_drivemotor_error_code_
       (write-word _chassis_drivemotor_error_code_ s)
     ;; int16 _chassis_whirlmotor_erro_code_
       (write-word _chassis_whirlmotor_erro_code_ s)
     ;; int16 _chassis_liftmotor_erro_code_
       (write-word _chassis_liftmotor_erro_code_ s)
     ;; int16 _chassis_task_id_
       (write-word _chassis_task_id_ s)
     ;; int16 _chassis_task_state_
       (write-word _chassis_task_state_ s)
     ;; int16 _chassis_selfcheck_error_code_
       (write-word _chassis_selfcheck_error_code_ s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _chassis_mileage_variable_
     (setq _chassis_mileage_variable_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _chassis_mileage_record_
     (setq _chassis_mileage_record_ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int16 _chassis_drive_speed_
     (setq _chassis_drive_speed_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_drive_angle_
     (setq _chassis_drive_angle_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_drivemotor_error_code_
     (setq _chassis_drivemotor_error_code_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_whirlmotor_erro_code_
     (setq _chassis_whirlmotor_erro_code_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_liftmotor_erro_code_
     (setq _chassis_liftmotor_erro_code_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_task_id_
     (setq _chassis_task_id_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_task_state_
     (setq _chassis_task_state_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _chassis_selfcheck_error_code_
     (setq _chassis_selfcheck_error_code_ (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get chassis_drive::chassis_state :md5sum-) "d74b78807fa4e4349088bf6fbedc2966")
(setf (get chassis_drive::chassis_state :datatype-) "chassis_drive/chassis_state")
(setf (get chassis_drive::chassis_state :definition-)
      "int32 chassis_mileage_variable_
int32 chassis_mileage_record_
int16 chassis_drive_speed_
int16 chassis_drive_angle_
int16 chassis_drivemotor_error_code_
int16 chassis_whirlmotor_erro_code_
int16 chassis_liftmotor_erro_code_

int16 chassis_task_id_
int16 chassis_task_state_
int16 chassis_selfcheck_error_code_
")



(provide :chassis_drive/chassis_state "d74b78807fa4e4349088bf6fbedc2966")


