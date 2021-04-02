;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::chassis_alarm)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'chassis_alarm (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::CHASSIS_ALARM")
  (make-package "CHASSIS_DRIVE::CHASSIS_ALARM"))

(in-package "ROS")
;;//! \htmlinclude chassis_alarm.msg.html


(defclass chassis_drive::chassis_alarm
  :super ros::object
  :slots (_alarm_forkphotoe_left_ _alarm_forkphotoe_right_ _alarm_cargophotos_left_ _alarm_cargophotos_right_ _alarm_micro_swtich_ _alarm_collision_avoidance_ _alarm_emergency_stop_swtich_ _alarm_auto_man_swtich_ _alarm_up_limit_ _alarm_down_limit_ ))

(defmethod chassis_drive::chassis_alarm
  (:init
   (&key
    ((:alarm_forkphotoe_left_ __alarm_forkphotoe_left_) nil)
    ((:alarm_forkphotoe_right_ __alarm_forkphotoe_right_) nil)
    ((:alarm_cargophotos_left_ __alarm_cargophotos_left_) nil)
    ((:alarm_cargophotos_right_ __alarm_cargophotos_right_) nil)
    ((:alarm_micro_swtich_ __alarm_micro_swtich_) nil)
    ((:alarm_collision_avoidance_ __alarm_collision_avoidance_) nil)
    ((:alarm_emergency_stop_swtich_ __alarm_emergency_stop_swtich_) nil)
    ((:alarm_auto_man_swtich_ __alarm_auto_man_swtich_) nil)
    ((:alarm_up_limit_ __alarm_up_limit_) nil)
    ((:alarm_down_limit_ __alarm_down_limit_) nil)
    )
   (send-super :init)
   (setq _alarm_forkphotoe_left_ __alarm_forkphotoe_left_)
   (setq _alarm_forkphotoe_right_ __alarm_forkphotoe_right_)
   (setq _alarm_cargophotos_left_ __alarm_cargophotos_left_)
   (setq _alarm_cargophotos_right_ __alarm_cargophotos_right_)
   (setq _alarm_micro_swtich_ __alarm_micro_swtich_)
   (setq _alarm_collision_avoidance_ __alarm_collision_avoidance_)
   (setq _alarm_emergency_stop_swtich_ __alarm_emergency_stop_swtich_)
   (setq _alarm_auto_man_swtich_ __alarm_auto_man_swtich_)
   (setq _alarm_up_limit_ __alarm_up_limit_)
   (setq _alarm_down_limit_ __alarm_down_limit_)
   self)
  (:alarm_forkphotoe_left_
   (&optional __alarm_forkphotoe_left_)
   (if __alarm_forkphotoe_left_ (setq _alarm_forkphotoe_left_ __alarm_forkphotoe_left_)) _alarm_forkphotoe_left_)
  (:alarm_forkphotoe_right_
   (&optional __alarm_forkphotoe_right_)
   (if __alarm_forkphotoe_right_ (setq _alarm_forkphotoe_right_ __alarm_forkphotoe_right_)) _alarm_forkphotoe_right_)
  (:alarm_cargophotos_left_
   (&optional __alarm_cargophotos_left_)
   (if __alarm_cargophotos_left_ (setq _alarm_cargophotos_left_ __alarm_cargophotos_left_)) _alarm_cargophotos_left_)
  (:alarm_cargophotos_right_
   (&optional __alarm_cargophotos_right_)
   (if __alarm_cargophotos_right_ (setq _alarm_cargophotos_right_ __alarm_cargophotos_right_)) _alarm_cargophotos_right_)
  (:alarm_micro_swtich_
   (&optional __alarm_micro_swtich_)
   (if __alarm_micro_swtich_ (setq _alarm_micro_swtich_ __alarm_micro_swtich_)) _alarm_micro_swtich_)
  (:alarm_collision_avoidance_
   (&optional __alarm_collision_avoidance_)
   (if __alarm_collision_avoidance_ (setq _alarm_collision_avoidance_ __alarm_collision_avoidance_)) _alarm_collision_avoidance_)
  (:alarm_emergency_stop_swtich_
   (&optional __alarm_emergency_stop_swtich_)
   (if __alarm_emergency_stop_swtich_ (setq _alarm_emergency_stop_swtich_ __alarm_emergency_stop_swtich_)) _alarm_emergency_stop_swtich_)
  (:alarm_auto_man_swtich_
   (&optional __alarm_auto_man_swtich_)
   (if __alarm_auto_man_swtich_ (setq _alarm_auto_man_swtich_ __alarm_auto_man_swtich_)) _alarm_auto_man_swtich_)
  (:alarm_up_limit_
   (&optional __alarm_up_limit_)
   (if __alarm_up_limit_ (setq _alarm_up_limit_ __alarm_up_limit_)) _alarm_up_limit_)
  (:alarm_down_limit_
   (&optional __alarm_down_limit_)
   (if __alarm_down_limit_ (setq _alarm_down_limit_ __alarm_down_limit_)) _alarm_down_limit_)
  (:serialization-length
   ()
   (+
    ;; bool _alarm_forkphotoe_left_
    1
    ;; bool _alarm_forkphotoe_right_
    1
    ;; bool _alarm_cargophotos_left_
    1
    ;; bool _alarm_cargophotos_right_
    1
    ;; bool _alarm_micro_swtich_
    1
    ;; bool _alarm_collision_avoidance_
    1
    ;; bool _alarm_emergency_stop_swtich_
    1
    ;; bool _alarm_auto_man_swtich_
    1
    ;; bool _alarm_up_limit_
    1
    ;; bool _alarm_down_limit_
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _alarm_forkphotoe_left_
       (if _alarm_forkphotoe_left_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_forkphotoe_right_
       (if _alarm_forkphotoe_right_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_cargophotos_left_
       (if _alarm_cargophotos_left_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_cargophotos_right_
       (if _alarm_cargophotos_right_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_micro_swtich_
       (if _alarm_micro_swtich_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_collision_avoidance_
       (if _alarm_collision_avoidance_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_emergency_stop_swtich_
       (if _alarm_emergency_stop_swtich_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_auto_man_swtich_
       (if _alarm_auto_man_swtich_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_up_limit_
       (if _alarm_up_limit_ (write-byte -1 s) (write-byte 0 s))
     ;; bool _alarm_down_limit_
       (if _alarm_down_limit_ (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _alarm_forkphotoe_left_
     (setq _alarm_forkphotoe_left_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_forkphotoe_right_
     (setq _alarm_forkphotoe_right_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_cargophotos_left_
     (setq _alarm_cargophotos_left_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_cargophotos_right_
     (setq _alarm_cargophotos_right_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_micro_swtich_
     (setq _alarm_micro_swtich_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_collision_avoidance_
     (setq _alarm_collision_avoidance_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_emergency_stop_swtich_
     (setq _alarm_emergency_stop_swtich_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_auto_man_swtich_
     (setq _alarm_auto_man_swtich_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_up_limit_
     (setq _alarm_up_limit_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _alarm_down_limit_
     (setq _alarm_down_limit_ (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get chassis_drive::chassis_alarm :md5sum-) "ae8c21bba8b1459751e77f410496d9c0")
(setf (get chassis_drive::chassis_alarm :datatype-) "chassis_drive/chassis_alarm")
(setf (get chassis_drive::chassis_alarm :definition-)
      "bool alarm_forkphotoe_left_
bool alarm_forkphotoe_right_
bool alarm_cargophotos_left_
bool alarm_cargophotos_right_
bool alarm_micro_swtich_
bool alarm_collision_avoidance_
bool alarm_emergency_stop_swtich_
bool alarm_auto_man_swtich_
bool alarm_up_limit_
bool alarm_down_limit_

")



(provide :chassis_drive/chassis_alarm "ae8c21bba8b1459751e77f410496d9c0")


