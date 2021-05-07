;; Auto-generated. Do not edit!


(when (boundp 'chassis_drive::agvs_test)
  (if (not (find-package "CHASSIS_DRIVE"))
    (make-package "CHASSIS_DRIVE"))
  (shadow 'agvs_test (find-package "CHASSIS_DRIVE")))
(unless (find-package "CHASSIS_DRIVE::AGVS_TEST")
  (make-package "CHASSIS_DRIVE::AGVS_TEST"))

(in-package "ROS")
;;//! \htmlinclude agvs_test.msg.html
(if (not (find-package "AGVS_TASK"))
  (ros::roseus-add-msgs "agvs_task"))


(defclass chassis_drive::agvs_test
  :super ros::object
  :slots (_agvs_mode_1 _agvs_mode_2 _run_target_task ))

(defmethod chassis_drive::agvs_test
  (:init
   (&key
    ((:agvs_mode_1 __agvs_mode_1) (instance chassis_drive::agvs_mode :init))
    ((:agvs_mode_2 __agvs_mode_2) (instance chassis_drive::agvs_mode :init))
    ((:run_target_task __run_target_task) (instance agvs_task::route_target :init))
    )
   (send-super :init)
   (setq _agvs_mode_1 __agvs_mode_1)
   (setq _agvs_mode_2 __agvs_mode_2)
   (setq _run_target_task __run_target_task)
   self)
  (:agvs_mode_1
   (&rest __agvs_mode_1)
   (if (keywordp (car __agvs_mode_1))
       (send* _agvs_mode_1 __agvs_mode_1)
     (progn
       (if __agvs_mode_1 (setq _agvs_mode_1 (car __agvs_mode_1)))
       _agvs_mode_1)))
  (:agvs_mode_2
   (&rest __agvs_mode_2)
   (if (keywordp (car __agvs_mode_2))
       (send* _agvs_mode_2 __agvs_mode_2)
     (progn
       (if __agvs_mode_2 (setq _agvs_mode_2 (car __agvs_mode_2)))
       _agvs_mode_2)))
  (:run_target_task
   (&rest __run_target_task)
   (if (keywordp (car __run_target_task))
       (send* _run_target_task __run_target_task)
     (progn
       (if __run_target_task (setq _run_target_task (car __run_target_task)))
       _run_target_task)))
  (:serialization-length
   ()
   (+
    ;; chassis_drive/agvs_mode _agvs_mode_1
    (send _agvs_mode_1 :serialization-length)
    ;; chassis_drive/agvs_mode _agvs_mode_2
    (send _agvs_mode_2 :serialization-length)
    ;; agvs_task/route_target _run_target_task
    (send _run_target_task :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; chassis_drive/agvs_mode _agvs_mode_1
       (send _agvs_mode_1 :serialize s)
     ;; chassis_drive/agvs_mode _agvs_mode_2
       (send _agvs_mode_2 :serialize s)
     ;; agvs_task/route_target _run_target_task
       (send _run_target_task :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; chassis_drive/agvs_mode _agvs_mode_1
     (send _agvs_mode_1 :deserialize buf ptr-) (incf ptr- (send _agvs_mode_1 :serialization-length))
   ;; chassis_drive/agvs_mode _agvs_mode_2
     (send _agvs_mode_2 :deserialize buf ptr-) (incf ptr- (send _agvs_mode_2 :serialization-length))
   ;; agvs_task/route_target _run_target_task
     (send _run_target_task :deserialize buf ptr-) (incf ptr- (send _run_target_task :serialization-length))
   ;;
   self)
  )

(setf (get chassis_drive::agvs_test :md5sum-) "944f63dcbde35e41019eefa46ce8a035")
(setf (get chassis_drive::agvs_test :datatype-) "chassis_drive/agvs_test")
(setf (get chassis_drive::agvs_test :definition-)
      "agvs_mode agvs_mode_1
agvs_mode agvs_mode_2

agvs_task/route_target run_target_task

================================================================================
MSG: chassis_drive/agvs_mode
uint8 current_mode

uint8 agvs_mode_manual =0
uint8 agvs_mode_auto_floor_1 = 1
uint8 agvs_mode_auto_floor_2 = 2
================================================================================
MSG: agvs_task/route_target
float32 target_location_x
float32 target_location_y
float32 target_speed
uint8 task_direction
uint8 task_route_id

uint8 default_idle = 0
uint8 positive_direction = 1
uint8 opposite_direction = 2

")



(provide :chassis_drive/agvs_test "944f63dcbde35e41019eefa46ce8a035")


