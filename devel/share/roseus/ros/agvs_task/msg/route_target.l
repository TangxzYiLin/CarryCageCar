;; Auto-generated. Do not edit!


(when (boundp 'agvs_task::route_target)
  (if (not (find-package "AGVS_TASK"))
    (make-package "AGVS_TASK"))
  (shadow 'route_target (find-package "AGVS_TASK")))
(unless (find-package "AGVS_TASK::ROUTE_TARGET")
  (make-package "AGVS_TASK::ROUTE_TARGET"))

(in-package "ROS")
;;//! \htmlinclude route_target.msg.html


(defclass agvs_task::route_target
  :super ros::object
  :slots (_target_location_x _target_location_y _target_speed _task_direction _task_route_id ))

(defmethod agvs_task::route_target
  (:init
   (&key
    ((:target_location_x __target_location_x) 0.0)
    ((:target_location_y __target_location_y) 0.0)
    ((:target_speed __target_speed) 0.0)
    ((:task_direction __task_direction) 0)
    ((:task_route_id __task_route_id) 0)
    )
   (send-super :init)
   (setq _target_location_x (float __target_location_x))
   (setq _target_location_y (float __target_location_y))
   (setq _target_speed (float __target_speed))
   (setq _task_direction (round __task_direction))
   (setq _task_route_id (round __task_route_id))
   self)
  (:target_location_x
   (&optional __target_location_x)
   (if __target_location_x (setq _target_location_x __target_location_x)) _target_location_x)
  (:target_location_y
   (&optional __target_location_y)
   (if __target_location_y (setq _target_location_y __target_location_y)) _target_location_y)
  (:target_speed
   (&optional __target_speed)
   (if __target_speed (setq _target_speed __target_speed)) _target_speed)
  (:task_direction
   (&optional __task_direction)
   (if __task_direction (setq _task_direction __task_direction)) _task_direction)
  (:task_route_id
   (&optional __task_route_id)
   (if __task_route_id (setq _task_route_id __task_route_id)) _task_route_id)
  (:serialization-length
   ()
   (+
    ;; float32 _target_location_x
    4
    ;; float32 _target_location_y
    4
    ;; float32 _target_speed
    4
    ;; int8 _task_direction
    1
    ;; int8 _task_route_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _target_location_x
       (sys::poke _target_location_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_location_y
       (sys::poke _target_location_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_speed
       (sys::poke _target_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _task_direction
       (write-byte _task_direction s)
     ;; int8 _task_route_id
       (write-byte _task_route_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _target_location_x
     (setq _target_location_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_location_y
     (setq _target_location_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_speed
     (setq _target_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _task_direction
     (setq _task_direction (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _task_direction 127) (setq _task_direction (- _task_direction 256)))
   ;; int8 _task_route_id
     (setq _task_route_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _task_route_id 127) (setq _task_route_id (- _task_route_id 256)))
   ;;
   self)
  )

(setf (get agvs_task::route_target :md5sum-) "6a82df07cc526c047af2d9e2d7bfca86")
(setf (get agvs_task::route_target :datatype-) "agvs_task/route_target")
(setf (get agvs_task::route_target :definition-)
      "float32 target_location_x
float32 target_location_y
float32 target_speed
int8 task_direction
int8 task_route_id

")



(provide :agvs_task/route_target "6a82df07cc526c047af2d9e2d7bfca86")


