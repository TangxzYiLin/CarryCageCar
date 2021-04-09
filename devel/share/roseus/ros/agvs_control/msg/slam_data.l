;; Auto-generated. Do not edit!


(when (boundp 'agvs_control::slam_data)
  (if (not (find-package "AGVS_CONTROL"))
    (make-package "AGVS_CONTROL"))
  (shadow 'slam_data (find-package "AGVS_CONTROL")))
(unless (find-package "AGVS_CONTROL::SLAM_DATA")
  (make-package "AGVS_CONTROL::SLAM_DATA"))

(in-package "ROS")
;;//! \htmlinclude slam_data.msg.html


(defclass agvs_control::slam_data
  :super ros::object
  :slots (_theta_x _theta_y _theta_angle _speed_x _speed_y _speed_z ))

(defmethod agvs_control::slam_data
  (:init
   (&key
    ((:theta_x __theta_x) 0.0)
    ((:theta_y __theta_y) 0.0)
    ((:theta_angle __theta_angle) 0.0)
    ((:speed_x __speed_x) 0.0)
    ((:speed_y __speed_y) 0.0)
    ((:speed_z __speed_z) 0.0)
    )
   (send-super :init)
   (setq _theta_x (float __theta_x))
   (setq _theta_y (float __theta_y))
   (setq _theta_angle (float __theta_angle))
   (setq _speed_x (float __speed_x))
   (setq _speed_y (float __speed_y))
   (setq _speed_z (float __speed_z))
   self)
  (:theta_x
   (&optional __theta_x)
   (if __theta_x (setq _theta_x __theta_x)) _theta_x)
  (:theta_y
   (&optional __theta_y)
   (if __theta_y (setq _theta_y __theta_y)) _theta_y)
  (:theta_angle
   (&optional __theta_angle)
   (if __theta_angle (setq _theta_angle __theta_angle)) _theta_angle)
  (:speed_x
   (&optional __speed_x)
   (if __speed_x (setq _speed_x __speed_x)) _speed_x)
  (:speed_y
   (&optional __speed_y)
   (if __speed_y (setq _speed_y __speed_y)) _speed_y)
  (:speed_z
   (&optional __speed_z)
   (if __speed_z (setq _speed_z __speed_z)) _speed_z)
  (:serialization-length
   ()
   (+
    ;; float32 _theta_x
    4
    ;; float32 _theta_y
    4
    ;; float32 _theta_angle
    4
    ;; float32 _speed_x
    4
    ;; float32 _speed_y
    4
    ;; float32 _speed_z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _theta_x
       (sys::poke _theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_y
       (sys::poke _theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_angle
       (sys::poke _theta_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_x
       (sys::poke _speed_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_y
       (sys::poke _speed_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_z
       (sys::poke _speed_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _theta_x
     (setq _theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_y
     (setq _theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_angle
     (setq _theta_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_x
     (setq _speed_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_y
     (setq _speed_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_z
     (setq _speed_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get agvs_control::slam_data :md5sum-) "91db6ffd3f1cb419025d2f1a637f02e6")
(setf (get agvs_control::slam_data :datatype-) "agvs_control/slam_data")
(setf (get agvs_control::slam_data :definition-)
      "float32 theta_x
float32 theta_y
float32 theta_angle
float32 speed_x
float32 speed_y
float32 speed_z

")



(provide :agvs_control/slam_data "91db6ffd3f1cb419025d2f1a637f02e6")


