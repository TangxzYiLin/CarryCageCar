;; Auto-generated. Do not edit!


(when (boundp 'agvs_control::date_realtime_feedback)
  (if (not (find-package "AGVS_CONTROL"))
    (make-package "AGVS_CONTROL"))
  (shadow 'date_realtime_feedback (find-package "AGVS_CONTROL")))
(unless (find-package "AGVS_CONTROL::DATE_REALTIME_FEEDBACK")
  (make-package "AGVS_CONTROL::DATE_REALTIME_FEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude date_realtime_feedback.msg.html


(defclass agvs_control::date_realtime_feedback
  :super ros::object
  :slots (_theta_angle _theta_x _theta_y _speed_y ))

(defmethod agvs_control::date_realtime_feedback
  (:init
   (&key
    ((:theta_angle __theta_angle) 0.0)
    ((:theta_x __theta_x) 0.0)
    ((:theta_y __theta_y) 0.0)
    ((:speed_y __speed_y) 0.0)
    )
   (send-super :init)
   (setq _theta_angle (float __theta_angle))
   (setq _theta_x (float __theta_x))
   (setq _theta_y (float __theta_y))
   (setq _speed_y (float __speed_y))
   self)
  (:theta_angle
   (&optional __theta_angle)
   (if __theta_angle (setq _theta_angle __theta_angle)) _theta_angle)
  (:theta_x
   (&optional __theta_x)
   (if __theta_x (setq _theta_x __theta_x)) _theta_x)
  (:theta_y
   (&optional __theta_y)
   (if __theta_y (setq _theta_y __theta_y)) _theta_y)
  (:speed_y
   (&optional __speed_y)
   (if __speed_y (setq _speed_y __speed_y)) _speed_y)
  (:serialization-length
   ()
   (+
    ;; float32 _theta_angle
    4
    ;; float32 _theta_x
    4
    ;; float32 _theta_y
    4
    ;; float32 _speed_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _theta_angle
       (sys::poke _theta_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_x
       (sys::poke _theta_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta_y
       (sys::poke _theta_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_y
       (sys::poke _speed_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _theta_angle
     (setq _theta_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_x
     (setq _theta_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta_y
     (setq _theta_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_y
     (setq _speed_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get agvs_control::date_realtime_feedback :md5sum-) "9373b18cc1268d229a49f9c7a9f9ccc5")
(setf (get agvs_control::date_realtime_feedback :datatype-) "agvs_control/date_realtime_feedback")
(setf (get agvs_control::date_realtime_feedback :definition-)
      "float32 theta_angle
float32 theta_x
float32 theta_y

float32 speed_y

")



(provide :agvs_control/date_realtime_feedback "9373b18cc1268d229a49f9c7a9f9ccc5")


