;; Auto-generated. Do not edit!


(when (boundp 'agvs_control::date_pads_cmd)
  (if (not (find-package "AGVS_CONTROL"))
    (make-package "AGVS_CONTROL"))
  (shadow 'date_pads_cmd (find-package "AGVS_CONTROL")))
(unless (find-package "AGVS_CONTROL::DATE_PADS_CMD")
  (make-package "AGVS_CONTROL::DATE_PADS_CMD"))

(in-package "ROS")
;;//! \htmlinclude date_pads_cmd.msg.html


(defclass agvs_control::date_pads_cmd
  :super ros::object
  :slots (_angle_date _speed_date ))

(defmethod agvs_control::date_pads_cmd
  (:init
   (&key
    ((:angle_date __angle_date) 0.0)
    ((:speed_date __speed_date) 0.0)
    )
   (send-super :init)
   (setq _angle_date (float __angle_date))
   (setq _speed_date (float __speed_date))
   self)
  (:angle_date
   (&optional __angle_date)
   (if __angle_date (setq _angle_date __angle_date)) _angle_date)
  (:speed_date
   (&optional __speed_date)
   (if __speed_date (setq _speed_date __speed_date)) _speed_date)
  (:serialization-length
   ()
   (+
    ;; float32 _angle_date
    4
    ;; float32 _speed_date
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle_date
       (sys::poke _angle_date (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed_date
       (sys::poke _speed_date (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle_date
     (setq _angle_date (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed_date
     (setq _speed_date (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get agvs_control::date_pads_cmd :md5sum-) "400b13fd7cc1c6f886c0fcc01fcab9aa")
(setf (get agvs_control::date_pads_cmd :datatype-) "agvs_control/date_pads_cmd")
(setf (get agvs_control::date_pads_cmd :definition-)
      "float32 angle_date
float32 speed_date

")



(provide :agvs_control/date_pads_cmd "400b13fd7cc1c6f886c0fcc01fcab9aa")


