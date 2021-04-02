;; Auto-generated. Do not edit!


(when (boundp 'agvs_control::cmd_control_task)
  (if (not (find-package "AGVS_CONTROL"))
    (make-package "AGVS_CONTROL"))
  (shadow 'cmd_control_task (find-package "AGVS_CONTROL")))
(unless (find-package "AGVS_CONTROL::CMD_CONTROL_TASK")
  (make-package "AGVS_CONTROL::CMD_CONTROL_TASK"))
(unless (find-package "AGVS_CONTROL::CMD_CONTROL_TASKREQUEST")
  (make-package "AGVS_CONTROL::CMD_CONTROL_TASKREQUEST"))
(unless (find-package "AGVS_CONTROL::CMD_CONTROL_TASKRESPONSE")
  (make-package "AGVS_CONTROL::CMD_CONTROL_TASKRESPONSE"))

(in-package "ROS")





(intern "*MANUAL_MODE*" (find-package "AGVS_CONTROL::CMD_CONTROL_TASKREQUEST"))
(shadow '*MANUAL_MODE* (find-package "AGVS_CONTROL::CMD_CONTROL_TASKREQUEST"))
(defconstant agvs_control::cmd_control_taskRequest::*MANUAL_MODE* 1)
(intern "*AUTO_MODE*" (find-package "AGVS_CONTROL::CMD_CONTROL_TASKREQUEST"))
(shadow '*AUTO_MODE* (find-package "AGVS_CONTROL::CMD_CONTROL_TASKREQUEST"))
(defconstant agvs_control::cmd_control_taskRequest::*AUTO_MODE* 2)
(defclass agvs_control::cmd_control_taskRequest
  :super ros::object
  :slots (_mode_run ))

(defmethod agvs_control::cmd_control_taskRequest
  (:init
   (&key
    ((:mode_run __mode_run) 0)
    )
   (send-super :init)
   (setq _mode_run (round __mode_run))
   self)
  (:mode_run
   (&optional __mode_run)
   (if __mode_run (setq _mode_run __mode_run)) _mode_run)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode_run
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode_run
       (write-byte _mode_run s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode_run
     (setq _mode_run (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass agvs_control::cmd_control_taskResponse
  :super ros::object
  :slots ())

(defmethod agvs_control::cmd_control_taskResponse
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

(defclass agvs_control::cmd_control_task
  :super ros::object
  :slots ())

(setf (get agvs_control::cmd_control_task :md5sum-) "f9b5bf540932291b1fe9cba3efc63097")
(setf (get agvs_control::cmd_control_task :datatype-) "agvs_control/cmd_control_task")
(setf (get agvs_control::cmd_control_task :request) agvs_control::cmd_control_taskRequest)
(setf (get agvs_control::cmd_control_task :response) agvs_control::cmd_control_taskResponse)

(defmethod agvs_control::cmd_control_taskRequest
  (:response () (instance agvs_control::cmd_control_taskResponse :init)))

(setf (get agvs_control::cmd_control_taskRequest :md5sum-) "f9b5bf540932291b1fe9cba3efc63097")
(setf (get agvs_control::cmd_control_taskRequest :datatype-) "agvs_control/cmd_control_taskRequest")
(setf (get agvs_control::cmd_control_taskRequest :definition-)
      "uint8 mode_run 
uint8 manual_mode=1
uint8 auto_mode=2

---
")

(setf (get agvs_control::cmd_control_taskResponse :md5sum-) "f9b5bf540932291b1fe9cba3efc63097")
(setf (get agvs_control::cmd_control_taskResponse :datatype-) "agvs_control/cmd_control_taskResponse")
(setf (get agvs_control::cmd_control_taskResponse :definition-)
      "uint8 mode_run 
uint8 manual_mode=1
uint8 auto_mode=2

---
")



(provide :agvs_control/cmd_control_task "f9b5bf540932291b1fe9cba3efc63097")


