;; Auto-generated. Do not edit!


(when (boundp 'agvs_control::cmd_control_mode)
  (if (not (find-package "AGVS_CONTROL"))
    (make-package "AGVS_CONTROL"))
  (shadow 'cmd_control_mode (find-package "AGVS_CONTROL")))
(unless (find-package "AGVS_CONTROL::CMD_CONTROL_MODE")
  (make-package "AGVS_CONTROL::CMD_CONTROL_MODE"))
(unless (find-package "AGVS_CONTROL::CMD_CONTROL_MODEREQUEST")
  (make-package "AGVS_CONTROL::CMD_CONTROL_MODEREQUEST"))
(unless (find-package "AGVS_CONTROL::CMD_CONTROL_MODERESPONSE")
  (make-package "AGVS_CONTROL::CMD_CONTROL_MODERESPONSE"))

(in-package "ROS")





(intern "*MANUAL_MODE*" (find-package "AGVS_CONTROL::CMD_CONTROL_MODEREQUEST"))
(shadow '*MANUAL_MODE* (find-package "AGVS_CONTROL::CMD_CONTROL_MODEREQUEST"))
(defconstant agvs_control::cmd_control_modeRequest::*MANUAL_MODE* "\"manual_mode\"")
(intern "*AUTO_MODE*" (find-package "AGVS_CONTROL::CMD_CONTROL_MODEREQUEST"))
(shadow '*AUTO_MODE* (find-package "AGVS_CONTROL::CMD_CONTROL_MODEREQUEST"))
(defconstant agvs_control::cmd_control_modeRequest::*AUTO_MODE* "\"auto_mode\"")
(defclass agvs_control::cmd_control_modeRequest
  :super ros::object
  :slots (_mode_run ))

(defmethod agvs_control::cmd_control_modeRequest
  (:init
   (&key
    ((:mode_run __mode_run) "")
    )
   (send-super :init)
   (setq _mode_run (string __mode_run))
   self)
  (:mode_run
   (&optional __mode_run)
   (if __mode_run (setq _mode_run __mode_run)) _mode_run)
  (:serialization-length
   ()
   (+
    ;; string _mode_run
    4 (length _mode_run)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode_run
       (write-long (length _mode_run) s) (princ _mode_run s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode_run
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode_run (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass agvs_control::cmd_control_modeResponse
  :super ros::object
  :slots ())

(defmethod agvs_control::cmd_control_modeResponse
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

(defclass agvs_control::cmd_control_mode
  :super ros::object
  :slots ())

(setf (get agvs_control::cmd_control_mode :md5sum-) "0545bf3db61acfe8473d040eb76fcb1c")
(setf (get agvs_control::cmd_control_mode :datatype-) "agvs_control/cmd_control_mode")
(setf (get agvs_control::cmd_control_mode :request) agvs_control::cmd_control_modeRequest)
(setf (get agvs_control::cmd_control_mode :response) agvs_control::cmd_control_modeResponse)

(defmethod agvs_control::cmd_control_modeRequest
  (:response () (instance agvs_control::cmd_control_modeResponse :init)))

(setf (get agvs_control::cmd_control_modeRequest :md5sum-) "0545bf3db61acfe8473d040eb76fcb1c")
(setf (get agvs_control::cmd_control_modeRequest :datatype-) "agvs_control/cmd_control_modeRequest")
(setf (get agvs_control::cmd_control_modeRequest :definition-)
      "string mode_run 
string manual_mode=\"manual_mode\"
string auto_mode=\"auto_mode\"

---
")

(setf (get agvs_control::cmd_control_modeResponse :md5sum-) "0545bf3db61acfe8473d040eb76fcb1c")
(setf (get agvs_control::cmd_control_modeResponse :datatype-) "agvs_control/cmd_control_modeResponse")
(setf (get agvs_control::cmd_control_modeResponse :definition-)
      "string mode_run 
string manual_mode=\"manual_mode\"
string auto_mode=\"auto_mode\"

---
")



(provide :agvs_control/cmd_control_mode "0545bf3db61acfe8473d040eb76fcb1c")


