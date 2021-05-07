; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude agvs_test.msg.html

(cl:defclass <agvs_test> (roslisp-msg-protocol:ros-message)
  ((agvs_mode_1
    :reader agvs_mode_1
    :initarg :agvs_mode_1
    :type chassis_drive-msg:agvs_mode
    :initform (cl:make-instance 'chassis_drive-msg:agvs_mode))
   (agvs_mode_2
    :reader agvs_mode_2
    :initarg :agvs_mode_2
    :type chassis_drive-msg:agvs_mode
    :initform (cl:make-instance 'chassis_drive-msg:agvs_mode))
   (run_target_task
    :reader run_target_task
    :initarg :run_target_task
    :type agvs_task-msg:route_target
    :initform (cl:make-instance 'agvs_task-msg:route_target)))
)

(cl:defclass agvs_test (<agvs_test>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agvs_test>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agvs_test)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<agvs_test> is deprecated: use chassis_drive-msg:agvs_test instead.")))

(cl:ensure-generic-function 'agvs_mode_1-val :lambda-list '(m))
(cl:defmethod agvs_mode_1-val ((m <agvs_test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:agvs_mode_1-val is deprecated.  Use chassis_drive-msg:agvs_mode_1 instead.")
  (agvs_mode_1 m))

(cl:ensure-generic-function 'agvs_mode_2-val :lambda-list '(m))
(cl:defmethod agvs_mode_2-val ((m <agvs_test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:agvs_mode_2-val is deprecated.  Use chassis_drive-msg:agvs_mode_2 instead.")
  (agvs_mode_2 m))

(cl:ensure-generic-function 'run_target_task-val :lambda-list '(m))
(cl:defmethod run_target_task-val ((m <agvs_test>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:run_target_task-val is deprecated.  Use chassis_drive-msg:run_target_task instead.")
  (run_target_task m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agvs_test>) ostream)
  "Serializes a message object of type '<agvs_test>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'agvs_mode_1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'agvs_mode_2) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'run_target_task) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agvs_test>) istream)
  "Deserializes a message object of type '<agvs_test>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'agvs_mode_1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'agvs_mode_2) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'run_target_task) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agvs_test>)))
  "Returns string type for a message object of type '<agvs_test>"
  "chassis_drive/agvs_test")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agvs_test)))
  "Returns string type for a message object of type 'agvs_test"
  "chassis_drive/agvs_test")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agvs_test>)))
  "Returns md5sum for a message object of type '<agvs_test>"
  "944f63dcbde35e41019eefa46ce8a035")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agvs_test)))
  "Returns md5sum for a message object of type 'agvs_test"
  "944f63dcbde35e41019eefa46ce8a035")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agvs_test>)))
  "Returns full string definition for message of type '<agvs_test>"
  (cl:format cl:nil "agvs_mode agvs_mode_1~%agvs_mode agvs_mode_2~%~%agvs_task/route_target run_target_task~%~%================================================================================~%MSG: chassis_drive/agvs_mode~%uint8 current_mode~%~%uint8 agvs_mode_manual =0~%uint8 agvs_mode_auto_floor_1 = 1~%uint8 agvs_mode_auto_floor_2 = 2~%================================================================================~%MSG: agvs_task/route_target~%float32 target_location_x~%float32 target_location_y~%float32 target_speed~%uint8 task_direction~%uint8 task_route_id~%~%uint8 default_idle = 0~%uint8 positive_direction = 1~%uint8 opposite_direction = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agvs_test)))
  "Returns full string definition for message of type 'agvs_test"
  (cl:format cl:nil "agvs_mode agvs_mode_1~%agvs_mode agvs_mode_2~%~%agvs_task/route_target run_target_task~%~%================================================================================~%MSG: chassis_drive/agvs_mode~%uint8 current_mode~%~%uint8 agvs_mode_manual =0~%uint8 agvs_mode_auto_floor_1 = 1~%uint8 agvs_mode_auto_floor_2 = 2~%================================================================================~%MSG: agvs_task/route_target~%float32 target_location_x~%float32 target_location_y~%float32 target_speed~%uint8 task_direction~%uint8 task_route_id~%~%uint8 default_idle = 0~%uint8 positive_direction = 1~%uint8 opposite_direction = 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agvs_test>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'agvs_mode_1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'agvs_mode_2))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'run_target_task))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agvs_test>))
  "Converts a ROS message object to a list"
  (cl:list 'agvs_test
    (cl:cons ':agvs_mode_1 (agvs_mode_1 msg))
    (cl:cons ':agvs_mode_2 (agvs_mode_2 msg))
    (cl:cons ':run_target_task (run_target_task msg))
))
