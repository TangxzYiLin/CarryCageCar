; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude chassis_alarm.msg.html

(cl:defclass <chassis_alarm> (roslisp-msg-protocol:ros-message)
  ((alarm_forkphotoe_left_
    :reader alarm_forkphotoe_left_
    :initarg :alarm_forkphotoe_left_
    :type cl:boolean
    :initform cl:nil)
   (alarm_forkphotoe_right_
    :reader alarm_forkphotoe_right_
    :initarg :alarm_forkphotoe_right_
    :type cl:boolean
    :initform cl:nil)
   (alarm_cargophotos_left_
    :reader alarm_cargophotos_left_
    :initarg :alarm_cargophotos_left_
    :type cl:boolean
    :initform cl:nil)
   (alarm_cargophotos_right_
    :reader alarm_cargophotos_right_
    :initarg :alarm_cargophotos_right_
    :type cl:boolean
    :initform cl:nil)
   (alarm_micro_swtich_
    :reader alarm_micro_swtich_
    :initarg :alarm_micro_swtich_
    :type cl:boolean
    :initform cl:nil)
   (alarm_collision_avoidance_
    :reader alarm_collision_avoidance_
    :initarg :alarm_collision_avoidance_
    :type cl:boolean
    :initform cl:nil)
   (alarm_emergency_stop_swtich_
    :reader alarm_emergency_stop_swtich_
    :initarg :alarm_emergency_stop_swtich_
    :type cl:boolean
    :initform cl:nil)
   (alarm_auto_man_swtich_
    :reader alarm_auto_man_swtich_
    :initarg :alarm_auto_man_swtich_
    :type cl:boolean
    :initform cl:nil)
   (alarm_up_limit_
    :reader alarm_up_limit_
    :initarg :alarm_up_limit_
    :type cl:boolean
    :initform cl:nil)
   (alarm_down_limit_
    :reader alarm_down_limit_
    :initarg :alarm_down_limit_
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass chassis_alarm (<chassis_alarm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chassis_alarm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chassis_alarm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<chassis_alarm> is deprecated: use chassis_drive-msg:chassis_alarm instead.")))

(cl:ensure-generic-function 'alarm_forkphotoe_left_-val :lambda-list '(m))
(cl:defmethod alarm_forkphotoe_left_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_forkphotoe_left_-val is deprecated.  Use chassis_drive-msg:alarm_forkphotoe_left_ instead.")
  (alarm_forkphotoe_left_ m))

(cl:ensure-generic-function 'alarm_forkphotoe_right_-val :lambda-list '(m))
(cl:defmethod alarm_forkphotoe_right_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_forkphotoe_right_-val is deprecated.  Use chassis_drive-msg:alarm_forkphotoe_right_ instead.")
  (alarm_forkphotoe_right_ m))

(cl:ensure-generic-function 'alarm_cargophotos_left_-val :lambda-list '(m))
(cl:defmethod alarm_cargophotos_left_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_cargophotos_left_-val is deprecated.  Use chassis_drive-msg:alarm_cargophotos_left_ instead.")
  (alarm_cargophotos_left_ m))

(cl:ensure-generic-function 'alarm_cargophotos_right_-val :lambda-list '(m))
(cl:defmethod alarm_cargophotos_right_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_cargophotos_right_-val is deprecated.  Use chassis_drive-msg:alarm_cargophotos_right_ instead.")
  (alarm_cargophotos_right_ m))

(cl:ensure-generic-function 'alarm_micro_swtich_-val :lambda-list '(m))
(cl:defmethod alarm_micro_swtich_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_micro_swtich_-val is deprecated.  Use chassis_drive-msg:alarm_micro_swtich_ instead.")
  (alarm_micro_swtich_ m))

(cl:ensure-generic-function 'alarm_collision_avoidance_-val :lambda-list '(m))
(cl:defmethod alarm_collision_avoidance_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_collision_avoidance_-val is deprecated.  Use chassis_drive-msg:alarm_collision_avoidance_ instead.")
  (alarm_collision_avoidance_ m))

(cl:ensure-generic-function 'alarm_emergency_stop_swtich_-val :lambda-list '(m))
(cl:defmethod alarm_emergency_stop_swtich_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_emergency_stop_swtich_-val is deprecated.  Use chassis_drive-msg:alarm_emergency_stop_swtich_ instead.")
  (alarm_emergency_stop_swtich_ m))

(cl:ensure-generic-function 'alarm_auto_man_swtich_-val :lambda-list '(m))
(cl:defmethod alarm_auto_man_swtich_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_auto_man_swtich_-val is deprecated.  Use chassis_drive-msg:alarm_auto_man_swtich_ instead.")
  (alarm_auto_man_swtich_ m))

(cl:ensure-generic-function 'alarm_up_limit_-val :lambda-list '(m))
(cl:defmethod alarm_up_limit_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_up_limit_-val is deprecated.  Use chassis_drive-msg:alarm_up_limit_ instead.")
  (alarm_up_limit_ m))

(cl:ensure-generic-function 'alarm_down_limit_-val :lambda-list '(m))
(cl:defmethod alarm_down_limit_-val ((m <chassis_alarm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:alarm_down_limit_-val is deprecated.  Use chassis_drive-msg:alarm_down_limit_ instead.")
  (alarm_down_limit_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chassis_alarm>) ostream)
  "Serializes a message object of type '<chassis_alarm>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_forkphotoe_left_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_forkphotoe_right_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_cargophotos_left_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_cargophotos_right_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_micro_swtich_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_collision_avoidance_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_emergency_stop_swtich_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_auto_man_swtich_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_up_limit_) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'alarm_down_limit_) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chassis_alarm>) istream)
  "Deserializes a message object of type '<chassis_alarm>"
    (cl:setf (cl:slot-value msg 'alarm_forkphotoe_left_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_forkphotoe_right_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_cargophotos_left_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_cargophotos_right_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_micro_swtich_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_collision_avoidance_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_emergency_stop_swtich_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_auto_man_swtich_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_up_limit_) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'alarm_down_limit_) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chassis_alarm>)))
  "Returns string type for a message object of type '<chassis_alarm>"
  "chassis_drive/chassis_alarm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chassis_alarm)))
  "Returns string type for a message object of type 'chassis_alarm"
  "chassis_drive/chassis_alarm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chassis_alarm>)))
  "Returns md5sum for a message object of type '<chassis_alarm>"
  "ae8c21bba8b1459751e77f410496d9c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chassis_alarm)))
  "Returns md5sum for a message object of type 'chassis_alarm"
  "ae8c21bba8b1459751e77f410496d9c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chassis_alarm>)))
  "Returns full string definition for message of type '<chassis_alarm>"
  (cl:format cl:nil "bool alarm_forkphotoe_left_~%bool alarm_forkphotoe_right_~%bool alarm_cargophotos_left_~%bool alarm_cargophotos_right_~%bool alarm_micro_swtich_~%bool alarm_collision_avoidance_~%bool alarm_emergency_stop_swtich_~%bool alarm_auto_man_swtich_~%bool alarm_up_limit_~%bool alarm_down_limit_~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chassis_alarm)))
  "Returns full string definition for message of type 'chassis_alarm"
  (cl:format cl:nil "bool alarm_forkphotoe_left_~%bool alarm_forkphotoe_right_~%bool alarm_cargophotos_left_~%bool alarm_cargophotos_right_~%bool alarm_micro_swtich_~%bool alarm_collision_avoidance_~%bool alarm_emergency_stop_swtich_~%bool alarm_auto_man_swtich_~%bool alarm_up_limit_~%bool alarm_down_limit_~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chassis_alarm>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chassis_alarm>))
  "Converts a ROS message object to a list"
  (cl:list 'chassis_alarm
    (cl:cons ':alarm_forkphotoe_left_ (alarm_forkphotoe_left_ msg))
    (cl:cons ':alarm_forkphotoe_right_ (alarm_forkphotoe_right_ msg))
    (cl:cons ':alarm_cargophotos_left_ (alarm_cargophotos_left_ msg))
    (cl:cons ':alarm_cargophotos_right_ (alarm_cargophotos_right_ msg))
    (cl:cons ':alarm_micro_swtich_ (alarm_micro_swtich_ msg))
    (cl:cons ':alarm_collision_avoidance_ (alarm_collision_avoidance_ msg))
    (cl:cons ':alarm_emergency_stop_swtich_ (alarm_emergency_stop_swtich_ msg))
    (cl:cons ':alarm_auto_man_swtich_ (alarm_auto_man_swtich_ msg))
    (cl:cons ':alarm_up_limit_ (alarm_up_limit_ msg))
    (cl:cons ':alarm_down_limit_ (alarm_down_limit_ msg))
))
