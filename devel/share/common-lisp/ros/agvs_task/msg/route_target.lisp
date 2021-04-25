; Auto-generated. Do not edit!


(cl:in-package agvs_task-msg)


;//! \htmlinclude route_target.msg.html

(cl:defclass <route_target> (roslisp-msg-protocol:ros-message)
  ((target_location_x
    :reader target_location_x
    :initarg :target_location_x
    :type cl:float
    :initform 0.0)
   (target_location_y
    :reader target_location_y
    :initarg :target_location_y
    :type cl:float
    :initform 0.0)
   (target_speed
    :reader target_speed
    :initarg :target_speed
    :type cl:float
    :initform 0.0)
   (task_direction
    :reader task_direction
    :initarg :task_direction
    :type cl:fixnum
    :initform 0)
   (task_route_id
    :reader task_route_id
    :initarg :task_route_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass route_target (<route_target>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <route_target>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'route_target)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_task-msg:<route_target> is deprecated: use agvs_task-msg:route_target instead.")))

(cl:ensure-generic-function 'target_location_x-val :lambda-list '(m))
(cl:defmethod target_location_x-val ((m <route_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_task-msg:target_location_x-val is deprecated.  Use agvs_task-msg:target_location_x instead.")
  (target_location_x m))

(cl:ensure-generic-function 'target_location_y-val :lambda-list '(m))
(cl:defmethod target_location_y-val ((m <route_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_task-msg:target_location_y-val is deprecated.  Use agvs_task-msg:target_location_y instead.")
  (target_location_y m))

(cl:ensure-generic-function 'target_speed-val :lambda-list '(m))
(cl:defmethod target_speed-val ((m <route_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_task-msg:target_speed-val is deprecated.  Use agvs_task-msg:target_speed instead.")
  (target_speed m))

(cl:ensure-generic-function 'task_direction-val :lambda-list '(m))
(cl:defmethod task_direction-val ((m <route_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_task-msg:task_direction-val is deprecated.  Use agvs_task-msg:task_direction instead.")
  (task_direction m))

(cl:ensure-generic-function 'task_route_id-val :lambda-list '(m))
(cl:defmethod task_route_id-val ((m <route_target>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_task-msg:task_route_id-val is deprecated.  Use agvs_task-msg:task_route_id instead.")
  (task_route_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <route_target>) ostream)
  "Serializes a message object of type '<route_target>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_location_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_location_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'task_direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'task_route_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <route_target>) istream)
  "Deserializes a message object of type '<route_target>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_location_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_location_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_direction) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'task_route_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<route_target>)))
  "Returns string type for a message object of type '<route_target>"
  "agvs_task/route_target")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'route_target)))
  "Returns string type for a message object of type 'route_target"
  "agvs_task/route_target")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<route_target>)))
  "Returns md5sum for a message object of type '<route_target>"
  "6a82df07cc526c047af2d9e2d7bfca86")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'route_target)))
  "Returns md5sum for a message object of type 'route_target"
  "6a82df07cc526c047af2d9e2d7bfca86")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<route_target>)))
  "Returns full string definition for message of type '<route_target>"
  (cl:format cl:nil "float32 target_location_x~%float32 target_location_y~%float32 target_speed~%int8 task_direction~%int8 task_route_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'route_target)))
  "Returns full string definition for message of type 'route_target"
  (cl:format cl:nil "float32 target_location_x~%float32 target_location_y~%float32 target_speed~%int8 task_direction~%int8 task_route_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <route_target>))
  (cl:+ 0
     4
     4
     4
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <route_target>))
  "Converts a ROS message object to a list"
  (cl:list 'route_target
    (cl:cons ':target_location_x (target_location_x msg))
    (cl:cons ':target_location_y (target_location_y msg))
    (cl:cons ':target_speed (target_speed msg))
    (cl:cons ':task_direction (task_direction msg))
    (cl:cons ':task_route_id (task_route_id msg))
))
