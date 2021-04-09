; Auto-generated. Do not edit!


(cl:in-package agvs_control-msg)


;//! \htmlinclude slam_data.msg.html

(cl:defclass <slam_data> (roslisp-msg-protocol:ros-message)
  ((theta_x
    :reader theta_x
    :initarg :theta_x
    :type cl:float
    :initform 0.0)
   (theta_y
    :reader theta_y
    :initarg :theta_y
    :type cl:float
    :initform 0.0)
   (theta_angle
    :reader theta_angle
    :initarg :theta_angle
    :type cl:float
    :initform 0.0)
   (speed_x
    :reader speed_x
    :initarg :speed_x
    :type cl:float
    :initform 0.0)
   (speed_y
    :reader speed_y
    :initarg :speed_y
    :type cl:float
    :initform 0.0)
   (speed_z
    :reader speed_z
    :initarg :speed_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass slam_data (<slam_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <slam_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'slam_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-msg:<slam_data> is deprecated: use agvs_control-msg:slam_data instead.")))

(cl:ensure-generic-function 'theta_x-val :lambda-list '(m))
(cl:defmethod theta_x-val ((m <slam_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:theta_x-val is deprecated.  Use agvs_control-msg:theta_x instead.")
  (theta_x m))

(cl:ensure-generic-function 'theta_y-val :lambda-list '(m))
(cl:defmethod theta_y-val ((m <slam_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:theta_y-val is deprecated.  Use agvs_control-msg:theta_y instead.")
  (theta_y m))

(cl:ensure-generic-function 'theta_angle-val :lambda-list '(m))
(cl:defmethod theta_angle-val ((m <slam_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:theta_angle-val is deprecated.  Use agvs_control-msg:theta_angle instead.")
  (theta_angle m))

(cl:ensure-generic-function 'speed_x-val :lambda-list '(m))
(cl:defmethod speed_x-val ((m <slam_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:speed_x-val is deprecated.  Use agvs_control-msg:speed_x instead.")
  (speed_x m))

(cl:ensure-generic-function 'speed_y-val :lambda-list '(m))
(cl:defmethod speed_y-val ((m <slam_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:speed_y-val is deprecated.  Use agvs_control-msg:speed_y instead.")
  (speed_y m))

(cl:ensure-generic-function 'speed_z-val :lambda-list '(m))
(cl:defmethod speed_z-val ((m <slam_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:speed_z-val is deprecated.  Use agvs_control-msg:speed_z instead.")
  (speed_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <slam_data>) ostream)
  "Serializes a message object of type '<slam_data>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <slam_data>) istream)
  "Deserializes a message object of type '<slam_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<slam_data>)))
  "Returns string type for a message object of type '<slam_data>"
  "agvs_control/slam_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'slam_data)))
  "Returns string type for a message object of type 'slam_data"
  "agvs_control/slam_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<slam_data>)))
  "Returns md5sum for a message object of type '<slam_data>"
  "91db6ffd3f1cb419025d2f1a637f02e6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'slam_data)))
  "Returns md5sum for a message object of type 'slam_data"
  "91db6ffd3f1cb419025d2f1a637f02e6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<slam_data>)))
  "Returns full string definition for message of type '<slam_data>"
  (cl:format cl:nil "float32 theta_x~%float32 theta_y~%float32 theta_angle~%float32 speed_x~%float32 speed_y~%float32 speed_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'slam_data)))
  "Returns full string definition for message of type 'slam_data"
  (cl:format cl:nil "float32 theta_x~%float32 theta_y~%float32 theta_angle~%float32 speed_x~%float32 speed_y~%float32 speed_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <slam_data>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <slam_data>))
  "Converts a ROS message object to a list"
  (cl:list 'slam_data
    (cl:cons ':theta_x (theta_x msg))
    (cl:cons ':theta_y (theta_y msg))
    (cl:cons ':theta_angle (theta_angle msg))
    (cl:cons ':speed_x (speed_x msg))
    (cl:cons ':speed_y (speed_y msg))
    (cl:cons ':speed_z (speed_z msg))
))
