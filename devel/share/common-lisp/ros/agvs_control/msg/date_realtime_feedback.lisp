; Auto-generated. Do not edit!


(cl:in-package agvs_control-msg)


;//! \htmlinclude date_realtime_feedback.msg.html

(cl:defclass <date_realtime_feedback> (roslisp-msg-protocol:ros-message)
  ((theta_angle
    :reader theta_angle
    :initarg :theta_angle
    :type cl:float
    :initform 0.0)
   (theta_x
    :reader theta_x
    :initarg :theta_x
    :type cl:float
    :initform 0.0)
   (theta_y
    :reader theta_y
    :initarg :theta_y
    :type cl:float
    :initform 0.0)
   (speed_y
    :reader speed_y
    :initarg :speed_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass date_realtime_feedback (<date_realtime_feedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <date_realtime_feedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'date_realtime_feedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-msg:<date_realtime_feedback> is deprecated: use agvs_control-msg:date_realtime_feedback instead.")))

(cl:ensure-generic-function 'theta_angle-val :lambda-list '(m))
(cl:defmethod theta_angle-val ((m <date_realtime_feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:theta_angle-val is deprecated.  Use agvs_control-msg:theta_angle instead.")
  (theta_angle m))

(cl:ensure-generic-function 'theta_x-val :lambda-list '(m))
(cl:defmethod theta_x-val ((m <date_realtime_feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:theta_x-val is deprecated.  Use agvs_control-msg:theta_x instead.")
  (theta_x m))

(cl:ensure-generic-function 'theta_y-val :lambda-list '(m))
(cl:defmethod theta_y-val ((m <date_realtime_feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:theta_y-val is deprecated.  Use agvs_control-msg:theta_y instead.")
  (theta_y m))

(cl:ensure-generic-function 'speed_y-val :lambda-list '(m))
(cl:defmethod speed_y-val ((m <date_realtime_feedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:speed_y-val is deprecated.  Use agvs_control-msg:speed_y instead.")
  (speed_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <date_realtime_feedback>) ostream)
  "Serializes a message object of type '<date_realtime_feedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <date_realtime_feedback>) istream)
  "Deserializes a message object of type '<date_realtime_feedback>"
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
    (cl:setf (cl:slot-value msg 'speed_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<date_realtime_feedback>)))
  "Returns string type for a message object of type '<date_realtime_feedback>"
  "agvs_control/date_realtime_feedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'date_realtime_feedback)))
  "Returns string type for a message object of type 'date_realtime_feedback"
  "agvs_control/date_realtime_feedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<date_realtime_feedback>)))
  "Returns md5sum for a message object of type '<date_realtime_feedback>"
  "9373b18cc1268d229a49f9c7a9f9ccc5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'date_realtime_feedback)))
  "Returns md5sum for a message object of type 'date_realtime_feedback"
  "9373b18cc1268d229a49f9c7a9f9ccc5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<date_realtime_feedback>)))
  "Returns full string definition for message of type '<date_realtime_feedback>"
  (cl:format cl:nil "float32 theta_angle~%float32 theta_x~%float32 theta_y~%~%float32 speed_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'date_realtime_feedback)))
  "Returns full string definition for message of type 'date_realtime_feedback"
  (cl:format cl:nil "float32 theta_angle~%float32 theta_x~%float32 theta_y~%~%float32 speed_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <date_realtime_feedback>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <date_realtime_feedback>))
  "Converts a ROS message object to a list"
  (cl:list 'date_realtime_feedback
    (cl:cons ':theta_angle (theta_angle msg))
    (cl:cons ':theta_x (theta_x msg))
    (cl:cons ':theta_y (theta_y msg))
    (cl:cons ':speed_y (speed_y msg))
))
