; Auto-generated. Do not edit!


(cl:in-package agvs_control-msg)


;//! \htmlinclude date_pads_cmd.msg.html

(cl:defclass <date_pads_cmd> (roslisp-msg-protocol:ros-message)
  ((angle_date
    :reader angle_date
    :initarg :angle_date
    :type cl:float
    :initform 0.0)
   (speed_date
    :reader speed_date
    :initarg :speed_date
    :type cl:float
    :initform 0.0))
)

(cl:defclass date_pads_cmd (<date_pads_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <date_pads_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'date_pads_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-msg:<date_pads_cmd> is deprecated: use agvs_control-msg:date_pads_cmd instead.")))

(cl:ensure-generic-function 'angle_date-val :lambda-list '(m))
(cl:defmethod angle_date-val ((m <date_pads_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:angle_date-val is deprecated.  Use agvs_control-msg:angle_date instead.")
  (angle_date m))

(cl:ensure-generic-function 'speed_date-val :lambda-list '(m))
(cl:defmethod speed_date-val ((m <date_pads_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-msg:speed_date-val is deprecated.  Use agvs_control-msg:speed_date instead.")
  (speed_date m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <date_pads_cmd>) ostream)
  "Serializes a message object of type '<date_pads_cmd>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_date))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_date))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <date_pads_cmd>) istream)
  "Deserializes a message object of type '<date_pads_cmd>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_date) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_date) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<date_pads_cmd>)))
  "Returns string type for a message object of type '<date_pads_cmd>"
  "agvs_control/date_pads_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'date_pads_cmd)))
  "Returns string type for a message object of type 'date_pads_cmd"
  "agvs_control/date_pads_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<date_pads_cmd>)))
  "Returns md5sum for a message object of type '<date_pads_cmd>"
  "400b13fd7cc1c6f886c0fcc01fcab9aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'date_pads_cmd)))
  "Returns md5sum for a message object of type 'date_pads_cmd"
  "400b13fd7cc1c6f886c0fcc01fcab9aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<date_pads_cmd>)))
  "Returns full string definition for message of type '<date_pads_cmd>"
  (cl:format cl:nil "float32 angle_date~%float32 speed_date~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'date_pads_cmd)))
  "Returns full string definition for message of type 'date_pads_cmd"
  (cl:format cl:nil "float32 angle_date~%float32 speed_date~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <date_pads_cmd>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <date_pads_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'date_pads_cmd
    (cl:cons ':angle_date (angle_date msg))
    (cl:cons ':speed_date (speed_date msg))
))
