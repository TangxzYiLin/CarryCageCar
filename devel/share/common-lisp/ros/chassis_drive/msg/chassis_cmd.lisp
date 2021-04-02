; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude chassis_cmd.msg.html

(cl:defclass <chassis_cmd> (roslisp-msg-protocol:ros-message)
  ((chassis_vel_cmd_
    :reader chassis_vel_cmd_
    :initarg :chassis_vel_cmd_
    :type cl:fixnum
    :initform 0)
   (chassis_angle_cmd_
    :reader chassis_angle_cmd_
    :initarg :chassis_angle_cmd_
    :type cl:fixnum
    :initform 0)
   (chassis_indicator_cmd_
    :reader chassis_indicator_cmd_
    :initarg :chassis_indicator_cmd_
    :type cl:fixnum
    :initform 0)
   (chassis_brake_cmd_
    :reader chassis_brake_cmd_
    :initarg :chassis_brake_cmd_
    :type cl:fixnum
    :initform 0))
)

(cl:defclass chassis_cmd (<chassis_cmd>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chassis_cmd>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chassis_cmd)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<chassis_cmd> is deprecated: use chassis_drive-msg:chassis_cmd instead.")))

(cl:ensure-generic-function 'chassis_vel_cmd_-val :lambda-list '(m))
(cl:defmethod chassis_vel_cmd_-val ((m <chassis_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_vel_cmd_-val is deprecated.  Use chassis_drive-msg:chassis_vel_cmd_ instead.")
  (chassis_vel_cmd_ m))

(cl:ensure-generic-function 'chassis_angle_cmd_-val :lambda-list '(m))
(cl:defmethod chassis_angle_cmd_-val ((m <chassis_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_angle_cmd_-val is deprecated.  Use chassis_drive-msg:chassis_angle_cmd_ instead.")
  (chassis_angle_cmd_ m))

(cl:ensure-generic-function 'chassis_indicator_cmd_-val :lambda-list '(m))
(cl:defmethod chassis_indicator_cmd_-val ((m <chassis_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_indicator_cmd_-val is deprecated.  Use chassis_drive-msg:chassis_indicator_cmd_ instead.")
  (chassis_indicator_cmd_ m))

(cl:ensure-generic-function 'chassis_brake_cmd_-val :lambda-list '(m))
(cl:defmethod chassis_brake_cmd_-val ((m <chassis_cmd>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_brake_cmd_-val is deprecated.  Use chassis_drive-msg:chassis_brake_cmd_ instead.")
  (chassis_brake_cmd_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chassis_cmd>) ostream)
  "Serializes a message object of type '<chassis_cmd>"
  (cl:let* ((signed (cl:slot-value msg 'chassis_vel_cmd_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_angle_cmd_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_indicator_cmd_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_brake_cmd_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chassis_cmd>) istream)
  "Deserializes a message object of type '<chassis_cmd>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_vel_cmd_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_angle_cmd_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_indicator_cmd_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_brake_cmd_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chassis_cmd>)))
  "Returns string type for a message object of type '<chassis_cmd>"
  "chassis_drive/chassis_cmd")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chassis_cmd)))
  "Returns string type for a message object of type 'chassis_cmd"
  "chassis_drive/chassis_cmd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chassis_cmd>)))
  "Returns md5sum for a message object of type '<chassis_cmd>"
  "8c01b3f6487ca68f987b16e7e5933bc6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chassis_cmd)))
  "Returns md5sum for a message object of type 'chassis_cmd"
  "8c01b3f6487ca68f987b16e7e5933bc6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chassis_cmd>)))
  "Returns full string definition for message of type '<chassis_cmd>"
  (cl:format cl:nil "int16 chassis_vel_cmd_  ~%int16 chassis_angle_cmd_~%int16 chassis_indicator_cmd_~%int16 chassis_brake_cmd_~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chassis_cmd)))
  "Returns full string definition for message of type 'chassis_cmd"
  (cl:format cl:nil "int16 chassis_vel_cmd_  ~%int16 chassis_angle_cmd_~%int16 chassis_indicator_cmd_~%int16 chassis_brake_cmd_~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chassis_cmd>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chassis_cmd>))
  "Converts a ROS message object to a list"
  (cl:list 'chassis_cmd
    (cl:cons ':chassis_vel_cmd_ (chassis_vel_cmd_ msg))
    (cl:cons ':chassis_angle_cmd_ (chassis_angle_cmd_ msg))
    (cl:cons ':chassis_indicator_cmd_ (chassis_indicator_cmd_ msg))
    (cl:cons ':chassis_brake_cmd_ (chassis_brake_cmd_ msg))
))
