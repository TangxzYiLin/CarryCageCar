; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude chassis_bat.msg.html

(cl:defclass <chassis_bat> (roslisp-msg-protocol:ros-message)
  ((bat_cap_
    :reader bat_cap_
    :initarg :bat_cap_
    :type cl:fixnum
    :initform 0)
   (bat_vol_
    :reader bat_vol_
    :initarg :bat_vol_
    :type cl:fixnum
    :initform 0)
   (bat_cur_
    :reader bat_cur_
    :initarg :bat_cur_
    :type cl:fixnum
    :initform 0)
   (bat_error_code_
    :reader bat_error_code_
    :initarg :bat_error_code_
    :type cl:fixnum
    :initform 0))
)

(cl:defclass chassis_bat (<chassis_bat>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chassis_bat>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chassis_bat)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<chassis_bat> is deprecated: use chassis_drive-msg:chassis_bat instead.")))

(cl:ensure-generic-function 'bat_cap_-val :lambda-list '(m))
(cl:defmethod bat_cap_-val ((m <chassis_bat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:bat_cap_-val is deprecated.  Use chassis_drive-msg:bat_cap_ instead.")
  (bat_cap_ m))

(cl:ensure-generic-function 'bat_vol_-val :lambda-list '(m))
(cl:defmethod bat_vol_-val ((m <chassis_bat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:bat_vol_-val is deprecated.  Use chassis_drive-msg:bat_vol_ instead.")
  (bat_vol_ m))

(cl:ensure-generic-function 'bat_cur_-val :lambda-list '(m))
(cl:defmethod bat_cur_-val ((m <chassis_bat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:bat_cur_-val is deprecated.  Use chassis_drive-msg:bat_cur_ instead.")
  (bat_cur_ m))

(cl:ensure-generic-function 'bat_error_code_-val :lambda-list '(m))
(cl:defmethod bat_error_code_-val ((m <chassis_bat>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:bat_error_code_-val is deprecated.  Use chassis_drive-msg:bat_error_code_ instead.")
  (bat_error_code_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chassis_bat>) ostream)
  "Serializes a message object of type '<chassis_bat>"
  (cl:let* ((signed (cl:slot-value msg 'bat_cap_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bat_vol_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bat_cur_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'bat_error_code_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chassis_bat>) istream)
  "Deserializes a message object of type '<chassis_bat>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bat_cap_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bat_vol_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bat_cur_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bat_error_code_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chassis_bat>)))
  "Returns string type for a message object of type '<chassis_bat>"
  "chassis_drive/chassis_bat")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chassis_bat)))
  "Returns string type for a message object of type 'chassis_bat"
  "chassis_drive/chassis_bat")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chassis_bat>)))
  "Returns md5sum for a message object of type '<chassis_bat>"
  "1d7bde1a0bdfdf9227bac0927d779d8f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chassis_bat)))
  "Returns md5sum for a message object of type 'chassis_bat"
  "1d7bde1a0bdfdf9227bac0927d779d8f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chassis_bat>)))
  "Returns full string definition for message of type '<chassis_bat>"
  (cl:format cl:nil "int16 bat_cap_~%int16 bat_vol_~%int16 bat_cur_~%int16 bat_error_code_~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chassis_bat)))
  "Returns full string definition for message of type 'chassis_bat"
  (cl:format cl:nil "int16 bat_cap_~%int16 bat_vol_~%int16 bat_cur_~%int16 bat_error_code_~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chassis_bat>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chassis_bat>))
  "Converts a ROS message object to a list"
  (cl:list 'chassis_bat
    (cl:cons ':bat_cap_ (bat_cap_ msg))
    (cl:cons ':bat_vol_ (bat_vol_ msg))
    (cl:cons ':bat_cur_ (bat_cur_ msg))
    (cl:cons ':bat_error_code_ (bat_error_code_ msg))
))
