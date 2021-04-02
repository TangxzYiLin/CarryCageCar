; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude chassis_state.msg.html

(cl:defclass <chassis_state> (roslisp-msg-protocol:ros-message)
  ((chassis_mileage_variable_
    :reader chassis_mileage_variable_
    :initarg :chassis_mileage_variable_
    :type cl:integer
    :initform 0)
   (chassis_mileage_record_
    :reader chassis_mileage_record_
    :initarg :chassis_mileage_record_
    :type cl:integer
    :initform 0)
   (chassis_drive_speed_
    :reader chassis_drive_speed_
    :initarg :chassis_drive_speed_
    :type cl:fixnum
    :initform 0)
   (chassis_drive_angle_
    :reader chassis_drive_angle_
    :initarg :chassis_drive_angle_
    :type cl:fixnum
    :initform 0)
   (chassis_drivemotor_error_code_
    :reader chassis_drivemotor_error_code_
    :initarg :chassis_drivemotor_error_code_
    :type cl:fixnum
    :initform 0)
   (chassis_whirlmotor_erro_code_
    :reader chassis_whirlmotor_erro_code_
    :initarg :chassis_whirlmotor_erro_code_
    :type cl:fixnum
    :initform 0)
   (chassis_liftmotor_erro_code_
    :reader chassis_liftmotor_erro_code_
    :initarg :chassis_liftmotor_erro_code_
    :type cl:fixnum
    :initform 0)
   (chassis_task_id_
    :reader chassis_task_id_
    :initarg :chassis_task_id_
    :type cl:fixnum
    :initform 0)
   (chassis_task_state_
    :reader chassis_task_state_
    :initarg :chassis_task_state_
    :type cl:fixnum
    :initform 0)
   (chassis_selfcheck_error_code_
    :reader chassis_selfcheck_error_code_
    :initarg :chassis_selfcheck_error_code_
    :type cl:fixnum
    :initform 0))
)

(cl:defclass chassis_state (<chassis_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <chassis_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'chassis_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<chassis_state> is deprecated: use chassis_drive-msg:chassis_state instead.")))

(cl:ensure-generic-function 'chassis_mileage_variable_-val :lambda-list '(m))
(cl:defmethod chassis_mileage_variable_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_mileage_variable_-val is deprecated.  Use chassis_drive-msg:chassis_mileage_variable_ instead.")
  (chassis_mileage_variable_ m))

(cl:ensure-generic-function 'chassis_mileage_record_-val :lambda-list '(m))
(cl:defmethod chassis_mileage_record_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_mileage_record_-val is deprecated.  Use chassis_drive-msg:chassis_mileage_record_ instead.")
  (chassis_mileage_record_ m))

(cl:ensure-generic-function 'chassis_drive_speed_-val :lambda-list '(m))
(cl:defmethod chassis_drive_speed_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_drive_speed_-val is deprecated.  Use chassis_drive-msg:chassis_drive_speed_ instead.")
  (chassis_drive_speed_ m))

(cl:ensure-generic-function 'chassis_drive_angle_-val :lambda-list '(m))
(cl:defmethod chassis_drive_angle_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_drive_angle_-val is deprecated.  Use chassis_drive-msg:chassis_drive_angle_ instead.")
  (chassis_drive_angle_ m))

(cl:ensure-generic-function 'chassis_drivemotor_error_code_-val :lambda-list '(m))
(cl:defmethod chassis_drivemotor_error_code_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_drivemotor_error_code_-val is deprecated.  Use chassis_drive-msg:chassis_drivemotor_error_code_ instead.")
  (chassis_drivemotor_error_code_ m))

(cl:ensure-generic-function 'chassis_whirlmotor_erro_code_-val :lambda-list '(m))
(cl:defmethod chassis_whirlmotor_erro_code_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_whirlmotor_erro_code_-val is deprecated.  Use chassis_drive-msg:chassis_whirlmotor_erro_code_ instead.")
  (chassis_whirlmotor_erro_code_ m))

(cl:ensure-generic-function 'chassis_liftmotor_erro_code_-val :lambda-list '(m))
(cl:defmethod chassis_liftmotor_erro_code_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_liftmotor_erro_code_-val is deprecated.  Use chassis_drive-msg:chassis_liftmotor_erro_code_ instead.")
  (chassis_liftmotor_erro_code_ m))

(cl:ensure-generic-function 'chassis_task_id_-val :lambda-list '(m))
(cl:defmethod chassis_task_id_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_task_id_-val is deprecated.  Use chassis_drive-msg:chassis_task_id_ instead.")
  (chassis_task_id_ m))

(cl:ensure-generic-function 'chassis_task_state_-val :lambda-list '(m))
(cl:defmethod chassis_task_state_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_task_state_-val is deprecated.  Use chassis_drive-msg:chassis_task_state_ instead.")
  (chassis_task_state_ m))

(cl:ensure-generic-function 'chassis_selfcheck_error_code_-val :lambda-list '(m))
(cl:defmethod chassis_selfcheck_error_code_-val ((m <chassis_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:chassis_selfcheck_error_code_-val is deprecated.  Use chassis_drive-msg:chassis_selfcheck_error_code_ instead.")
  (chassis_selfcheck_error_code_ m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <chassis_state>) ostream)
  "Serializes a message object of type '<chassis_state>"
  (cl:let* ((signed (cl:slot-value msg 'chassis_mileage_variable_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_mileage_record_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_drive_speed_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_drive_angle_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_drivemotor_error_code_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_whirlmotor_erro_code_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_liftmotor_erro_code_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_task_id_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_task_state_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'chassis_selfcheck_error_code_)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <chassis_state>) istream)
  "Deserializes a message object of type '<chassis_state>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_mileage_variable_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_mileage_record_) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_drive_speed_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_drive_angle_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_drivemotor_error_code_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_whirlmotor_erro_code_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_liftmotor_erro_code_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_task_id_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_task_state_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'chassis_selfcheck_error_code_) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<chassis_state>)))
  "Returns string type for a message object of type '<chassis_state>"
  "chassis_drive/chassis_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'chassis_state)))
  "Returns string type for a message object of type 'chassis_state"
  "chassis_drive/chassis_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<chassis_state>)))
  "Returns md5sum for a message object of type '<chassis_state>"
  "d74b78807fa4e4349088bf6fbedc2966")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'chassis_state)))
  "Returns md5sum for a message object of type 'chassis_state"
  "d74b78807fa4e4349088bf6fbedc2966")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<chassis_state>)))
  "Returns full string definition for message of type '<chassis_state>"
  (cl:format cl:nil "int32 chassis_mileage_variable_~%int32 chassis_mileage_record_~%int16 chassis_drive_speed_~%int16 chassis_drive_angle_~%int16 chassis_drivemotor_error_code_~%int16 chassis_whirlmotor_erro_code_~%int16 chassis_liftmotor_erro_code_~%~%int16 chassis_task_id_~%int16 chassis_task_state_~%int16 chassis_selfcheck_error_code_~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'chassis_state)))
  "Returns full string definition for message of type 'chassis_state"
  (cl:format cl:nil "int32 chassis_mileage_variable_~%int32 chassis_mileage_record_~%int16 chassis_drive_speed_~%int16 chassis_drive_angle_~%int16 chassis_drivemotor_error_code_~%int16 chassis_whirlmotor_erro_code_~%int16 chassis_liftmotor_erro_code_~%~%int16 chassis_task_id_~%int16 chassis_task_state_~%int16 chassis_selfcheck_error_code_~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <chassis_state>))
  (cl:+ 0
     4
     4
     2
     2
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <chassis_state>))
  "Converts a ROS message object to a list"
  (cl:list 'chassis_state
    (cl:cons ':chassis_mileage_variable_ (chassis_mileage_variable_ msg))
    (cl:cons ':chassis_mileage_record_ (chassis_mileage_record_ msg))
    (cl:cons ':chassis_drive_speed_ (chassis_drive_speed_ msg))
    (cl:cons ':chassis_drive_angle_ (chassis_drive_angle_ msg))
    (cl:cons ':chassis_drivemotor_error_code_ (chassis_drivemotor_error_code_ msg))
    (cl:cons ':chassis_whirlmotor_erro_code_ (chassis_whirlmotor_erro_code_ msg))
    (cl:cons ':chassis_liftmotor_erro_code_ (chassis_liftmotor_erro_code_ msg))
    (cl:cons ':chassis_task_id_ (chassis_task_id_ msg))
    (cl:cons ':chassis_task_state_ (chassis_task_state_ msg))
    (cl:cons ':chassis_selfcheck_error_code_ (chassis_selfcheck_error_code_ msg))
))
