; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude agvs_mode.msg.html

(cl:defclass <agvs_mode> (roslisp-msg-protocol:ros-message)
  ((current_mode
    :reader current_mode
    :initarg :current_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass agvs_mode (<agvs_mode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agvs_mode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agvs_mode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<agvs_mode> is deprecated: use chassis_drive-msg:agvs_mode instead.")))

(cl:ensure-generic-function 'current_mode-val :lambda-list '(m))
(cl:defmethod current_mode-val ((m <agvs_mode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:current_mode-val is deprecated.  Use chassis_drive-msg:current_mode instead.")
  (current_mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<agvs_mode>)))
    "Constants for message type '<agvs_mode>"
  '((:AGVS_MODE_MANUAL . 0)
    (:AGVS_MODE_AUTO_FLOOR_1 . 1)
    (:AGVS_MODE_AUTO_FLOOR_2 . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'agvs_mode)))
    "Constants for message type 'agvs_mode"
  '((:AGVS_MODE_MANUAL . 0)
    (:AGVS_MODE_AUTO_FLOOR_1 . 1)
    (:AGVS_MODE_AUTO_FLOOR_2 . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agvs_mode>) ostream)
  "Serializes a message object of type '<agvs_mode>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agvs_mode>) istream)
  "Deserializes a message object of type '<agvs_mode>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'current_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agvs_mode>)))
  "Returns string type for a message object of type '<agvs_mode>"
  "chassis_drive/agvs_mode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agvs_mode)))
  "Returns string type for a message object of type 'agvs_mode"
  "chassis_drive/agvs_mode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agvs_mode>)))
  "Returns md5sum for a message object of type '<agvs_mode>"
  "334b5039384f49ff4aa9ee0483feb402")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agvs_mode)))
  "Returns md5sum for a message object of type 'agvs_mode"
  "334b5039384f49ff4aa9ee0483feb402")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agvs_mode>)))
  "Returns full string definition for message of type '<agvs_mode>"
  (cl:format cl:nil "uint8 current_mode~%~%uint8 agvs_mode_manual =0~%uint8 agvs_mode_auto_floor_1 = 1~%uint8 agvs_mode_auto_floor_2 = 2~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agvs_mode)))
  "Returns full string definition for message of type 'agvs_mode"
  (cl:format cl:nil "uint8 current_mode~%~%uint8 agvs_mode_manual =0~%uint8 agvs_mode_auto_floor_1 = 1~%uint8 agvs_mode_auto_floor_2 = 2~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agvs_mode>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agvs_mode>))
  "Converts a ROS message object to a list"
  (cl:list 'agvs_mode
    (cl:cons ':current_mode (current_mode msg))
))
