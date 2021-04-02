; Auto-generated. Do not edit!


(cl:in-package agvs_control-srv)


;//! \htmlinclude cmd_control_mode-request.msg.html

(cl:defclass <cmd_control_mode-request> (roslisp-msg-protocol:ros-message)
  ((mode_run
    :reader mode_run
    :initarg :mode_run
    :type cl:string
    :initform ""))
)

(cl:defclass cmd_control_mode-request (<cmd_control_mode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_control_mode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_control_mode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-srv:<cmd_control_mode-request> is deprecated: use agvs_control-srv:cmd_control_mode-request instead.")))

(cl:ensure-generic-function 'mode_run-val :lambda-list '(m))
(cl:defmethod mode_run-val ((m <cmd_control_mode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-srv:mode_run-val is deprecated.  Use agvs_control-srv:mode_run instead.")
  (mode_run m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<cmd_control_mode-request>)))
    "Constants for message type '<cmd_control_mode-request>"
  '((:MANUAL_MODE . "manual_mode")
    (:AUTO_MODE . "auto_mode"))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'cmd_control_mode-request)))
    "Constants for message type 'cmd_control_mode-request"
  '((:MANUAL_MODE . "manual_mode")
    (:AUTO_MODE . "auto_mode"))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_control_mode-request>) ostream)
  "Serializes a message object of type '<cmd_control_mode-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode_run))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode_run))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_control_mode-request>) istream)
  "Deserializes a message object of type '<cmd_control_mode-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode_run) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode_run) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_control_mode-request>)))
  "Returns string type for a service object of type '<cmd_control_mode-request>"
  "agvs_control/cmd_control_modeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_control_mode-request)))
  "Returns string type for a service object of type 'cmd_control_mode-request"
  "agvs_control/cmd_control_modeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_control_mode-request>)))
  "Returns md5sum for a message object of type '<cmd_control_mode-request>"
  "0545bf3db61acfe8473d040eb76fcb1c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_control_mode-request)))
  "Returns md5sum for a message object of type 'cmd_control_mode-request"
  "0545bf3db61acfe8473d040eb76fcb1c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_control_mode-request>)))
  "Returns full string definition for message of type '<cmd_control_mode-request>"
  (cl:format cl:nil "string mode_run ~%string manual_mode=\"manual_mode\"~%string auto_mode=\"auto_mode\"~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_control_mode-request)))
  "Returns full string definition for message of type 'cmd_control_mode-request"
  (cl:format cl:nil "string mode_run ~%string manual_mode=\"manual_mode\"~%string auto_mode=\"auto_mode\"~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_control_mode-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode_run))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_control_mode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_control_mode-request
    (cl:cons ':mode_run (mode_run msg))
))
;//! \htmlinclude cmd_control_mode-response.msg.html

(cl:defclass <cmd_control_mode-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass cmd_control_mode-response (<cmd_control_mode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_control_mode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_control_mode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-srv:<cmd_control_mode-response> is deprecated: use agvs_control-srv:cmd_control_mode-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_control_mode-response>) ostream)
  "Serializes a message object of type '<cmd_control_mode-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_control_mode-response>) istream)
  "Deserializes a message object of type '<cmd_control_mode-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_control_mode-response>)))
  "Returns string type for a service object of type '<cmd_control_mode-response>"
  "agvs_control/cmd_control_modeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_control_mode-response)))
  "Returns string type for a service object of type 'cmd_control_mode-response"
  "agvs_control/cmd_control_modeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_control_mode-response>)))
  "Returns md5sum for a message object of type '<cmd_control_mode-response>"
  "0545bf3db61acfe8473d040eb76fcb1c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_control_mode-response)))
  "Returns md5sum for a message object of type 'cmd_control_mode-response"
  "0545bf3db61acfe8473d040eb76fcb1c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_control_mode-response>)))
  "Returns full string definition for message of type '<cmd_control_mode-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_control_mode-response)))
  "Returns full string definition for message of type 'cmd_control_mode-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_control_mode-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_control_mode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_control_mode-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cmd_control_mode)))
  'cmd_control_mode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cmd_control_mode)))
  'cmd_control_mode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_control_mode)))
  "Returns string type for a service object of type '<cmd_control_mode>"
  "agvs_control/cmd_control_mode")