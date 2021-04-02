; Auto-generated. Do not edit!


(cl:in-package agvs_control-srv)


;//! \htmlinclude cmd_control_task-request.msg.html

(cl:defclass <cmd_control_task-request> (roslisp-msg-protocol:ros-message)
  ((mode_run
    :reader mode_run
    :initarg :mode_run
    :type cl:fixnum
    :initform 0))
)

(cl:defclass cmd_control_task-request (<cmd_control_task-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_control_task-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_control_task-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-srv:<cmd_control_task-request> is deprecated: use agvs_control-srv:cmd_control_task-request instead.")))

(cl:ensure-generic-function 'mode_run-val :lambda-list '(m))
(cl:defmethod mode_run-val ((m <cmd_control_task-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_control-srv:mode_run-val is deprecated.  Use agvs_control-srv:mode_run instead.")
  (mode_run m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<cmd_control_task-request>)))
    "Constants for message type '<cmd_control_task-request>"
  '((:MANUAL_MODE . 1)
    (:AUTO_MODE . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'cmd_control_task-request)))
    "Constants for message type 'cmd_control_task-request"
  '((:MANUAL_MODE . 1)
    (:AUTO_MODE . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_control_task-request>) ostream)
  "Serializes a message object of type '<cmd_control_task-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_run)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_control_task-request>) istream)
  "Deserializes a message object of type '<cmd_control_task-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode_run)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_control_task-request>)))
  "Returns string type for a service object of type '<cmd_control_task-request>"
  "agvs_control/cmd_control_taskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_control_task-request)))
  "Returns string type for a service object of type 'cmd_control_task-request"
  "agvs_control/cmd_control_taskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_control_task-request>)))
  "Returns md5sum for a message object of type '<cmd_control_task-request>"
  "f9b5bf540932291b1fe9cba3efc63097")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_control_task-request)))
  "Returns md5sum for a message object of type 'cmd_control_task-request"
  "f9b5bf540932291b1fe9cba3efc63097")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_control_task-request>)))
  "Returns full string definition for message of type '<cmd_control_task-request>"
  (cl:format cl:nil "uint8 mode_run ~%uint8 manual_mode=1~%uint8 auto_mode=2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_control_task-request)))
  "Returns full string definition for message of type 'cmd_control_task-request"
  (cl:format cl:nil "uint8 mode_run ~%uint8 manual_mode=1~%uint8 auto_mode=2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_control_task-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_control_task-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_control_task-request
    (cl:cons ':mode_run (mode_run msg))
))
;//! \htmlinclude cmd_control_task-response.msg.html

(cl:defclass <cmd_control_task-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass cmd_control_task-response (<cmd_control_task-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_control_task-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_control_task-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_control-srv:<cmd_control_task-response> is deprecated: use agvs_control-srv:cmd_control_task-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_control_task-response>) ostream)
  "Serializes a message object of type '<cmd_control_task-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_control_task-response>) istream)
  "Deserializes a message object of type '<cmd_control_task-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_control_task-response>)))
  "Returns string type for a service object of type '<cmd_control_task-response>"
  "agvs_control/cmd_control_taskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_control_task-response)))
  "Returns string type for a service object of type 'cmd_control_task-response"
  "agvs_control/cmd_control_taskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_control_task-response>)))
  "Returns md5sum for a message object of type '<cmd_control_task-response>"
  "f9b5bf540932291b1fe9cba3efc63097")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_control_task-response)))
  "Returns md5sum for a message object of type 'cmd_control_task-response"
  "f9b5bf540932291b1fe9cba3efc63097")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_control_task-response>)))
  "Returns full string definition for message of type '<cmd_control_task-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_control_task-response)))
  "Returns full string definition for message of type 'cmd_control_task-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_control_task-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_control_task-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_control_task-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cmd_control_task)))
  'cmd_control_task-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cmd_control_task)))
  'cmd_control_task-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_control_task)))
  "Returns string type for a service object of type '<cmd_control_task>"
  "agvs_control/cmd_control_task")