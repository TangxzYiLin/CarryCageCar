; Auto-generated. Do not edit!


(cl:in-package agvs_pad-srv)


;//! \htmlinclude cmd_lift-request.msg.html

(cl:defclass <cmd_lift-request> (roslisp-msg-protocol:ros-message)
  ((lift_active
    :reader lift_active
    :initarg :lift_active
    :type cl:fixnum
    :initform 0))
)

(cl:defclass cmd_lift-request (<cmd_lift-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_lift-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_lift-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_pad-srv:<cmd_lift-request> is deprecated: use agvs_pad-srv:cmd_lift-request instead.")))

(cl:ensure-generic-function 'lift_active-val :lambda-list '(m))
(cl:defmethod lift_active-val ((m <cmd_lift-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agvs_pad-srv:lift_active-val is deprecated.  Use agvs_pad-srv:lift_active instead.")
  (lift_active m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<cmd_lift-request>)))
    "Constants for message type '<cmd_lift-request>"
  '((:MODE_UP . 1)
    (:MODE_DOWN . 0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'cmd_lift-request)))
    "Constants for message type 'cmd_lift-request"
  '((:MODE_UP . 1)
    (:MODE_DOWN . 0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_lift-request>) ostream)
  "Serializes a message object of type '<cmd_lift-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lift_active)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lift_active)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_lift-request>) istream)
  "Deserializes a message object of type '<cmd_lift-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lift_active)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lift_active)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_lift-request>)))
  "Returns string type for a service object of type '<cmd_lift-request>"
  "agvs_pad/cmd_liftRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_lift-request)))
  "Returns string type for a service object of type 'cmd_lift-request"
  "agvs_pad/cmd_liftRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_lift-request>)))
  "Returns md5sum for a message object of type '<cmd_lift-request>"
  "3ebadb64f1f7323f82c99c510cfe6f5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_lift-request)))
  "Returns md5sum for a message object of type 'cmd_lift-request"
  "3ebadb64f1f7323f82c99c510cfe6f5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_lift-request>)))
  "Returns full string definition for message of type '<cmd_lift-request>"
  (cl:format cl:nil "uint16 lift_active~%uint16 mode_up=1~%uint16 mode_down=0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_lift-request)))
  "Returns full string definition for message of type 'cmd_lift-request"
  (cl:format cl:nil "uint16 lift_active~%uint16 mode_up=1~%uint16 mode_down=0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_lift-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_lift-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_lift-request
    (cl:cons ':lift_active (lift_active msg))
))
;//! \htmlinclude cmd_lift-response.msg.html

(cl:defclass <cmd_lift-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass cmd_lift-response (<cmd_lift-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cmd_lift-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cmd_lift-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agvs_pad-srv:<cmd_lift-response> is deprecated: use agvs_pad-srv:cmd_lift-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cmd_lift-response>) ostream)
  "Serializes a message object of type '<cmd_lift-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cmd_lift-response>) istream)
  "Deserializes a message object of type '<cmd_lift-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cmd_lift-response>)))
  "Returns string type for a service object of type '<cmd_lift-response>"
  "agvs_pad/cmd_liftResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_lift-response)))
  "Returns string type for a service object of type 'cmd_lift-response"
  "agvs_pad/cmd_liftResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cmd_lift-response>)))
  "Returns md5sum for a message object of type '<cmd_lift-response>"
  "3ebadb64f1f7323f82c99c510cfe6f5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cmd_lift-response)))
  "Returns md5sum for a message object of type 'cmd_lift-response"
  "3ebadb64f1f7323f82c99c510cfe6f5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cmd_lift-response>)))
  "Returns full string definition for message of type '<cmd_lift-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cmd_lift-response)))
  "Returns full string definition for message of type 'cmd_lift-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cmd_lift-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cmd_lift-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cmd_lift-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cmd_lift)))
  'cmd_lift-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cmd_lift)))
  'cmd_lift-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cmd_lift)))
  "Returns string type for a service object of type '<cmd_lift>"
  "agvs_pad/cmd_lift")