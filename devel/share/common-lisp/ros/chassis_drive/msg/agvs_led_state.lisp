; Auto-generated. Do not edit!


(cl:in-package chassis_drive-msg)


;//! \htmlinclude agvs_led_state.msg.html

(cl:defclass <agvs_led_state> (roslisp-msg-protocol:ros-message)
  ((update_state
    :reader update_state
    :initarg :update_state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass agvs_led_state (<agvs_led_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <agvs_led_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'agvs_led_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name chassis_drive-msg:<agvs_led_state> is deprecated: use chassis_drive-msg:agvs_led_state instead.")))

(cl:ensure-generic-function 'update_state-val :lambda-list '(m))
(cl:defmethod update_state-val ((m <agvs_led_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader chassis_drive-msg:update_state-val is deprecated.  Use chassis_drive-msg:update_state instead.")
  (update_state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<agvs_led_state>)))
    "Constants for message type '<agvs_led_state>"
  '((:CHASSIS_LED_STATE_ALL_OFF . 0)
    (:CHASSIS_LED_STATE_RED_ON . 1)
    (:CHASSIS_LED_STATE_BULE_ON . 2)
    (:CHASSIS_LED_STATE_GREEN_ON . 3)
    (:CHASSIS_LED_STATE_RED_FLASH . 4)
    (:CHASSIS_LED_STATE_GREEN_FLASH . 5)
    (:CHASSIS_LED_STATE_BLUE_FLASH . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'agvs_led_state)))
    "Constants for message type 'agvs_led_state"
  '((:CHASSIS_LED_STATE_ALL_OFF . 0)
    (:CHASSIS_LED_STATE_RED_ON . 1)
    (:CHASSIS_LED_STATE_BULE_ON . 2)
    (:CHASSIS_LED_STATE_GREEN_ON . 3)
    (:CHASSIS_LED_STATE_RED_FLASH . 4)
    (:CHASSIS_LED_STATE_GREEN_FLASH . 5)
    (:CHASSIS_LED_STATE_BLUE_FLASH . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <agvs_led_state>) ostream)
  "Serializes a message object of type '<agvs_led_state>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'update_state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <agvs_led_state>) istream)
  "Deserializes a message object of type '<agvs_led_state>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'update_state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<agvs_led_state>)))
  "Returns string type for a message object of type '<agvs_led_state>"
  "chassis_drive/agvs_led_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'agvs_led_state)))
  "Returns string type for a message object of type 'agvs_led_state"
  "chassis_drive/agvs_led_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<agvs_led_state>)))
  "Returns md5sum for a message object of type '<agvs_led_state>"
  "82295a76d34cf26c9e6b43410588f16c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'agvs_led_state)))
  "Returns md5sum for a message object of type 'agvs_led_state"
  "82295a76d34cf26c9e6b43410588f16c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<agvs_led_state>)))
  "Returns full string definition for message of type '<agvs_led_state>"
  (cl:format cl:nil "uint8 update_state~%       ~%uint8 chassis_led_state_all_off = 0~%uint8 chassis_led_state_red_on  = 1~%uint8 chassis_led_state_bule_on = 2~%uint8 chassis_led_state_green_on = 3~%uint8 chassis_led_state_red_flash = 4~%uint8 chassis_led_state_green_flash = 5~%uint8 chassis_led_state_blue_flash  = 6~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'agvs_led_state)))
  "Returns full string definition for message of type 'agvs_led_state"
  (cl:format cl:nil "uint8 update_state~%       ~%uint8 chassis_led_state_all_off = 0~%uint8 chassis_led_state_red_on  = 1~%uint8 chassis_led_state_bule_on = 2~%uint8 chassis_led_state_green_on = 3~%uint8 chassis_led_state_red_flash = 4~%uint8 chassis_led_state_green_flash = 5~%uint8 chassis_led_state_blue_flash  = 6~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <agvs_led_state>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <agvs_led_state>))
  "Converts a ROS message object to a list"
  (cl:list 'agvs_led_state
    (cl:cons ':update_state (update_state msg))
))
