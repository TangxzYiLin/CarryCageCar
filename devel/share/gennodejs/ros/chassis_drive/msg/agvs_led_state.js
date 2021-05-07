// Auto-generated. Do not edit!

// (in-package chassis_drive.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class agvs_led_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.update_state = null;
    }
    else {
      if (initObj.hasOwnProperty('update_state')) {
        this.update_state = initObj.update_state
      }
      else {
        this.update_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type agvs_led_state
    // Serialize message field [update_state]
    bufferOffset = _serializer.uint8(obj.update_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type agvs_led_state
    let len;
    let data = new agvs_led_state(null);
    // Deserialize message field [update_state]
    data.update_state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chassis_drive/agvs_led_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '82295a76d34cf26c9e6b43410588f16c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 update_state
           
    uint8 chassis_led_state_all_off = 0
    uint8 chassis_led_state_red_on  = 1
    uint8 chassis_led_state_bule_on = 2
    uint8 chassis_led_state_green_on = 3
    uint8 chassis_led_state_red_flash = 4
    uint8 chassis_led_state_green_flash = 5
    uint8 chassis_led_state_blue_flash  = 6
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new agvs_led_state(null);
    if (msg.update_state !== undefined) {
      resolved.update_state = msg.update_state;
    }
    else {
      resolved.update_state = 0
    }

    return resolved;
    }
};

// Constants for message
agvs_led_state.Constants = {
  CHASSIS_LED_STATE_ALL_OFF: 0,
  CHASSIS_LED_STATE_RED_ON: 1,
  CHASSIS_LED_STATE_BULE_ON: 2,
  CHASSIS_LED_STATE_GREEN_ON: 3,
  CHASSIS_LED_STATE_RED_FLASH: 4,
  CHASSIS_LED_STATE_GREEN_FLASH: 5,
  CHASSIS_LED_STATE_BLUE_FLASH: 6,
}

module.exports = agvs_led_state;
