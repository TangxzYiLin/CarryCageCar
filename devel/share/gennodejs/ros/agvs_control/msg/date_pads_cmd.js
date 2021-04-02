// Auto-generated. Do not edit!

// (in-package agvs_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class date_pads_cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle_date = null;
      this.speed_date = null;
    }
    else {
      if (initObj.hasOwnProperty('angle_date')) {
        this.angle_date = initObj.angle_date
      }
      else {
        this.angle_date = 0.0;
      }
      if (initObj.hasOwnProperty('speed_date')) {
        this.speed_date = initObj.speed_date
      }
      else {
        this.speed_date = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type date_pads_cmd
    // Serialize message field [angle_date]
    bufferOffset = _serializer.float32(obj.angle_date, buffer, bufferOffset);
    // Serialize message field [speed_date]
    bufferOffset = _serializer.float32(obj.speed_date, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type date_pads_cmd
    let len;
    let data = new date_pads_cmd(null);
    // Deserialize message field [angle_date]
    data.angle_date = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_date]
    data.speed_date = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'agvs_control/date_pads_cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '400b13fd7cc1c6f886c0fcc01fcab9aa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 angle_date
    float32 speed_date
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new date_pads_cmd(null);
    if (msg.angle_date !== undefined) {
      resolved.angle_date = msg.angle_date;
    }
    else {
      resolved.angle_date = 0.0
    }

    if (msg.speed_date !== undefined) {
      resolved.speed_date = msg.speed_date;
    }
    else {
      resolved.speed_date = 0.0
    }

    return resolved;
    }
};

module.exports = date_pads_cmd;
