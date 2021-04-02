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

class chassis_bat {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bat_cap_ = null;
      this.bat_vol_ = null;
      this.bat_cur_ = null;
      this.bat_error_code_ = null;
    }
    else {
      if (initObj.hasOwnProperty('bat_cap_')) {
        this.bat_cap_ = initObj.bat_cap_
      }
      else {
        this.bat_cap_ = 0;
      }
      if (initObj.hasOwnProperty('bat_vol_')) {
        this.bat_vol_ = initObj.bat_vol_
      }
      else {
        this.bat_vol_ = 0;
      }
      if (initObj.hasOwnProperty('bat_cur_')) {
        this.bat_cur_ = initObj.bat_cur_
      }
      else {
        this.bat_cur_ = 0;
      }
      if (initObj.hasOwnProperty('bat_error_code_')) {
        this.bat_error_code_ = initObj.bat_error_code_
      }
      else {
        this.bat_error_code_ = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type chassis_bat
    // Serialize message field [bat_cap_]
    bufferOffset = _serializer.int16(obj.bat_cap_, buffer, bufferOffset);
    // Serialize message field [bat_vol_]
    bufferOffset = _serializer.int16(obj.bat_vol_, buffer, bufferOffset);
    // Serialize message field [bat_cur_]
    bufferOffset = _serializer.int16(obj.bat_cur_, buffer, bufferOffset);
    // Serialize message field [bat_error_code_]
    bufferOffset = _serializer.int16(obj.bat_error_code_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type chassis_bat
    let len;
    let data = new chassis_bat(null);
    // Deserialize message field [bat_cap_]
    data.bat_cap_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [bat_vol_]
    data.bat_vol_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [bat_cur_]
    data.bat_cur_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [bat_error_code_]
    data.bat_error_code_ = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chassis_drive/chassis_bat';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d7bde1a0bdfdf9227bac0927d779d8f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 bat_cap_
    int16 bat_vol_
    int16 bat_cur_
    int16 bat_error_code_
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new chassis_bat(null);
    if (msg.bat_cap_ !== undefined) {
      resolved.bat_cap_ = msg.bat_cap_;
    }
    else {
      resolved.bat_cap_ = 0
    }

    if (msg.bat_vol_ !== undefined) {
      resolved.bat_vol_ = msg.bat_vol_;
    }
    else {
      resolved.bat_vol_ = 0
    }

    if (msg.bat_cur_ !== undefined) {
      resolved.bat_cur_ = msg.bat_cur_;
    }
    else {
      resolved.bat_cur_ = 0
    }

    if (msg.bat_error_code_ !== undefined) {
      resolved.bat_error_code_ = msg.bat_error_code_;
    }
    else {
      resolved.bat_error_code_ = 0
    }

    return resolved;
    }
};

module.exports = chassis_bat;
