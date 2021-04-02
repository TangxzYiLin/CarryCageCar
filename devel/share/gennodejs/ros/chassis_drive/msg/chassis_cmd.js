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

class chassis_cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.chassis_vel_cmd_ = null;
      this.chassis_angle_cmd_ = null;
      this.chassis_indicator_cmd_ = null;
      this.chassis_brake_cmd_ = null;
    }
    else {
      if (initObj.hasOwnProperty('chassis_vel_cmd_')) {
        this.chassis_vel_cmd_ = initObj.chassis_vel_cmd_
      }
      else {
        this.chassis_vel_cmd_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_angle_cmd_')) {
        this.chassis_angle_cmd_ = initObj.chassis_angle_cmd_
      }
      else {
        this.chassis_angle_cmd_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_indicator_cmd_')) {
        this.chassis_indicator_cmd_ = initObj.chassis_indicator_cmd_
      }
      else {
        this.chassis_indicator_cmd_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_brake_cmd_')) {
        this.chassis_brake_cmd_ = initObj.chassis_brake_cmd_
      }
      else {
        this.chassis_brake_cmd_ = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type chassis_cmd
    // Serialize message field [chassis_vel_cmd_]
    bufferOffset = _serializer.int16(obj.chassis_vel_cmd_, buffer, bufferOffset);
    // Serialize message field [chassis_angle_cmd_]
    bufferOffset = _serializer.int16(obj.chassis_angle_cmd_, buffer, bufferOffset);
    // Serialize message field [chassis_indicator_cmd_]
    bufferOffset = _serializer.int16(obj.chassis_indicator_cmd_, buffer, bufferOffset);
    // Serialize message field [chassis_brake_cmd_]
    bufferOffset = _serializer.int16(obj.chassis_brake_cmd_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type chassis_cmd
    let len;
    let data = new chassis_cmd(null);
    // Deserialize message field [chassis_vel_cmd_]
    data.chassis_vel_cmd_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_angle_cmd_]
    data.chassis_angle_cmd_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_indicator_cmd_]
    data.chassis_indicator_cmd_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_brake_cmd_]
    data.chassis_brake_cmd_ = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chassis_drive/chassis_cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c01b3f6487ca68f987b16e7e5933bc6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 chassis_vel_cmd_  
    int16 chassis_angle_cmd_
    int16 chassis_indicator_cmd_
    int16 chassis_brake_cmd_
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new chassis_cmd(null);
    if (msg.chassis_vel_cmd_ !== undefined) {
      resolved.chassis_vel_cmd_ = msg.chassis_vel_cmd_;
    }
    else {
      resolved.chassis_vel_cmd_ = 0
    }

    if (msg.chassis_angle_cmd_ !== undefined) {
      resolved.chassis_angle_cmd_ = msg.chassis_angle_cmd_;
    }
    else {
      resolved.chassis_angle_cmd_ = 0
    }

    if (msg.chassis_indicator_cmd_ !== undefined) {
      resolved.chassis_indicator_cmd_ = msg.chassis_indicator_cmd_;
    }
    else {
      resolved.chassis_indicator_cmd_ = 0
    }

    if (msg.chassis_brake_cmd_ !== undefined) {
      resolved.chassis_brake_cmd_ = msg.chassis_brake_cmd_;
    }
    else {
      resolved.chassis_brake_cmd_ = 0
    }

    return resolved;
    }
};

module.exports = chassis_cmd;
