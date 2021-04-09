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

class date_realtime_feedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.theta_y = null;
      this.theta_x = null;
      this.theta_angle = null;
      this.speed_x = null;
      this.speed_y = null;
      this.speed_z = null;
    }
    else {
      if (initObj.hasOwnProperty('theta_y')) {
        this.theta_y = initObj.theta_y
      }
      else {
        this.theta_y = 0.0;
      }
      if (initObj.hasOwnProperty('theta_x')) {
        this.theta_x = initObj.theta_x
      }
      else {
        this.theta_x = 0.0;
      }
      if (initObj.hasOwnProperty('theta_angle')) {
        this.theta_angle = initObj.theta_angle
      }
      else {
        this.theta_angle = 0.0;
      }
      if (initObj.hasOwnProperty('speed_x')) {
        this.speed_x = initObj.speed_x
      }
      else {
        this.speed_x = 0.0;
      }
      if (initObj.hasOwnProperty('speed_y')) {
        this.speed_y = initObj.speed_y
      }
      else {
        this.speed_y = 0.0;
      }
      if (initObj.hasOwnProperty('speed_z')) {
        this.speed_z = initObj.speed_z
      }
      else {
        this.speed_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type date_realtime_feedback
    // Serialize message field [theta_y]
    bufferOffset = _serializer.float32(obj.theta_y, buffer, bufferOffset);
    // Serialize message field [theta_x]
    bufferOffset = _serializer.float32(obj.theta_x, buffer, bufferOffset);
    // Serialize message field [theta_angle]
    bufferOffset = _serializer.float32(obj.theta_angle, buffer, bufferOffset);
    // Serialize message field [speed_x]
    bufferOffset = _serializer.float32(obj.speed_x, buffer, bufferOffset);
    // Serialize message field [speed_y]
    bufferOffset = _serializer.float32(obj.speed_y, buffer, bufferOffset);
    // Serialize message field [speed_z]
    bufferOffset = _serializer.float32(obj.speed_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type date_realtime_feedback
    let len;
    let data = new date_realtime_feedback(null);
    // Deserialize message field [theta_y]
    data.theta_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_x]
    data.theta_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta_angle]
    data.theta_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_x]
    data.speed_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_y]
    data.speed_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [speed_z]
    data.speed_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'agvs_control/date_realtime_feedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c0dced49a8c13be403e74b79b5a5c9de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 theta_y
    float32 theta_x
    float32 theta_angle
    float32 speed_x
    float32 speed_y
    float32 speed_z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new date_realtime_feedback(null);
    if (msg.theta_y !== undefined) {
      resolved.theta_y = msg.theta_y;
    }
    else {
      resolved.theta_y = 0.0
    }

    if (msg.theta_x !== undefined) {
      resolved.theta_x = msg.theta_x;
    }
    else {
      resolved.theta_x = 0.0
    }

    if (msg.theta_angle !== undefined) {
      resolved.theta_angle = msg.theta_angle;
    }
    else {
      resolved.theta_angle = 0.0
    }

    if (msg.speed_x !== undefined) {
      resolved.speed_x = msg.speed_x;
    }
    else {
      resolved.speed_x = 0.0
    }

    if (msg.speed_y !== undefined) {
      resolved.speed_y = msg.speed_y;
    }
    else {
      resolved.speed_y = 0.0
    }

    if (msg.speed_z !== undefined) {
      resolved.speed_z = msg.speed_z;
    }
    else {
      resolved.speed_z = 0.0
    }

    return resolved;
    }
};

module.exports = date_realtime_feedback;
