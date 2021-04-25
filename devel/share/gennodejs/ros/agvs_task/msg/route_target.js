// Auto-generated. Do not edit!

// (in-package agvs_task.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class route_target {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_location_x = null;
      this.target_location_y = null;
      this.target_speed = null;
      this.task_direction = null;
      this.task_route_id = null;
    }
    else {
      if (initObj.hasOwnProperty('target_location_x')) {
        this.target_location_x = initObj.target_location_x
      }
      else {
        this.target_location_x = 0.0;
      }
      if (initObj.hasOwnProperty('target_location_y')) {
        this.target_location_y = initObj.target_location_y
      }
      else {
        this.target_location_y = 0.0;
      }
      if (initObj.hasOwnProperty('target_speed')) {
        this.target_speed = initObj.target_speed
      }
      else {
        this.target_speed = 0.0;
      }
      if (initObj.hasOwnProperty('task_direction')) {
        this.task_direction = initObj.task_direction
      }
      else {
        this.task_direction = 0;
      }
      if (initObj.hasOwnProperty('task_route_id')) {
        this.task_route_id = initObj.task_route_id
      }
      else {
        this.task_route_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type route_target
    // Serialize message field [target_location_x]
    bufferOffset = _serializer.float32(obj.target_location_x, buffer, bufferOffset);
    // Serialize message field [target_location_y]
    bufferOffset = _serializer.float32(obj.target_location_y, buffer, bufferOffset);
    // Serialize message field [target_speed]
    bufferOffset = _serializer.float32(obj.target_speed, buffer, bufferOffset);
    // Serialize message field [task_direction]
    bufferOffset = _serializer.int8(obj.task_direction, buffer, bufferOffset);
    // Serialize message field [task_route_id]
    bufferOffset = _serializer.int8(obj.task_route_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type route_target
    let len;
    let data = new route_target(null);
    // Deserialize message field [target_location_x]
    data.target_location_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_location_y]
    data.target_location_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_speed]
    data.target_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [task_direction]
    data.task_direction = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [task_route_id]
    data.task_route_id = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 14;
  }

  static datatype() {
    // Returns string type for a message object
    return 'agvs_task/route_target';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a82df07cc526c047af2d9e2d7bfca86';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 target_location_x
    float32 target_location_y
    float32 target_speed
    int8 task_direction
    int8 task_route_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new route_target(null);
    if (msg.target_location_x !== undefined) {
      resolved.target_location_x = msg.target_location_x;
    }
    else {
      resolved.target_location_x = 0.0
    }

    if (msg.target_location_y !== undefined) {
      resolved.target_location_y = msg.target_location_y;
    }
    else {
      resolved.target_location_y = 0.0
    }

    if (msg.target_speed !== undefined) {
      resolved.target_speed = msg.target_speed;
    }
    else {
      resolved.target_speed = 0.0
    }

    if (msg.task_direction !== undefined) {
      resolved.task_direction = msg.task_direction;
    }
    else {
      resolved.task_direction = 0
    }

    if (msg.task_route_id !== undefined) {
      resolved.task_route_id = msg.task_route_id;
    }
    else {
      resolved.task_route_id = 0
    }

    return resolved;
    }
};

module.exports = route_target;
