// Auto-generated. Do not edit!

// (in-package chassis_drive.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let agvs_mode = require('./agvs_mode.js');
let agvs_task = _finder('agvs_task');

//-----------------------------------------------------------

class agvs_test {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.agvs_mode_1 = null;
      this.agvs_mode_2 = null;
      this.run_target_task = null;
    }
    else {
      if (initObj.hasOwnProperty('agvs_mode_1')) {
        this.agvs_mode_1 = initObj.agvs_mode_1
      }
      else {
        this.agvs_mode_1 = new agvs_mode();
      }
      if (initObj.hasOwnProperty('agvs_mode_2')) {
        this.agvs_mode_2 = initObj.agvs_mode_2
      }
      else {
        this.agvs_mode_2 = new agvs_mode();
      }
      if (initObj.hasOwnProperty('run_target_task')) {
        this.run_target_task = initObj.run_target_task
      }
      else {
        this.run_target_task = new agvs_task.msg.route_target();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type agvs_test
    // Serialize message field [agvs_mode_1]
    bufferOffset = agvs_mode.serialize(obj.agvs_mode_1, buffer, bufferOffset);
    // Serialize message field [agvs_mode_2]
    bufferOffset = agvs_mode.serialize(obj.agvs_mode_2, buffer, bufferOffset);
    // Serialize message field [run_target_task]
    bufferOffset = agvs_task.msg.route_target.serialize(obj.run_target_task, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type agvs_test
    let len;
    let data = new agvs_test(null);
    // Deserialize message field [agvs_mode_1]
    data.agvs_mode_1 = agvs_mode.deserialize(buffer, bufferOffset);
    // Deserialize message field [agvs_mode_2]
    data.agvs_mode_2 = agvs_mode.deserialize(buffer, bufferOffset);
    // Deserialize message field [run_target_task]
    data.run_target_task = agvs_task.msg.route_target.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chassis_drive/agvs_test';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '944f63dcbde35e41019eefa46ce8a035';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    agvs_mode agvs_mode_1
    agvs_mode agvs_mode_2
    
    agvs_task/route_target run_target_task
    
    ================================================================================
    MSG: chassis_drive/agvs_mode
    uint8 current_mode
    
    uint8 agvs_mode_manual =0
    uint8 agvs_mode_auto_floor_1 = 1
    uint8 agvs_mode_auto_floor_2 = 2
    ================================================================================
    MSG: agvs_task/route_target
    float32 target_location_x
    float32 target_location_y
    float32 target_speed
    uint8 task_direction
    uint8 task_route_id
    
    uint8 default_idle = 0
    uint8 positive_direction = 1
    uint8 opposite_direction = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new agvs_test(null);
    if (msg.agvs_mode_1 !== undefined) {
      resolved.agvs_mode_1 = agvs_mode.Resolve(msg.agvs_mode_1)
    }
    else {
      resolved.agvs_mode_1 = new agvs_mode()
    }

    if (msg.agvs_mode_2 !== undefined) {
      resolved.agvs_mode_2 = agvs_mode.Resolve(msg.agvs_mode_2)
    }
    else {
      resolved.agvs_mode_2 = new agvs_mode()
    }

    if (msg.run_target_task !== undefined) {
      resolved.run_target_task = agvs_task.msg.route_target.Resolve(msg.run_target_task)
    }
    else {
      resolved.run_target_task = new agvs_task.msg.route_target()
    }

    return resolved;
    }
};

module.exports = agvs_test;
