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

class chassis_alarm {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.alarm_forkphotoe_left_ = null;
      this.alarm_forkphotoe_right_ = null;
      this.alarm_cargophotos_left_ = null;
      this.alarm_cargophotos_right_ = null;
      this.alarm_micro_swtich_ = null;
      this.alarm_collision_avoidance_ = null;
      this.alarm_emergency_stop_swtich_ = null;
      this.alarm_auto_man_swtich_ = null;
      this.alarm_up_limit_ = null;
      this.alarm_down_limit_ = null;
    }
    else {
      if (initObj.hasOwnProperty('alarm_forkphotoe_left_')) {
        this.alarm_forkphotoe_left_ = initObj.alarm_forkphotoe_left_
      }
      else {
        this.alarm_forkphotoe_left_ = false;
      }
      if (initObj.hasOwnProperty('alarm_forkphotoe_right_')) {
        this.alarm_forkphotoe_right_ = initObj.alarm_forkphotoe_right_
      }
      else {
        this.alarm_forkphotoe_right_ = false;
      }
      if (initObj.hasOwnProperty('alarm_cargophotos_left_')) {
        this.alarm_cargophotos_left_ = initObj.alarm_cargophotos_left_
      }
      else {
        this.alarm_cargophotos_left_ = false;
      }
      if (initObj.hasOwnProperty('alarm_cargophotos_right_')) {
        this.alarm_cargophotos_right_ = initObj.alarm_cargophotos_right_
      }
      else {
        this.alarm_cargophotos_right_ = false;
      }
      if (initObj.hasOwnProperty('alarm_micro_swtich_')) {
        this.alarm_micro_swtich_ = initObj.alarm_micro_swtich_
      }
      else {
        this.alarm_micro_swtich_ = false;
      }
      if (initObj.hasOwnProperty('alarm_collision_avoidance_')) {
        this.alarm_collision_avoidance_ = initObj.alarm_collision_avoidance_
      }
      else {
        this.alarm_collision_avoidance_ = false;
      }
      if (initObj.hasOwnProperty('alarm_emergency_stop_swtich_')) {
        this.alarm_emergency_stop_swtich_ = initObj.alarm_emergency_stop_swtich_
      }
      else {
        this.alarm_emergency_stop_swtich_ = false;
      }
      if (initObj.hasOwnProperty('alarm_auto_man_swtich_')) {
        this.alarm_auto_man_swtich_ = initObj.alarm_auto_man_swtich_
      }
      else {
        this.alarm_auto_man_swtich_ = false;
      }
      if (initObj.hasOwnProperty('alarm_up_limit_')) {
        this.alarm_up_limit_ = initObj.alarm_up_limit_
      }
      else {
        this.alarm_up_limit_ = false;
      }
      if (initObj.hasOwnProperty('alarm_down_limit_')) {
        this.alarm_down_limit_ = initObj.alarm_down_limit_
      }
      else {
        this.alarm_down_limit_ = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type chassis_alarm
    // Serialize message field [alarm_forkphotoe_left_]
    bufferOffset = _serializer.bool(obj.alarm_forkphotoe_left_, buffer, bufferOffset);
    // Serialize message field [alarm_forkphotoe_right_]
    bufferOffset = _serializer.bool(obj.alarm_forkphotoe_right_, buffer, bufferOffset);
    // Serialize message field [alarm_cargophotos_left_]
    bufferOffset = _serializer.bool(obj.alarm_cargophotos_left_, buffer, bufferOffset);
    // Serialize message field [alarm_cargophotos_right_]
    bufferOffset = _serializer.bool(obj.alarm_cargophotos_right_, buffer, bufferOffset);
    // Serialize message field [alarm_micro_swtich_]
    bufferOffset = _serializer.bool(obj.alarm_micro_swtich_, buffer, bufferOffset);
    // Serialize message field [alarm_collision_avoidance_]
    bufferOffset = _serializer.bool(obj.alarm_collision_avoidance_, buffer, bufferOffset);
    // Serialize message field [alarm_emergency_stop_swtich_]
    bufferOffset = _serializer.bool(obj.alarm_emergency_stop_swtich_, buffer, bufferOffset);
    // Serialize message field [alarm_auto_man_swtich_]
    bufferOffset = _serializer.bool(obj.alarm_auto_man_swtich_, buffer, bufferOffset);
    // Serialize message field [alarm_up_limit_]
    bufferOffset = _serializer.bool(obj.alarm_up_limit_, buffer, bufferOffset);
    // Serialize message field [alarm_down_limit_]
    bufferOffset = _serializer.bool(obj.alarm_down_limit_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type chassis_alarm
    let len;
    let data = new chassis_alarm(null);
    // Deserialize message field [alarm_forkphotoe_left_]
    data.alarm_forkphotoe_left_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_forkphotoe_right_]
    data.alarm_forkphotoe_right_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_cargophotos_left_]
    data.alarm_cargophotos_left_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_cargophotos_right_]
    data.alarm_cargophotos_right_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_micro_swtich_]
    data.alarm_micro_swtich_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_collision_avoidance_]
    data.alarm_collision_avoidance_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_emergency_stop_swtich_]
    data.alarm_emergency_stop_swtich_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_auto_man_swtich_]
    data.alarm_auto_man_swtich_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_up_limit_]
    data.alarm_up_limit_ = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alarm_down_limit_]
    data.alarm_down_limit_ = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chassis_drive/chassis_alarm';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae8c21bba8b1459751e77f410496d9c0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool alarm_forkphotoe_left_
    bool alarm_forkphotoe_right_
    bool alarm_cargophotos_left_
    bool alarm_cargophotos_right_
    bool alarm_micro_swtich_
    bool alarm_collision_avoidance_
    bool alarm_emergency_stop_swtich_
    bool alarm_auto_man_swtich_
    bool alarm_up_limit_
    bool alarm_down_limit_
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new chassis_alarm(null);
    if (msg.alarm_forkphotoe_left_ !== undefined) {
      resolved.alarm_forkphotoe_left_ = msg.alarm_forkphotoe_left_;
    }
    else {
      resolved.alarm_forkphotoe_left_ = false
    }

    if (msg.alarm_forkphotoe_right_ !== undefined) {
      resolved.alarm_forkphotoe_right_ = msg.alarm_forkphotoe_right_;
    }
    else {
      resolved.alarm_forkphotoe_right_ = false
    }

    if (msg.alarm_cargophotos_left_ !== undefined) {
      resolved.alarm_cargophotos_left_ = msg.alarm_cargophotos_left_;
    }
    else {
      resolved.alarm_cargophotos_left_ = false
    }

    if (msg.alarm_cargophotos_right_ !== undefined) {
      resolved.alarm_cargophotos_right_ = msg.alarm_cargophotos_right_;
    }
    else {
      resolved.alarm_cargophotos_right_ = false
    }

    if (msg.alarm_micro_swtich_ !== undefined) {
      resolved.alarm_micro_swtich_ = msg.alarm_micro_swtich_;
    }
    else {
      resolved.alarm_micro_swtich_ = false
    }

    if (msg.alarm_collision_avoidance_ !== undefined) {
      resolved.alarm_collision_avoidance_ = msg.alarm_collision_avoidance_;
    }
    else {
      resolved.alarm_collision_avoidance_ = false
    }

    if (msg.alarm_emergency_stop_swtich_ !== undefined) {
      resolved.alarm_emergency_stop_swtich_ = msg.alarm_emergency_stop_swtich_;
    }
    else {
      resolved.alarm_emergency_stop_swtich_ = false
    }

    if (msg.alarm_auto_man_swtich_ !== undefined) {
      resolved.alarm_auto_man_swtich_ = msg.alarm_auto_man_swtich_;
    }
    else {
      resolved.alarm_auto_man_swtich_ = false
    }

    if (msg.alarm_up_limit_ !== undefined) {
      resolved.alarm_up_limit_ = msg.alarm_up_limit_;
    }
    else {
      resolved.alarm_up_limit_ = false
    }

    if (msg.alarm_down_limit_ !== undefined) {
      resolved.alarm_down_limit_ = msg.alarm_down_limit_;
    }
    else {
      resolved.alarm_down_limit_ = false
    }

    return resolved;
    }
};

module.exports = chassis_alarm;
