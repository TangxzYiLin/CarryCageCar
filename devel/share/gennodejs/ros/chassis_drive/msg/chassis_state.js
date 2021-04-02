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

class chassis_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.chassis_mileage_variable_ = null;
      this.chassis_mileage_record_ = null;
      this.chassis_drive_speed_ = null;
      this.chassis_drive_angle_ = null;
      this.chassis_drivemotor_error_code_ = null;
      this.chassis_whirlmotor_erro_code_ = null;
      this.chassis_liftmotor_erro_code_ = null;
      this.chassis_task_id_ = null;
      this.chassis_task_state_ = null;
      this.chassis_selfcheck_error_code_ = null;
    }
    else {
      if (initObj.hasOwnProperty('chassis_mileage_variable_')) {
        this.chassis_mileage_variable_ = initObj.chassis_mileage_variable_
      }
      else {
        this.chassis_mileage_variable_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_mileage_record_')) {
        this.chassis_mileage_record_ = initObj.chassis_mileage_record_
      }
      else {
        this.chassis_mileage_record_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_drive_speed_')) {
        this.chassis_drive_speed_ = initObj.chassis_drive_speed_
      }
      else {
        this.chassis_drive_speed_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_drive_angle_')) {
        this.chassis_drive_angle_ = initObj.chassis_drive_angle_
      }
      else {
        this.chassis_drive_angle_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_drivemotor_error_code_')) {
        this.chassis_drivemotor_error_code_ = initObj.chassis_drivemotor_error_code_
      }
      else {
        this.chassis_drivemotor_error_code_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_whirlmotor_erro_code_')) {
        this.chassis_whirlmotor_erro_code_ = initObj.chassis_whirlmotor_erro_code_
      }
      else {
        this.chassis_whirlmotor_erro_code_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_liftmotor_erro_code_')) {
        this.chassis_liftmotor_erro_code_ = initObj.chassis_liftmotor_erro_code_
      }
      else {
        this.chassis_liftmotor_erro_code_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_task_id_')) {
        this.chassis_task_id_ = initObj.chassis_task_id_
      }
      else {
        this.chassis_task_id_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_task_state_')) {
        this.chassis_task_state_ = initObj.chassis_task_state_
      }
      else {
        this.chassis_task_state_ = 0;
      }
      if (initObj.hasOwnProperty('chassis_selfcheck_error_code_')) {
        this.chassis_selfcheck_error_code_ = initObj.chassis_selfcheck_error_code_
      }
      else {
        this.chassis_selfcheck_error_code_ = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type chassis_state
    // Serialize message field [chassis_mileage_variable_]
    bufferOffset = _serializer.int32(obj.chassis_mileage_variable_, buffer, bufferOffset);
    // Serialize message field [chassis_mileage_record_]
    bufferOffset = _serializer.int32(obj.chassis_mileage_record_, buffer, bufferOffset);
    // Serialize message field [chassis_drive_speed_]
    bufferOffset = _serializer.int16(obj.chassis_drive_speed_, buffer, bufferOffset);
    // Serialize message field [chassis_drive_angle_]
    bufferOffset = _serializer.int16(obj.chassis_drive_angle_, buffer, bufferOffset);
    // Serialize message field [chassis_drivemotor_error_code_]
    bufferOffset = _serializer.int16(obj.chassis_drivemotor_error_code_, buffer, bufferOffset);
    // Serialize message field [chassis_whirlmotor_erro_code_]
    bufferOffset = _serializer.int16(obj.chassis_whirlmotor_erro_code_, buffer, bufferOffset);
    // Serialize message field [chassis_liftmotor_erro_code_]
    bufferOffset = _serializer.int16(obj.chassis_liftmotor_erro_code_, buffer, bufferOffset);
    // Serialize message field [chassis_task_id_]
    bufferOffset = _serializer.int16(obj.chassis_task_id_, buffer, bufferOffset);
    // Serialize message field [chassis_task_state_]
    bufferOffset = _serializer.int16(obj.chassis_task_state_, buffer, bufferOffset);
    // Serialize message field [chassis_selfcheck_error_code_]
    bufferOffset = _serializer.int16(obj.chassis_selfcheck_error_code_, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type chassis_state
    let len;
    let data = new chassis_state(null);
    // Deserialize message field [chassis_mileage_variable_]
    data.chassis_mileage_variable_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [chassis_mileage_record_]
    data.chassis_mileage_record_ = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [chassis_drive_speed_]
    data.chassis_drive_speed_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_drive_angle_]
    data.chassis_drive_angle_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_drivemotor_error_code_]
    data.chassis_drivemotor_error_code_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_whirlmotor_erro_code_]
    data.chassis_whirlmotor_erro_code_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_liftmotor_erro_code_]
    data.chassis_liftmotor_erro_code_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_task_id_]
    data.chassis_task_id_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_task_state_]
    data.chassis_task_state_ = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [chassis_selfcheck_error_code_]
    data.chassis_selfcheck_error_code_ = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'chassis_drive/chassis_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd74b78807fa4e4349088bf6fbedc2966';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 chassis_mileage_variable_
    int32 chassis_mileage_record_
    int16 chassis_drive_speed_
    int16 chassis_drive_angle_
    int16 chassis_drivemotor_error_code_
    int16 chassis_whirlmotor_erro_code_
    int16 chassis_liftmotor_erro_code_
    
    int16 chassis_task_id_
    int16 chassis_task_state_
    int16 chassis_selfcheck_error_code_
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new chassis_state(null);
    if (msg.chassis_mileage_variable_ !== undefined) {
      resolved.chassis_mileage_variable_ = msg.chassis_mileage_variable_;
    }
    else {
      resolved.chassis_mileage_variable_ = 0
    }

    if (msg.chassis_mileage_record_ !== undefined) {
      resolved.chassis_mileage_record_ = msg.chassis_mileage_record_;
    }
    else {
      resolved.chassis_mileage_record_ = 0
    }

    if (msg.chassis_drive_speed_ !== undefined) {
      resolved.chassis_drive_speed_ = msg.chassis_drive_speed_;
    }
    else {
      resolved.chassis_drive_speed_ = 0
    }

    if (msg.chassis_drive_angle_ !== undefined) {
      resolved.chassis_drive_angle_ = msg.chassis_drive_angle_;
    }
    else {
      resolved.chassis_drive_angle_ = 0
    }

    if (msg.chassis_drivemotor_error_code_ !== undefined) {
      resolved.chassis_drivemotor_error_code_ = msg.chassis_drivemotor_error_code_;
    }
    else {
      resolved.chassis_drivemotor_error_code_ = 0
    }

    if (msg.chassis_whirlmotor_erro_code_ !== undefined) {
      resolved.chassis_whirlmotor_erro_code_ = msg.chassis_whirlmotor_erro_code_;
    }
    else {
      resolved.chassis_whirlmotor_erro_code_ = 0
    }

    if (msg.chassis_liftmotor_erro_code_ !== undefined) {
      resolved.chassis_liftmotor_erro_code_ = msg.chassis_liftmotor_erro_code_;
    }
    else {
      resolved.chassis_liftmotor_erro_code_ = 0
    }

    if (msg.chassis_task_id_ !== undefined) {
      resolved.chassis_task_id_ = msg.chassis_task_id_;
    }
    else {
      resolved.chassis_task_id_ = 0
    }

    if (msg.chassis_task_state_ !== undefined) {
      resolved.chassis_task_state_ = msg.chassis_task_state_;
    }
    else {
      resolved.chassis_task_state_ = 0
    }

    if (msg.chassis_selfcheck_error_code_ !== undefined) {
      resolved.chassis_selfcheck_error_code_ = msg.chassis_selfcheck_error_code_;
    }
    else {
      resolved.chassis_selfcheck_error_code_ = 0
    }

    return resolved;
    }
};

module.exports = chassis_state;
