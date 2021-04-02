// Auto-generated. Do not edit!

// (in-package agvs_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class cmd_control_taskRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode_run = null;
    }
    else {
      if (initObj.hasOwnProperty('mode_run')) {
        this.mode_run = initObj.mode_run
      }
      else {
        this.mode_run = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_control_taskRequest
    // Serialize message field [mode_run]
    bufferOffset = _serializer.uint8(obj.mode_run, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_control_taskRequest
    let len;
    let data = new cmd_control_taskRequest(null);
    // Deserialize message field [mode_run]
    data.mode_run = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agvs_control/cmd_control_taskRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9b5bf540932291b1fe9cba3efc63097';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 mode_run 
    uint8 manual_mode=1
    uint8 auto_mode=2
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_control_taskRequest(null);
    if (msg.mode_run !== undefined) {
      resolved.mode_run = msg.mode_run;
    }
    else {
      resolved.mode_run = 0
    }

    return resolved;
    }
};

// Constants for message
cmd_control_taskRequest.Constants = {
  MANUAL_MODE: 1,
  AUTO_MODE: 2,
}

class cmd_control_taskResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_control_taskResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_control_taskResponse
    let len;
    let data = new cmd_control_taskResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agvs_control/cmd_control_taskResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_control_taskResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: cmd_control_taskRequest,
  Response: cmd_control_taskResponse,
  md5sum() { return 'f9b5bf540932291b1fe9cba3efc63097'; },
  datatype() { return 'agvs_control/cmd_control_task'; }
};
