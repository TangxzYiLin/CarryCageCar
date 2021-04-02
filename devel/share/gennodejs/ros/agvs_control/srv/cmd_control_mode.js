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

class cmd_control_modeRequest {
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
        this.mode_run = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_control_modeRequest
    // Serialize message field [mode_run]
    bufferOffset = _serializer.string(obj.mode_run, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_control_modeRequest
    let len;
    let data = new cmd_control_modeRequest(null);
    // Deserialize message field [mode_run]
    data.mode_run = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mode_run.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agvs_control/cmd_control_modeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0545bf3db61acfe8473d040eb76fcb1c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string mode_run 
    string manual_mode="manual_mode"
    string auto_mode="auto_mode"
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_control_modeRequest(null);
    if (msg.mode_run !== undefined) {
      resolved.mode_run = msg.mode_run;
    }
    else {
      resolved.mode_run = ''
    }

    return resolved;
    }
};

// Constants for message
cmd_control_modeRequest.Constants = {
  MANUAL_MODE: '"manual_mode"',
  AUTO_MODE: '"auto_mode"',
}

class cmd_control_modeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_control_modeResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_control_modeResponse
    let len;
    let data = new cmd_control_modeResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'agvs_control/cmd_control_modeResponse';
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
    const resolved = new cmd_control_modeResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: cmd_control_modeRequest,
  Response: cmd_control_modeResponse,
  md5sum() { return '0545bf3db61acfe8473d040eb76fcb1c'; },
  datatype() { return 'agvs_control/cmd_control_mode'; }
};
