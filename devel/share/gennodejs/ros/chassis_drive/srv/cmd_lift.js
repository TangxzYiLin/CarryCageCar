// Auto-generated. Do not edit!

// (in-package chassis_drive.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class cmd_liftRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lift_active = null;
    }
    else {
      if (initObj.hasOwnProperty('lift_active')) {
        this.lift_active = initObj.lift_active
      }
      else {
        this.lift_active = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_liftRequest
    // Serialize message field [lift_active]
    bufferOffset = _serializer.uint16(obj.lift_active, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_liftRequest
    let len;
    let data = new cmd_liftRequest(null);
    // Deserialize message field [lift_active]
    data.lift_active = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'chassis_drive/cmd_liftRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ebadb64f1f7323f82c99c510cfe6f5c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 lift_active
    uint16 mode_up=1
    uint16 mode_down=0
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_liftRequest(null);
    if (msg.lift_active !== undefined) {
      resolved.lift_active = msg.lift_active;
    }
    else {
      resolved.lift_active = 0
    }

    return resolved;
    }
};

// Constants for message
cmd_liftRequest.Constants = {
  MODE_UP: 1,
  MODE_DOWN: 0,
}

class cmd_liftResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_liftResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_liftResponse
    let len;
    let data = new cmd_liftResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'chassis_drive/cmd_liftResponse';
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
    const resolved = new cmd_liftResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: cmd_liftRequest,
  Response: cmd_liftResponse,
  md5sum() { return '3ebadb64f1f7323f82c99c510cfe6f5c'; },
  datatype() { return 'chassis_drive/cmd_lift'; }
};
