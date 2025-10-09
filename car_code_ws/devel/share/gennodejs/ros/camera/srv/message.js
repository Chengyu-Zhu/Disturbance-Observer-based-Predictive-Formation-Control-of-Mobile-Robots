// Auto-generated. Do not edit!

// (in-package camera.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class messageRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.if_request = null;
    }
    else {
      if (initObj.hasOwnProperty('if_request')) {
        this.if_request = initObj.if_request
      }
      else {
        this.if_request = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type messageRequest
    // Serialize message field [if_request]
    bufferOffset = _serializer.int32(obj.if_request, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type messageRequest
    let len;
    let data = new messageRequest(null);
    // Deserialize message field [if_request]
    data.if_request = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera/messageRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf853f9571ad9688abfb5c1e3565de2c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 客户端请求时发送的两个数字
    int32 if_request
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new messageRequest(null);
    if (msg.if_request !== undefined) {
      resolved.if_request = msg.if_request;
    }
    else {
      resolved.if_request = 0
    }

    return resolved;
    }
};

class messageResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_p = null;
      this.y_q = null;
      this.depth = null;
    }
    else {
      if (initObj.hasOwnProperty('x_p')) {
        this.x_p = initObj.x_p
      }
      else {
        this.x_p = 0.0;
      }
      if (initObj.hasOwnProperty('y_q')) {
        this.y_q = initObj.y_q
      }
      else {
        this.y_q = 0.0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type messageResponse
    // Serialize message field [x_p]
    bufferOffset = _serializer.float64(obj.x_p, buffer, bufferOffset);
    // Serialize message field [y_q]
    bufferOffset = _serializer.float64(obj.y_q, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float64(obj.depth, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type messageResponse
    let len;
    let data = new messageResponse(null);
    // Deserialize message field [x_p]
    data.x_p = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_q]
    data.y_q = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'camera/messageResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a9efcb4d91788a37410c13642a952590';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 服务器响应发送的数据
    float64 x_p
    float64 y_q
    float64 depth
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new messageResponse(null);
    if (msg.x_p !== undefined) {
      resolved.x_p = msg.x_p;
    }
    else {
      resolved.x_p = 0.0
    }

    if (msg.y_q !== undefined) {
      resolved.y_q = msg.y_q;
    }
    else {
      resolved.y_q = 0.0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: messageRequest,
  Response: messageResponse,
  md5sum() { return 'b9d1c9fee0054923e6d8acbb284d7b0b'; },
  datatype() { return 'camera/message'; }
};
