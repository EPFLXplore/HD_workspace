// Auto-generated. Do not edit!

// (in-package vision_no_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class vector_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.reliability = null;
      this.x_pos = null;
      this.y_pos = null;
      this.z_pos = null;
      this.x_rot = null;
      this.y_rot = null;
      this.z_rot = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('reliability')) {
        this.reliability = initObj.reliability
      }
      else {
        this.reliability = 0;
      }
      if (initObj.hasOwnProperty('x_pos')) {
        this.x_pos = initObj.x_pos
      }
      else {
        this.x_pos = 0;
      }
      if (initObj.hasOwnProperty('y_pos')) {
        this.y_pos = initObj.y_pos
      }
      else {
        this.y_pos = 0;
      }
      if (initObj.hasOwnProperty('z_pos')) {
        this.z_pos = initObj.z_pos
      }
      else {
        this.z_pos = 0;
      }
      if (initObj.hasOwnProperty('x_rot')) {
        this.x_rot = initObj.x_rot
      }
      else {
        this.x_rot = 0;
      }
      if (initObj.hasOwnProperty('y_rot')) {
        this.y_rot = initObj.y_rot
      }
      else {
        this.y_rot = 0;
      }
      if (initObj.hasOwnProperty('z_rot')) {
        this.z_rot = initObj.z_rot
      }
      else {
        this.z_rot = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type vector_msg
    // Serialize message field [id]
    bufferOffset = _serializer.int16(obj.id, buffer, bufferOffset);
    // Serialize message field [reliability]
    bufferOffset = _serializer.int8(obj.reliability, buffer, bufferOffset);
    // Serialize message field [x_pos]
    bufferOffset = _serializer.int16(obj.x_pos, buffer, bufferOffset);
    // Serialize message field [y_pos]
    bufferOffset = _serializer.int16(obj.y_pos, buffer, bufferOffset);
    // Serialize message field [z_pos]
    bufferOffset = _serializer.int16(obj.z_pos, buffer, bufferOffset);
    // Serialize message field [x_rot]
    bufferOffset = _serializer.int16(obj.x_rot, buffer, bufferOffset);
    // Serialize message field [y_rot]
    bufferOffset = _serializer.int16(obj.y_rot, buffer, bufferOffset);
    // Serialize message field [z_rot]
    bufferOffset = _serializer.int16(obj.z_rot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type vector_msg
    let len;
    let data = new vector_msg(null);
    // Deserialize message field [id]
    data.id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [reliability]
    data.reliability = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [x_pos]
    data.x_pos = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [y_pos]
    data.y_pos = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [z_pos]
    data.z_pos = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [x_rot]
    data.x_rot = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [y_rot]
    data.y_rot = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [z_rot]
    data.z_rot = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'vision_no_ros/vector_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5293413431842928eb9de0ce34594eab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 id
    int8 reliability
    int16 x_pos
    int16 y_pos
    int16 z_pos
    int16 x_rot
    int16 y_rot
    int16 z_rot
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new vector_msg(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.reliability !== undefined) {
      resolved.reliability = msg.reliability;
    }
    else {
      resolved.reliability = 0
    }

    if (msg.x_pos !== undefined) {
      resolved.x_pos = msg.x_pos;
    }
    else {
      resolved.x_pos = 0
    }

    if (msg.y_pos !== undefined) {
      resolved.y_pos = msg.y_pos;
    }
    else {
      resolved.y_pos = 0
    }

    if (msg.z_pos !== undefined) {
      resolved.z_pos = msg.z_pos;
    }
    else {
      resolved.z_pos = 0
    }

    if (msg.x_rot !== undefined) {
      resolved.x_rot = msg.x_rot;
    }
    else {
      resolved.x_rot = 0
    }

    if (msg.y_rot !== undefined) {
      resolved.y_rot = msg.y_rot;
    }
    else {
      resolved.y_rot = 0
    }

    if (msg.z_rot !== undefined) {
      resolved.z_rot = msg.z_rot;
    }
    else {
      resolved.z_rot = 0
    }

    return resolved;
    }
};

module.exports = vector_msg;
