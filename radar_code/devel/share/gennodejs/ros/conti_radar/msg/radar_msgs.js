// Auto-generated. Do not edit!

// (in-package conti_radar.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class radar_msgs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_ID = null;
      this.x = null;
      this.y = null;
      this.velx = null;
      this.vely = null;
    }
    else {
      if (initObj.hasOwnProperty('object_ID')) {
        this.object_ID = initObj.object_ID
      }
      else {
        this.object_ID = [];
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
      if (initObj.hasOwnProperty('velx')) {
        this.velx = initObj.velx
      }
      else {
        this.velx = [];
      }
      if (initObj.hasOwnProperty('vely')) {
        this.vely = initObj.vely
      }
      else {
        this.vely = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type radar_msgs
    // Serialize message field [object_ID]
    bufferOffset = _arraySerializer.int32(obj.object_ID, buffer, bufferOffset, null);
    // Serialize message field [x]
    bufferOffset = _arraySerializer.float32(obj.x, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.float32(obj.y, buffer, bufferOffset, null);
    // Serialize message field [velx]
    bufferOffset = _arraySerializer.float32(obj.velx, buffer, bufferOffset, null);
    // Serialize message field [vely]
    bufferOffset = _arraySerializer.float32(obj.vely, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type radar_msgs
    let len;
    let data = new radar_msgs(null);
    // Deserialize message field [object_ID]
    data.object_ID = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [x]
    data.x = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [velx]
    data.velx = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [vely]
    data.vely = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.object_ID.length;
    length += 4 * object.x.length;
    length += 4 * object.y.length;
    length += 4 * object.velx.length;
    length += 4 * object.vely.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'conti_radar/radar_msgs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b031195c2ce10d5f86075f318ebd08cc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] object_ID
    float32[] x
    float32[] y
    float32[] velx
    float32[] vely
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new radar_msgs(null);
    if (msg.object_ID !== undefined) {
      resolved.object_ID = msg.object_ID;
    }
    else {
      resolved.object_ID = []
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    if (msg.velx !== undefined) {
      resolved.velx = msg.velx;
    }
    else {
      resolved.velx = []
    }

    if (msg.vely !== undefined) {
      resolved.vely = msg.vely;
    }
    else {
      resolved.vely = []
    }

    return resolved;
    }
};

module.exports = radar_msgs;
