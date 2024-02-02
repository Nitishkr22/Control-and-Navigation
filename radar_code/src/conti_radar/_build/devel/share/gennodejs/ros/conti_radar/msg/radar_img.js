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

class radar_img {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.range = null;
      this.azimuth = null;
      this.VrelRad = null;
      this.RCS = null;
      this.rangevar = null;
      this.vrelradvar = null;
      this.azimuthvar = null;
      this.SNR = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = [];
      }
      if (initObj.hasOwnProperty('azimuth')) {
        this.azimuth = initObj.azimuth
      }
      else {
        this.azimuth = [];
      }
      if (initObj.hasOwnProperty('VrelRad')) {
        this.VrelRad = initObj.VrelRad
      }
      else {
        this.VrelRad = [];
      }
      if (initObj.hasOwnProperty('RCS')) {
        this.RCS = initObj.RCS
      }
      else {
        this.RCS = [];
      }
      if (initObj.hasOwnProperty('rangevar')) {
        this.rangevar = initObj.rangevar
      }
      else {
        this.rangevar = [];
      }
      if (initObj.hasOwnProperty('vrelradvar')) {
        this.vrelradvar = initObj.vrelradvar
      }
      else {
        this.vrelradvar = [];
      }
      if (initObj.hasOwnProperty('azimuthvar')) {
        this.azimuthvar = initObj.azimuthvar
      }
      else {
        this.azimuthvar = [];
      }
      if (initObj.hasOwnProperty('SNR')) {
        this.SNR = initObj.SNR
      }
      else {
        this.SNR = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type radar_img
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _arraySerializer.float32(obj.range, buffer, bufferOffset, null);
    // Serialize message field [azimuth]
    bufferOffset = _arraySerializer.float32(obj.azimuth, buffer, bufferOffset, null);
    // Serialize message field [VrelRad]
    bufferOffset = _arraySerializer.float32(obj.VrelRad, buffer, bufferOffset, null);
    // Serialize message field [RCS]
    bufferOffset = _arraySerializer.float32(obj.RCS, buffer, bufferOffset, null);
    // Serialize message field [rangevar]
    bufferOffset = _arraySerializer.float32(obj.rangevar, buffer, bufferOffset, null);
    // Serialize message field [vrelradvar]
    bufferOffset = _arraySerializer.float32(obj.vrelradvar, buffer, bufferOffset, null);
    // Serialize message field [azimuthvar]
    bufferOffset = _arraySerializer.float32(obj.azimuthvar, buffer, bufferOffset, null);
    // Serialize message field [SNR]
    bufferOffset = _arraySerializer.float32(obj.SNR, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type radar_img
    let len;
    let data = new radar_img(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [azimuth]
    data.azimuth = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [VrelRad]
    data.VrelRad = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [RCS]
    data.RCS = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [rangevar]
    data.rangevar = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [vrelradvar]
    data.vrelradvar = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [azimuthvar]
    data.azimuthvar = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [SNR]
    data.SNR = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.range.length;
    length += 4 * object.azimuth.length;
    length += 4 * object.VrelRad.length;
    length += 4 * object.RCS.length;
    length += 4 * object.rangevar.length;
    length += 4 * object.vrelradvar.length;
    length += 4 * object.azimuthvar.length;
    length += 4 * object.SNR.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'conti_radar/radar_img';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4bee43583744ef6616006334aa5e5b2e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id
    float32[] range
    float32[] azimuth
    float32[] VrelRad
    float32[] RCS
    float32[] rangevar
    float32[] vrelradvar
    float32[] azimuthvar
    float32[] SNR
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new radar_img(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = []
    }

    if (msg.azimuth !== undefined) {
      resolved.azimuth = msg.azimuth;
    }
    else {
      resolved.azimuth = []
    }

    if (msg.VrelRad !== undefined) {
      resolved.VrelRad = msg.VrelRad;
    }
    else {
      resolved.VrelRad = []
    }

    if (msg.RCS !== undefined) {
      resolved.RCS = msg.RCS;
    }
    else {
      resolved.RCS = []
    }

    if (msg.rangevar !== undefined) {
      resolved.rangevar = msg.rangevar;
    }
    else {
      resolved.rangevar = []
    }

    if (msg.vrelradvar !== undefined) {
      resolved.vrelradvar = msg.vrelradvar;
    }
    else {
      resolved.vrelradvar = []
    }

    if (msg.azimuthvar !== undefined) {
      resolved.azimuthvar = msg.azimuthvar;
    }
    else {
      resolved.azimuthvar = []
    }

    if (msg.SNR !== undefined) {
      resolved.SNR = msg.SNR;
    }
    else {
      resolved.SNR = []
    }

    return resolved;
    }
};

module.exports = radar_img;
