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

class radar_obj {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.f_DistX = null;
      this.f_DistY = null;
      this.f_VrelX = null;
      this.f_VrelY = null;
      this.f_ArelX = null;
      this.f_ArelY = null;
      this.f_DistXStd = null;
      this.f_DistYStd = null;
      this.f_VrelXStd = null;
      this.f_VrelYStd = null;
      this.f_ArelXStd = null;
      this.f_ArelYStd = null;
      this.f_LDeltaX_left = null;
      this.f_LDeltaX_mid = null;
      this.f_LDeltaX_right = null;
      this.f_LDeltaY_left = null;
      this.f_LDeltaY_mid = null;
      this.f_LDeltaY_right = null;
      this.f_RCS = null;
      this.f_ObjectScore = null;
      this.f_VabsX = null;
      this.f_VabsY = null;
      this.f_AabsX = null;
      this.f_AabsY = null;
      this.f_VabsXStd = null;
      this.f_VabsYStd = null;
      this.f_AabsXStd = null;
      this.f_AabsYStd = null;
      this.f_Orientation = null;
      this.f_OrientationStd = null;
      this.f_DistZ = null;
      this.f_DistZStd = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('f_DistX')) {
        this.f_DistX = initObj.f_DistX
      }
      else {
        this.f_DistX = [];
      }
      if (initObj.hasOwnProperty('f_DistY')) {
        this.f_DistY = initObj.f_DistY
      }
      else {
        this.f_DistY = [];
      }
      if (initObj.hasOwnProperty('f_VrelX')) {
        this.f_VrelX = initObj.f_VrelX
      }
      else {
        this.f_VrelX = [];
      }
      if (initObj.hasOwnProperty('f_VrelY')) {
        this.f_VrelY = initObj.f_VrelY
      }
      else {
        this.f_VrelY = [];
      }
      if (initObj.hasOwnProperty('f_ArelX')) {
        this.f_ArelX = initObj.f_ArelX
      }
      else {
        this.f_ArelX = [];
      }
      if (initObj.hasOwnProperty('f_ArelY')) {
        this.f_ArelY = initObj.f_ArelY
      }
      else {
        this.f_ArelY = [];
      }
      if (initObj.hasOwnProperty('f_DistXStd')) {
        this.f_DistXStd = initObj.f_DistXStd
      }
      else {
        this.f_DistXStd = [];
      }
      if (initObj.hasOwnProperty('f_DistYStd')) {
        this.f_DistYStd = initObj.f_DistYStd
      }
      else {
        this.f_DistYStd = [];
      }
      if (initObj.hasOwnProperty('f_VrelXStd')) {
        this.f_VrelXStd = initObj.f_VrelXStd
      }
      else {
        this.f_VrelXStd = [];
      }
      if (initObj.hasOwnProperty('f_VrelYStd')) {
        this.f_VrelYStd = initObj.f_VrelYStd
      }
      else {
        this.f_VrelYStd = [];
      }
      if (initObj.hasOwnProperty('f_ArelXStd')) {
        this.f_ArelXStd = initObj.f_ArelXStd
      }
      else {
        this.f_ArelXStd = [];
      }
      if (initObj.hasOwnProperty('f_ArelYStd')) {
        this.f_ArelYStd = initObj.f_ArelYStd
      }
      else {
        this.f_ArelYStd = [];
      }
      if (initObj.hasOwnProperty('f_LDeltaX_left')) {
        this.f_LDeltaX_left = initObj.f_LDeltaX_left
      }
      else {
        this.f_LDeltaX_left = [];
      }
      if (initObj.hasOwnProperty('f_LDeltaX_mid')) {
        this.f_LDeltaX_mid = initObj.f_LDeltaX_mid
      }
      else {
        this.f_LDeltaX_mid = [];
      }
      if (initObj.hasOwnProperty('f_LDeltaX_right')) {
        this.f_LDeltaX_right = initObj.f_LDeltaX_right
      }
      else {
        this.f_LDeltaX_right = [];
      }
      if (initObj.hasOwnProperty('f_LDeltaY_left')) {
        this.f_LDeltaY_left = initObj.f_LDeltaY_left
      }
      else {
        this.f_LDeltaY_left = [];
      }
      if (initObj.hasOwnProperty('f_LDeltaY_mid')) {
        this.f_LDeltaY_mid = initObj.f_LDeltaY_mid
      }
      else {
        this.f_LDeltaY_mid = [];
      }
      if (initObj.hasOwnProperty('f_LDeltaY_right')) {
        this.f_LDeltaY_right = initObj.f_LDeltaY_right
      }
      else {
        this.f_LDeltaY_right = [];
      }
      if (initObj.hasOwnProperty('f_RCS')) {
        this.f_RCS = initObj.f_RCS
      }
      else {
        this.f_RCS = [];
      }
      if (initObj.hasOwnProperty('f_ObjectScore')) {
        this.f_ObjectScore = initObj.f_ObjectScore
      }
      else {
        this.f_ObjectScore = [];
      }
      if (initObj.hasOwnProperty('f_VabsX')) {
        this.f_VabsX = initObj.f_VabsX
      }
      else {
        this.f_VabsX = [];
      }
      if (initObj.hasOwnProperty('f_VabsY')) {
        this.f_VabsY = initObj.f_VabsY
      }
      else {
        this.f_VabsY = [];
      }
      if (initObj.hasOwnProperty('f_AabsX')) {
        this.f_AabsX = initObj.f_AabsX
      }
      else {
        this.f_AabsX = [];
      }
      if (initObj.hasOwnProperty('f_AabsY')) {
        this.f_AabsY = initObj.f_AabsY
      }
      else {
        this.f_AabsY = [];
      }
      if (initObj.hasOwnProperty('f_VabsXStd')) {
        this.f_VabsXStd = initObj.f_VabsXStd
      }
      else {
        this.f_VabsXStd = [];
      }
      if (initObj.hasOwnProperty('f_VabsYStd')) {
        this.f_VabsYStd = initObj.f_VabsYStd
      }
      else {
        this.f_VabsYStd = [];
      }
      if (initObj.hasOwnProperty('f_AabsXStd')) {
        this.f_AabsXStd = initObj.f_AabsXStd
      }
      else {
        this.f_AabsXStd = [];
      }
      if (initObj.hasOwnProperty('f_AabsYStd')) {
        this.f_AabsYStd = initObj.f_AabsYStd
      }
      else {
        this.f_AabsYStd = [];
      }
      if (initObj.hasOwnProperty('f_Orientation')) {
        this.f_Orientation = initObj.f_Orientation
      }
      else {
        this.f_Orientation = [];
      }
      if (initObj.hasOwnProperty('f_OrientationStd')) {
        this.f_OrientationStd = initObj.f_OrientationStd
      }
      else {
        this.f_OrientationStd = [];
      }
      if (initObj.hasOwnProperty('f_DistZ')) {
        this.f_DistZ = initObj.f_DistZ
      }
      else {
        this.f_DistZ = [];
      }
      if (initObj.hasOwnProperty('f_DistZStd')) {
        this.f_DistZStd = initObj.f_DistZStd
      }
      else {
        this.f_DistZStd = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type radar_obj
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [f_DistX]
    bufferOffset = _arraySerializer.float32(obj.f_DistX, buffer, bufferOffset, null);
    // Serialize message field [f_DistY]
    bufferOffset = _arraySerializer.float32(obj.f_DistY, buffer, bufferOffset, null);
    // Serialize message field [f_VrelX]
    bufferOffset = _arraySerializer.float32(obj.f_VrelX, buffer, bufferOffset, null);
    // Serialize message field [f_VrelY]
    bufferOffset = _arraySerializer.float32(obj.f_VrelY, buffer, bufferOffset, null);
    // Serialize message field [f_ArelX]
    bufferOffset = _arraySerializer.float32(obj.f_ArelX, buffer, bufferOffset, null);
    // Serialize message field [f_ArelY]
    bufferOffset = _arraySerializer.float32(obj.f_ArelY, buffer, bufferOffset, null);
    // Serialize message field [f_DistXStd]
    bufferOffset = _arraySerializer.float32(obj.f_DistXStd, buffer, bufferOffset, null);
    // Serialize message field [f_DistYStd]
    bufferOffset = _arraySerializer.float32(obj.f_DistYStd, buffer, bufferOffset, null);
    // Serialize message field [f_VrelXStd]
    bufferOffset = _arraySerializer.float32(obj.f_VrelXStd, buffer, bufferOffset, null);
    // Serialize message field [f_VrelYStd]
    bufferOffset = _arraySerializer.float32(obj.f_VrelYStd, buffer, bufferOffset, null);
    // Serialize message field [f_ArelXStd]
    bufferOffset = _arraySerializer.float32(obj.f_ArelXStd, buffer, bufferOffset, null);
    // Serialize message field [f_ArelYStd]
    bufferOffset = _arraySerializer.float32(obj.f_ArelYStd, buffer, bufferOffset, null);
    // Serialize message field [f_LDeltaX_left]
    bufferOffset = _arraySerializer.float32(obj.f_LDeltaX_left, buffer, bufferOffset, null);
    // Serialize message field [f_LDeltaX_mid]
    bufferOffset = _arraySerializer.float32(obj.f_LDeltaX_mid, buffer, bufferOffset, null);
    // Serialize message field [f_LDeltaX_right]
    bufferOffset = _arraySerializer.float32(obj.f_LDeltaX_right, buffer, bufferOffset, null);
    // Serialize message field [f_LDeltaY_left]
    bufferOffset = _arraySerializer.float32(obj.f_LDeltaY_left, buffer, bufferOffset, null);
    // Serialize message field [f_LDeltaY_mid]
    bufferOffset = _arraySerializer.float32(obj.f_LDeltaY_mid, buffer, bufferOffset, null);
    // Serialize message field [f_LDeltaY_right]
    bufferOffset = _arraySerializer.float32(obj.f_LDeltaY_right, buffer, bufferOffset, null);
    // Serialize message field [f_RCS]
    bufferOffset = _arraySerializer.float32(obj.f_RCS, buffer, bufferOffset, null);
    // Serialize message field [f_ObjectScore]
    bufferOffset = _arraySerializer.float32(obj.f_ObjectScore, buffer, bufferOffset, null);
    // Serialize message field [f_VabsX]
    bufferOffset = _arraySerializer.float32(obj.f_VabsX, buffer, bufferOffset, null);
    // Serialize message field [f_VabsY]
    bufferOffset = _arraySerializer.float32(obj.f_VabsY, buffer, bufferOffset, null);
    // Serialize message field [f_AabsX]
    bufferOffset = _arraySerializer.float32(obj.f_AabsX, buffer, bufferOffset, null);
    // Serialize message field [f_AabsY]
    bufferOffset = _arraySerializer.float32(obj.f_AabsY, buffer, bufferOffset, null);
    // Serialize message field [f_VabsXStd]
    bufferOffset = _arraySerializer.float32(obj.f_VabsXStd, buffer, bufferOffset, null);
    // Serialize message field [f_VabsYStd]
    bufferOffset = _arraySerializer.float32(obj.f_VabsYStd, buffer, bufferOffset, null);
    // Serialize message field [f_AabsXStd]
    bufferOffset = _arraySerializer.float32(obj.f_AabsXStd, buffer, bufferOffset, null);
    // Serialize message field [f_AabsYStd]
    bufferOffset = _arraySerializer.float32(obj.f_AabsYStd, buffer, bufferOffset, null);
    // Serialize message field [f_Orientation]
    bufferOffset = _arraySerializer.float32(obj.f_Orientation, buffer, bufferOffset, null);
    // Serialize message field [f_OrientationStd]
    bufferOffset = _arraySerializer.float32(obj.f_OrientationStd, buffer, bufferOffset, null);
    // Serialize message field [f_DistZ]
    bufferOffset = _arraySerializer.float32(obj.f_DistZ, buffer, bufferOffset, null);
    // Serialize message field [f_DistZStd]
    bufferOffset = _arraySerializer.float32(obj.f_DistZStd, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type radar_obj
    let len;
    let data = new radar_obj(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [f_DistX]
    data.f_DistX = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_DistY]
    data.f_DistY = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VrelX]
    data.f_VrelX = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VrelY]
    data.f_VrelY = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_ArelX]
    data.f_ArelX = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_ArelY]
    data.f_ArelY = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_DistXStd]
    data.f_DistXStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_DistYStd]
    data.f_DistYStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VrelXStd]
    data.f_VrelXStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VrelYStd]
    data.f_VrelYStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_ArelXStd]
    data.f_ArelXStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_ArelYStd]
    data.f_ArelYStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_LDeltaX_left]
    data.f_LDeltaX_left = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_LDeltaX_mid]
    data.f_LDeltaX_mid = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_LDeltaX_right]
    data.f_LDeltaX_right = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_LDeltaY_left]
    data.f_LDeltaY_left = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_LDeltaY_mid]
    data.f_LDeltaY_mid = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_LDeltaY_right]
    data.f_LDeltaY_right = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_RCS]
    data.f_RCS = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_ObjectScore]
    data.f_ObjectScore = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VabsX]
    data.f_VabsX = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VabsY]
    data.f_VabsY = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_AabsX]
    data.f_AabsX = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_AabsY]
    data.f_AabsY = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VabsXStd]
    data.f_VabsXStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_VabsYStd]
    data.f_VabsYStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_AabsXStd]
    data.f_AabsXStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_AabsYStd]
    data.f_AabsYStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_Orientation]
    data.f_Orientation = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_OrientationStd]
    data.f_OrientationStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_DistZ]
    data.f_DistZ = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [f_DistZStd]
    data.f_DistZStd = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.f_DistX.length;
    length += 4 * object.f_DistY.length;
    length += 4 * object.f_VrelX.length;
    length += 4 * object.f_VrelY.length;
    length += 4 * object.f_ArelX.length;
    length += 4 * object.f_ArelY.length;
    length += 4 * object.f_DistXStd.length;
    length += 4 * object.f_DistYStd.length;
    length += 4 * object.f_VrelXStd.length;
    length += 4 * object.f_VrelYStd.length;
    length += 4 * object.f_ArelXStd.length;
    length += 4 * object.f_ArelYStd.length;
    length += 4 * object.f_LDeltaX_left.length;
    length += 4 * object.f_LDeltaX_mid.length;
    length += 4 * object.f_LDeltaX_right.length;
    length += 4 * object.f_LDeltaY_left.length;
    length += 4 * object.f_LDeltaY_mid.length;
    length += 4 * object.f_LDeltaY_right.length;
    length += 4 * object.f_RCS.length;
    length += 4 * object.f_ObjectScore.length;
    length += 4 * object.f_VabsX.length;
    length += 4 * object.f_VabsY.length;
    length += 4 * object.f_AabsX.length;
    length += 4 * object.f_AabsY.length;
    length += 4 * object.f_VabsXStd.length;
    length += 4 * object.f_VabsYStd.length;
    length += 4 * object.f_AabsXStd.length;
    length += 4 * object.f_AabsYStd.length;
    length += 4 * object.f_Orientation.length;
    length += 4 * object.f_OrientationStd.length;
    length += 4 * object.f_DistZ.length;
    length += 4 * object.f_DistZStd.length;
    return length + 132;
  }

  static datatype() {
    // Returns string type for a message object
    return 'conti_radar/radar_obj';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd139e1dca278064cbcdfddd5e7dfb3bb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 id
    float32[] f_DistX
    float32[] f_DistY
    float32[] f_VrelX
    float32[] f_VrelY
    float32[] f_ArelX
    float32[] f_ArelY
    float32[] f_DistXStd
    float32[] f_DistYStd
    float32[] f_VrelXStd
    float32[] f_VrelYStd
    float32[] f_ArelXStd
    float32[] f_ArelYStd
    float32[] f_LDeltaX_left
    float32[] f_LDeltaX_mid
    float32[] f_LDeltaX_right
    float32[] f_LDeltaY_left
    
    float32[] f_LDeltaY_mid
    float32[] f_LDeltaY_right
    float32[] f_RCS
    float32[] f_ObjectScore
    float32[] f_VabsX
    float32[] f_VabsY
    float32[] f_AabsX
    float32[] f_AabsY
    float32[] f_VabsXStd
    float32[] f_VabsYStd
    float32[] f_AabsXStd
    float32[] f_AabsYStd
    
    float32[] f_Orientation
    float32[] f_OrientationStd
    float32[] f_DistZ
    float32[] f_DistZStd
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new radar_obj(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.f_DistX !== undefined) {
      resolved.f_DistX = msg.f_DistX;
    }
    else {
      resolved.f_DistX = []
    }

    if (msg.f_DistY !== undefined) {
      resolved.f_DistY = msg.f_DistY;
    }
    else {
      resolved.f_DistY = []
    }

    if (msg.f_VrelX !== undefined) {
      resolved.f_VrelX = msg.f_VrelX;
    }
    else {
      resolved.f_VrelX = []
    }

    if (msg.f_VrelY !== undefined) {
      resolved.f_VrelY = msg.f_VrelY;
    }
    else {
      resolved.f_VrelY = []
    }

    if (msg.f_ArelX !== undefined) {
      resolved.f_ArelX = msg.f_ArelX;
    }
    else {
      resolved.f_ArelX = []
    }

    if (msg.f_ArelY !== undefined) {
      resolved.f_ArelY = msg.f_ArelY;
    }
    else {
      resolved.f_ArelY = []
    }

    if (msg.f_DistXStd !== undefined) {
      resolved.f_DistXStd = msg.f_DistXStd;
    }
    else {
      resolved.f_DistXStd = []
    }

    if (msg.f_DistYStd !== undefined) {
      resolved.f_DistYStd = msg.f_DistYStd;
    }
    else {
      resolved.f_DistYStd = []
    }

    if (msg.f_VrelXStd !== undefined) {
      resolved.f_VrelXStd = msg.f_VrelXStd;
    }
    else {
      resolved.f_VrelXStd = []
    }

    if (msg.f_VrelYStd !== undefined) {
      resolved.f_VrelYStd = msg.f_VrelYStd;
    }
    else {
      resolved.f_VrelYStd = []
    }

    if (msg.f_ArelXStd !== undefined) {
      resolved.f_ArelXStd = msg.f_ArelXStd;
    }
    else {
      resolved.f_ArelXStd = []
    }

    if (msg.f_ArelYStd !== undefined) {
      resolved.f_ArelYStd = msg.f_ArelYStd;
    }
    else {
      resolved.f_ArelYStd = []
    }

    if (msg.f_LDeltaX_left !== undefined) {
      resolved.f_LDeltaX_left = msg.f_LDeltaX_left;
    }
    else {
      resolved.f_LDeltaX_left = []
    }

    if (msg.f_LDeltaX_mid !== undefined) {
      resolved.f_LDeltaX_mid = msg.f_LDeltaX_mid;
    }
    else {
      resolved.f_LDeltaX_mid = []
    }

    if (msg.f_LDeltaX_right !== undefined) {
      resolved.f_LDeltaX_right = msg.f_LDeltaX_right;
    }
    else {
      resolved.f_LDeltaX_right = []
    }

    if (msg.f_LDeltaY_left !== undefined) {
      resolved.f_LDeltaY_left = msg.f_LDeltaY_left;
    }
    else {
      resolved.f_LDeltaY_left = []
    }

    if (msg.f_LDeltaY_mid !== undefined) {
      resolved.f_LDeltaY_mid = msg.f_LDeltaY_mid;
    }
    else {
      resolved.f_LDeltaY_mid = []
    }

    if (msg.f_LDeltaY_right !== undefined) {
      resolved.f_LDeltaY_right = msg.f_LDeltaY_right;
    }
    else {
      resolved.f_LDeltaY_right = []
    }

    if (msg.f_RCS !== undefined) {
      resolved.f_RCS = msg.f_RCS;
    }
    else {
      resolved.f_RCS = []
    }

    if (msg.f_ObjectScore !== undefined) {
      resolved.f_ObjectScore = msg.f_ObjectScore;
    }
    else {
      resolved.f_ObjectScore = []
    }

    if (msg.f_VabsX !== undefined) {
      resolved.f_VabsX = msg.f_VabsX;
    }
    else {
      resolved.f_VabsX = []
    }

    if (msg.f_VabsY !== undefined) {
      resolved.f_VabsY = msg.f_VabsY;
    }
    else {
      resolved.f_VabsY = []
    }

    if (msg.f_AabsX !== undefined) {
      resolved.f_AabsX = msg.f_AabsX;
    }
    else {
      resolved.f_AabsX = []
    }

    if (msg.f_AabsY !== undefined) {
      resolved.f_AabsY = msg.f_AabsY;
    }
    else {
      resolved.f_AabsY = []
    }

    if (msg.f_VabsXStd !== undefined) {
      resolved.f_VabsXStd = msg.f_VabsXStd;
    }
    else {
      resolved.f_VabsXStd = []
    }

    if (msg.f_VabsYStd !== undefined) {
      resolved.f_VabsYStd = msg.f_VabsYStd;
    }
    else {
      resolved.f_VabsYStd = []
    }

    if (msg.f_AabsXStd !== undefined) {
      resolved.f_AabsXStd = msg.f_AabsXStd;
    }
    else {
      resolved.f_AabsXStd = []
    }

    if (msg.f_AabsYStd !== undefined) {
      resolved.f_AabsYStd = msg.f_AabsYStd;
    }
    else {
      resolved.f_AabsYStd = []
    }

    if (msg.f_Orientation !== undefined) {
      resolved.f_Orientation = msg.f_Orientation;
    }
    else {
      resolved.f_Orientation = []
    }

    if (msg.f_OrientationStd !== undefined) {
      resolved.f_OrientationStd = msg.f_OrientationStd;
    }
    else {
      resolved.f_OrientationStd = []
    }

    if (msg.f_DistZ !== undefined) {
      resolved.f_DistZ = msg.f_DistZ;
    }
    else {
      resolved.f_DistZ = []
    }

    if (msg.f_DistZStd !== undefined) {
      resolved.f_DistZStd = msg.f_DistZStd;
    }
    else {
      resolved.f_DistZStd = []
    }

    return resolved;
    }
};

module.exports = radar_obj;
