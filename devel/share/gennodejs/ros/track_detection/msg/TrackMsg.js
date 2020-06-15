// Auto-generated. Do not edit!

// (in-package track_detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let PointMsg = require('./PointMsg.js');

//-----------------------------------------------------------

class TrackMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.leftLength = null;
      this.left = null;
      this.rightLength = null;
      this.right = null;
    }
    else {
      if (initObj.hasOwnProperty('leftLength')) {
        this.leftLength = initObj.leftLength
      }
      else {
        this.leftLength = 0;
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = [];
      }
      if (initObj.hasOwnProperty('rightLength')) {
        this.rightLength = initObj.rightLength
      }
      else {
        this.rightLength = 0;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackMsg
    // Serialize message field [leftLength]
    bufferOffset = _serializer.int32(obj.leftLength, buffer, bufferOffset);
    // Serialize message field [left]
    // Serialize the length for message field [left]
    bufferOffset = _serializer.uint32(obj.left.length, buffer, bufferOffset);
    obj.left.forEach((val) => {
      bufferOffset = PointMsg.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [rightLength]
    bufferOffset = _serializer.int32(obj.rightLength, buffer, bufferOffset);
    // Serialize message field [right]
    // Serialize the length for message field [right]
    bufferOffset = _serializer.uint32(obj.right.length, buffer, bufferOffset);
    obj.right.forEach((val) => {
      bufferOffset = PointMsg.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackMsg
    let len;
    let data = new TrackMsg(null);
    // Deserialize message field [leftLength]
    data.leftLength = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [left]
    // Deserialize array length for message field [left]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.left = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.left[i] = PointMsg.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [rightLength]
    data.rightLength = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [right]
    // Deserialize array length for message field [right]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.right = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.right[i] = PointMsg.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 16 * object.left.length;
    length += 16 * object.right.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'track_detection/TrackMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cdb50cb692e6a3e0b32b69782a762116';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # TODO Add track direction
    
    int32 leftLength
    PointMsg[] left
    
    int32 rightLength
    PointMsg[] right
    ================================================================================
    MSG: track_detection/PointMsg
    float64 x
    float64 y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackMsg(null);
    if (msg.leftLength !== undefined) {
      resolved.leftLength = msg.leftLength;
    }
    else {
      resolved.leftLength = 0
    }

    if (msg.left !== undefined) {
      resolved.left = new Array(msg.left.length);
      for (let i = 0; i < resolved.left.length; ++i) {
        resolved.left[i] = PointMsg.Resolve(msg.left[i]);
      }
    }
    else {
      resolved.left = []
    }

    if (msg.rightLength !== undefined) {
      resolved.rightLength = msg.rightLength;
    }
    else {
      resolved.rightLength = 0
    }

    if (msg.right !== undefined) {
      resolved.right = new Array(msg.right.length);
      for (let i = 0; i < resolved.right.length; ++i) {
        resolved.right[i] = PointMsg.Resolve(msg.right[i]);
      }
    }
    else {
      resolved.right = []
    }

    return resolved;
    }
};

module.exports = TrackMsg;
