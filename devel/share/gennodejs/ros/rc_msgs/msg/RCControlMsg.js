// Auto-generated. Do not edit!

// (in-package rc_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class RCControlMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.throttle_cmd = null;
      this.steering_cmd = null;
    }
    else {
      if (initObj.hasOwnProperty('throttle_cmd')) {
        this.throttle_cmd = initObj.throttle_cmd
      }
      else {
        this.throttle_cmd = 0;
      }
      if (initObj.hasOwnProperty('steering_cmd')) {
        this.steering_cmd = initObj.steering_cmd
      }
      else {
        this.steering_cmd = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RCControlMsg
    // Serialize message field [throttle_cmd]
    bufferOffset = _serializer.int32(obj.throttle_cmd, buffer, bufferOffset);
    // Serialize message field [steering_cmd]
    bufferOffset = _serializer.int32(obj.steering_cmd, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RCControlMsg
    let len;
    let data = new RCControlMsg(null);
    // Deserialize message field [throttle_cmd]
    data.throttle_cmd = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [steering_cmd]
    data.steering_cmd = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rc_msgs/RCControlMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dc4cb0d59f80caaf2e977b6feae39f15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 throttle_cmd
    int32 steering_cmd
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RCControlMsg(null);
    if (msg.throttle_cmd !== undefined) {
      resolved.throttle_cmd = msg.throttle_cmd;
    }
    else {
      resolved.throttle_cmd = 0
    }

    if (msg.steering_cmd !== undefined) {
      resolved.steering_cmd = msg.steering_cmd;
    }
    else {
      resolved.steering_cmd = 0
    }

    return resolved;
    }
};

module.exports = RCControlMsg;
