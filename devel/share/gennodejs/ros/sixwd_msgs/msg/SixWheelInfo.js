// Auto-generated. Do not edit!

// (in-package sixwd_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SixWheelInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.linearspeed = null;
      this.motor1_speed = null;
      this.motor2_speed = null;
      this.motor3_speed = null;
      this.motor4_speed = null;
      this.motor5_speed = null;
      this.motor6_speed = null;
      this.motor1_current = null;
      this.motor2_current = null;
      this.motor3_current = null;
      this.motor4_current = null;
      this.motor5_current = null;
      this.motor6_current = null;
      this.voltage = null;
      this.temperature = null;
    }
    else {
      if (initObj.hasOwnProperty('linearspeed')) {
        this.linearspeed = initObj.linearspeed
      }
      else {
        this.linearspeed = 0;
      }
      if (initObj.hasOwnProperty('motor1_speed')) {
        this.motor1_speed = initObj.motor1_speed
      }
      else {
        this.motor1_speed = 0;
      }
      if (initObj.hasOwnProperty('motor2_speed')) {
        this.motor2_speed = initObj.motor2_speed
      }
      else {
        this.motor2_speed = 0;
      }
      if (initObj.hasOwnProperty('motor3_speed')) {
        this.motor3_speed = initObj.motor3_speed
      }
      else {
        this.motor3_speed = 0;
      }
      if (initObj.hasOwnProperty('motor4_speed')) {
        this.motor4_speed = initObj.motor4_speed
      }
      else {
        this.motor4_speed = 0;
      }
      if (initObj.hasOwnProperty('motor5_speed')) {
        this.motor5_speed = initObj.motor5_speed
      }
      else {
        this.motor5_speed = 0;
      }
      if (initObj.hasOwnProperty('motor6_speed')) {
        this.motor6_speed = initObj.motor6_speed
      }
      else {
        this.motor6_speed = 0;
      }
      if (initObj.hasOwnProperty('motor1_current')) {
        this.motor1_current = initObj.motor1_current
      }
      else {
        this.motor1_current = 0;
      }
      if (initObj.hasOwnProperty('motor2_current')) {
        this.motor2_current = initObj.motor2_current
      }
      else {
        this.motor2_current = 0;
      }
      if (initObj.hasOwnProperty('motor3_current')) {
        this.motor3_current = initObj.motor3_current
      }
      else {
        this.motor3_current = 0;
      }
      if (initObj.hasOwnProperty('motor4_current')) {
        this.motor4_current = initObj.motor4_current
      }
      else {
        this.motor4_current = 0;
      }
      if (initObj.hasOwnProperty('motor5_current')) {
        this.motor5_current = initObj.motor5_current
      }
      else {
        this.motor5_current = 0;
      }
      if (initObj.hasOwnProperty('motor6_current')) {
        this.motor6_current = initObj.motor6_current
      }
      else {
        this.motor6_current = 0;
      }
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = 0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SixWheelInfo
    // Serialize message field [linearspeed]
    bufferOffset = _serializer.int16(obj.linearspeed, buffer, bufferOffset);
    // Serialize message field [motor1_speed]
    bufferOffset = _serializer.int16(obj.motor1_speed, buffer, bufferOffset);
    // Serialize message field [motor2_speed]
    bufferOffset = _serializer.int16(obj.motor2_speed, buffer, bufferOffset);
    // Serialize message field [motor3_speed]
    bufferOffset = _serializer.int16(obj.motor3_speed, buffer, bufferOffset);
    // Serialize message field [motor4_speed]
    bufferOffset = _serializer.int16(obj.motor4_speed, buffer, bufferOffset);
    // Serialize message field [motor5_speed]
    bufferOffset = _serializer.int16(obj.motor5_speed, buffer, bufferOffset);
    // Serialize message field [motor6_speed]
    bufferOffset = _serializer.int16(obj.motor6_speed, buffer, bufferOffset);
    // Serialize message field [motor1_current]
    bufferOffset = _serializer.int16(obj.motor1_current, buffer, bufferOffset);
    // Serialize message field [motor2_current]
    bufferOffset = _serializer.int16(obj.motor2_current, buffer, bufferOffset);
    // Serialize message field [motor3_current]
    bufferOffset = _serializer.int16(obj.motor3_current, buffer, bufferOffset);
    // Serialize message field [motor4_current]
    bufferOffset = _serializer.int16(obj.motor4_current, buffer, bufferOffset);
    // Serialize message field [motor5_current]
    bufferOffset = _serializer.int16(obj.motor5_current, buffer, bufferOffset);
    // Serialize message field [motor6_current]
    bufferOffset = _serializer.int16(obj.motor6_current, buffer, bufferOffset);
    // Serialize message field [voltage]
    bufferOffset = _serializer.int16(obj.voltage, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.int16(obj.temperature, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SixWheelInfo
    let len;
    let data = new SixWheelInfo(null);
    // Deserialize message field [linearspeed]
    data.linearspeed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor1_speed]
    data.motor1_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor2_speed]
    data.motor2_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor3_speed]
    data.motor3_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor4_speed]
    data.motor4_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor5_speed]
    data.motor5_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor6_speed]
    data.motor6_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor1_current]
    data.motor1_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor2_current]
    data.motor2_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor3_current]
    data.motor3_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor4_current]
    data.motor4_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor5_current]
    data.motor5_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor6_current]
    data.motor6_current = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [voltage]
    data.voltage = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 30;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sixwd_msgs/SixWheelInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4dc9fd6e27eb023af6f7e522d9e61ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int16 linearspeed   #Linear Speed or right or left speed with angle
    int16 motor1_speed #Individual Speed info from each motor
    int16 motor2_speed
    int16 motor3_speed
    int16 motor4_speed
    int16 motor5_speed
    int16 motor6_speed
    int16 motor1_current
    int16 motor2_current
    int16 motor3_current
    int16 motor4_current
    int16 motor5_current
    int16 motor6_current
    int16 voltage  #Battery voltage reading and temperature
    int16 temperature
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SixWheelInfo(null);
    if (msg.linearspeed !== undefined) {
      resolved.linearspeed = msg.linearspeed;
    }
    else {
      resolved.linearspeed = 0
    }

    if (msg.motor1_speed !== undefined) {
      resolved.motor1_speed = msg.motor1_speed;
    }
    else {
      resolved.motor1_speed = 0
    }

    if (msg.motor2_speed !== undefined) {
      resolved.motor2_speed = msg.motor2_speed;
    }
    else {
      resolved.motor2_speed = 0
    }

    if (msg.motor3_speed !== undefined) {
      resolved.motor3_speed = msg.motor3_speed;
    }
    else {
      resolved.motor3_speed = 0
    }

    if (msg.motor4_speed !== undefined) {
      resolved.motor4_speed = msg.motor4_speed;
    }
    else {
      resolved.motor4_speed = 0
    }

    if (msg.motor5_speed !== undefined) {
      resolved.motor5_speed = msg.motor5_speed;
    }
    else {
      resolved.motor5_speed = 0
    }

    if (msg.motor6_speed !== undefined) {
      resolved.motor6_speed = msg.motor6_speed;
    }
    else {
      resolved.motor6_speed = 0
    }

    if (msg.motor1_current !== undefined) {
      resolved.motor1_current = msg.motor1_current;
    }
    else {
      resolved.motor1_current = 0
    }

    if (msg.motor2_current !== undefined) {
      resolved.motor2_current = msg.motor2_current;
    }
    else {
      resolved.motor2_current = 0
    }

    if (msg.motor3_current !== undefined) {
      resolved.motor3_current = msg.motor3_current;
    }
    else {
      resolved.motor3_current = 0
    }

    if (msg.motor4_current !== undefined) {
      resolved.motor4_current = msg.motor4_current;
    }
    else {
      resolved.motor4_current = 0
    }

    if (msg.motor5_current !== undefined) {
      resolved.motor5_current = msg.motor5_current;
    }
    else {
      resolved.motor5_current = 0
    }

    if (msg.motor6_current !== undefined) {
      resolved.motor6_current = msg.motor6_current;
    }
    else {
      resolved.motor6_current = 0
    }

    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = 0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0
    }

    return resolved;
    }
};

module.exports = SixWheelInfo;
