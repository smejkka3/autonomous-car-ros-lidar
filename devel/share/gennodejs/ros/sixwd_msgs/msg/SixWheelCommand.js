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

class SixWheelCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controltype = null;
      this.linearspeed = null;
      this.angle = null;
      this.motor_number = null;
      this.individual_motors_speed = null;
      this.right_speed = null;
      this.left_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('controltype')) {
        this.controltype = initObj.controltype
      }
      else {
        this.controltype = 0;
      }
      if (initObj.hasOwnProperty('linearspeed')) {
        this.linearspeed = initObj.linearspeed
      }
      else {
        this.linearspeed = 0;
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0;
      }
      if (initObj.hasOwnProperty('motor_number')) {
        this.motor_number = initObj.motor_number
      }
      else {
        this.motor_number = 0;
      }
      if (initObj.hasOwnProperty('individual_motors_speed')) {
        this.individual_motors_speed = initObj.individual_motors_speed
      }
      else {
        this.individual_motors_speed = 0;
      }
      if (initObj.hasOwnProperty('right_speed')) {
        this.right_speed = initObj.right_speed
      }
      else {
        this.right_speed = 0;
      }
      if (initObj.hasOwnProperty('left_speed')) {
        this.left_speed = initObj.left_speed
      }
      else {
        this.left_speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SixWheelCommand
    // Serialize message field [controltype]
    bufferOffset = _serializer.int16(obj.controltype, buffer, bufferOffset);
    // Serialize message field [linearspeed]
    bufferOffset = _serializer.int16(obj.linearspeed, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.int16(obj.angle, buffer, bufferOffset);
    // Serialize message field [motor_number]
    bufferOffset = _serializer.uint8(obj.motor_number, buffer, bufferOffset);
    // Serialize message field [individual_motors_speed]
    bufferOffset = _serializer.int16(obj.individual_motors_speed, buffer, bufferOffset);
    // Serialize message field [right_speed]
    bufferOffset = _serializer.int16(obj.right_speed, buffer, bufferOffset);
    // Serialize message field [left_speed]
    bufferOffset = _serializer.int16(obj.left_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SixWheelCommand
    let len;
    let data = new SixWheelCommand(null);
    // Deserialize message field [controltype]
    data.controltype = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [linearspeed]
    data.linearspeed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [motor_number]
    data.motor_number = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [individual_motors_speed]
    data.individual_motors_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [right_speed]
    data.right_speed = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [left_speed]
    data.left_speed = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sixwd_msgs/SixWheelCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8eb7be4689f84a603224726e6da73ba6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 controltype  #For individual control send 0 for linear control send 1
    int16 linearspeed  #Linear Speed or right or left speed with angle and what to command
    int16 angle
    uint8 motor_number #Select motor 
    int16 individual_motors_speed #İndividual speed commands for each motor
    int16 right_speed
    int16 left_speed
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SixWheelCommand(null);
    if (msg.controltype !== undefined) {
      resolved.controltype = msg.controltype;
    }
    else {
      resolved.controltype = 0
    }

    if (msg.linearspeed !== undefined) {
      resolved.linearspeed = msg.linearspeed;
    }
    else {
      resolved.linearspeed = 0
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0
    }

    if (msg.motor_number !== undefined) {
      resolved.motor_number = msg.motor_number;
    }
    else {
      resolved.motor_number = 0
    }

    if (msg.individual_motors_speed !== undefined) {
      resolved.individual_motors_speed = msg.individual_motors_speed;
    }
    else {
      resolved.individual_motors_speed = 0
    }

    if (msg.right_speed !== undefined) {
      resolved.right_speed = msg.right_speed;
    }
    else {
      resolved.right_speed = 0
    }

    if (msg.left_speed !== undefined) {
      resolved.left_speed = msg.left_speed;
    }
    else {
      resolved.left_speed = 0
    }

    return resolved;
    }
};

module.exports = SixWheelCommand;
