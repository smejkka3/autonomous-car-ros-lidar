// Generated by gencpp from file sixwd_msgs/SixWheelCommand.msg
// DO NOT EDIT!


#ifndef SIXWD_MSGS_MESSAGE_SIXWHEELCOMMAND_H
#define SIXWD_MSGS_MESSAGE_SIXWHEELCOMMAND_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sixwd_msgs
{
template <class ContainerAllocator>
struct SixWheelCommand_
{
  typedef SixWheelCommand_<ContainerAllocator> Type;

  SixWheelCommand_()
    : controltype(0)
    , linearspeed(0)
    , angle(0)
    , motor_number(0)
    , individual_motors_speed(0)
    , right_speed(0)
    , left_speed(0)  {
    }
  SixWheelCommand_(const ContainerAllocator& _alloc)
    : controltype(0)
    , linearspeed(0)
    , angle(0)
    , motor_number(0)
    , individual_motors_speed(0)
    , right_speed(0)
    , left_speed(0)  {
  (void)_alloc;
    }



   typedef int16_t _controltype_type;
  _controltype_type controltype;

   typedef int16_t _linearspeed_type;
  _linearspeed_type linearspeed;

   typedef int16_t _angle_type;
  _angle_type angle;

   typedef uint8_t _motor_number_type;
  _motor_number_type motor_number;

   typedef int16_t _individual_motors_speed_type;
  _individual_motors_speed_type individual_motors_speed;

   typedef int16_t _right_speed_type;
  _right_speed_type right_speed;

   typedef int16_t _left_speed_type;
  _left_speed_type left_speed;





  typedef boost::shared_ptr< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> const> ConstPtr;

}; // struct SixWheelCommand_

typedef ::sixwd_msgs::SixWheelCommand_<std::allocator<void> > SixWheelCommand;

typedef boost::shared_ptr< ::sixwd_msgs::SixWheelCommand > SixWheelCommandPtr;
typedef boost::shared_ptr< ::sixwd_msgs::SixWheelCommand const> SixWheelCommandConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sixwd_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'sixwd_msgs': ['/home/karel/Documents/University/WS2019/AAIP/software_integration/src/sixwd_msgs/src/sixwd_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8eb7be4689f84a603224726e6da73ba6";
  }

  static const char* value(const ::sixwd_msgs::SixWheelCommand_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8eb7be4689f84a60ULL;
  static const uint64_t static_value2 = 0x3224726e6da73ba6ULL;
};

template<class ContainerAllocator>
struct DataType< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sixwd_msgs/SixWheelCommand";
  }

  static const char* value(const ::sixwd_msgs::SixWheelCommand_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 controltype  #For individual control send 0 for linear control send 1\n"
"int16 linearspeed  #Linear Speed or right or left speed with angle and what to command\n"
"int16 angle\n"
"uint8 motor_number #Select motor \n"
"int16 individual_motors_speed #İndividual speed commands for each motor\n"
"int16 right_speed\n"
"int16 left_speed\n"
"\n"
;
  }

  static const char* value(const ::sixwd_msgs::SixWheelCommand_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.controltype);
      stream.next(m.linearspeed);
      stream.next(m.angle);
      stream.next(m.motor_number);
      stream.next(m.individual_motors_speed);
      stream.next(m.right_speed);
      stream.next(m.left_speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SixWheelCommand_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sixwd_msgs::SixWheelCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sixwd_msgs::SixWheelCommand_<ContainerAllocator>& v)
  {
    s << indent << "controltype: ";
    Printer<int16_t>::stream(s, indent + "  ", v.controltype);
    s << indent << "linearspeed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.linearspeed);
    s << indent << "angle: ";
    Printer<int16_t>::stream(s, indent + "  ", v.angle);
    s << indent << "motor_number: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motor_number);
    s << indent << "individual_motors_speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.individual_motors_speed);
    s << indent << "right_speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.right_speed);
    s << indent << "left_speed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.left_speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIXWD_MSGS_MESSAGE_SIXWHEELCOMMAND_H
