// Generated by gencpp from file track_detection/PointMsg.msg
// DO NOT EDIT!


#ifndef TRACK_DETECTION_MESSAGE_POINTMSG_H
#define TRACK_DETECTION_MESSAGE_POINTMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace track_detection
{
template <class ContainerAllocator>
struct PointMsg_
{
  typedef PointMsg_<ContainerAllocator> Type;

  PointMsg_()
    : x(0.0)
    , y(0.0)  {
    }
  PointMsg_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::track_detection::PointMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::track_detection::PointMsg_<ContainerAllocator> const> ConstPtr;

}; // struct PointMsg_

typedef ::track_detection::PointMsg_<std::allocator<void> > PointMsg;

typedef boost::shared_ptr< ::track_detection::PointMsg > PointMsgPtr;
typedef boost::shared_ptr< ::track_detection::PointMsg const> PointMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::track_detection::PointMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::track_detection::PointMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace track_detection

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': True, 'HasHeader': False}
// {'track_detection': ['/home/karel/Documents/University/WS2019/AAIP/software_integration/src/track_detection/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::track_detection::PointMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::track_detection::PointMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::track_detection::PointMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::track_detection::PointMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::track_detection::PointMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::track_detection::PointMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::track_detection::PointMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "209f516d3eb691f0663e25cb750d67c1";
  }

  static const char* value(const ::track_detection::PointMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x209f516d3eb691f0ULL;
  static const uint64_t static_value2 = 0x663e25cb750d67c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::track_detection::PointMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "track_detection/PointMsg";
  }

  static const char* value(const ::track_detection::PointMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::track_detection::PointMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
;
  }

  static const char* value(const ::track_detection::PointMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::track_detection::PointMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PointMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::track_detection::PointMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::track_detection::PointMsg_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRACK_DETECTION_MESSAGE_POINTMSG_H
