// Generated by gencpp from file track_detection/TrackMsg.msg
// DO NOT EDIT!


#ifndef TRACK_DETECTION_MESSAGE_TRACKMSG_H
#define TRACK_DETECTION_MESSAGE_TRACKMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <track_detection/PointMsg.h>
#include <track_detection/PointMsg.h>

namespace track_detection
{
template <class ContainerAllocator>
struct TrackMsg_
{
  typedef TrackMsg_<ContainerAllocator> Type;

  TrackMsg_()
    : leftLength(0)
    , left()
    , rightLength(0)
    , right()  {
    }
  TrackMsg_(const ContainerAllocator& _alloc)
    : leftLength(0)
    , left(_alloc)
    , rightLength(0)
    , right(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _leftLength_type;
  _leftLength_type leftLength;

   typedef std::vector< ::track_detection::PointMsg_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::track_detection::PointMsg_<ContainerAllocator> >::other >  _left_type;
  _left_type left;

   typedef int32_t _rightLength_type;
  _rightLength_type rightLength;

   typedef std::vector< ::track_detection::PointMsg_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::track_detection::PointMsg_<ContainerAllocator> >::other >  _right_type;
  _right_type right;





  typedef boost::shared_ptr< ::track_detection::TrackMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::track_detection::TrackMsg_<ContainerAllocator> const> ConstPtr;

}; // struct TrackMsg_

typedef ::track_detection::TrackMsg_<std::allocator<void> > TrackMsg;

typedef boost::shared_ptr< ::track_detection::TrackMsg > TrackMsgPtr;
typedef boost::shared_ptr< ::track_detection::TrackMsg const> TrackMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::track_detection::TrackMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::track_detection::TrackMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace track_detection

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'IsFixedSize': False, 'HasHeader': False}
// {'track_detection': ['/home/karel/Documents/University/WS2019/AAIP/software_integration/src/track_detection/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::track_detection::TrackMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::track_detection::TrackMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::track_detection::TrackMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::track_detection::TrackMsg_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::track_detection::TrackMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::track_detection::TrackMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::track_detection::TrackMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cdb50cb692e6a3e0b32b69782a762116";
  }

  static const char* value(const ::track_detection::TrackMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcdb50cb692e6a3e0ULL;
  static const uint64_t static_value2 = 0xb32b69782a762116ULL;
};

template<class ContainerAllocator>
struct DataType< ::track_detection::TrackMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "track_detection/TrackMsg";
  }

  static const char* value(const ::track_detection::TrackMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::track_detection::TrackMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# TODO Add track direction\n"
"\n"
"int32 leftLength\n"
"PointMsg[] left\n"
"\n"
"int32 rightLength\n"
"PointMsg[] right\n"
"================================================================================\n"
"MSG: track_detection/PointMsg\n"
"float64 x\n"
"float64 y\n"
;
  }

  static const char* value(const ::track_detection::TrackMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::track_detection::TrackMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.leftLength);
      stream.next(m.left);
      stream.next(m.rightLength);
      stream.next(m.right);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::track_detection::TrackMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::track_detection::TrackMsg_<ContainerAllocator>& v)
  {
    s << indent << "leftLength: ";
    Printer<int32_t>::stream(s, indent + "  ", v.leftLength);
    s << indent << "left[]" << std::endl;
    for (size_t i = 0; i < v.left.size(); ++i)
    {
      s << indent << "  left[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::track_detection::PointMsg_<ContainerAllocator> >::stream(s, indent + "    ", v.left[i]);
    }
    s << indent << "rightLength: ";
    Printer<int32_t>::stream(s, indent + "  ", v.rightLength);
    s << indent << "right[]" << std::endl;
    for (size_t i = 0; i < v.right.size(); ++i)
    {
      s << indent << "  right[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::track_detection::PointMsg_<ContainerAllocator> >::stream(s, indent + "    ", v.right[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRACK_DETECTION_MESSAGE_TRACKMSG_H
