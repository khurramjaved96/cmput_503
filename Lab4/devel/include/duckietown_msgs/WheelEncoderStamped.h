// Generated by gencpp from file duckietown_msgs/WheelEncoderStamped.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_WHEELENCODERSTAMPED_H
#define DUCKIETOWN_MSGS_MESSAGE_WHEELENCODERSTAMPED_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace duckietown_msgs
{
template <class ContainerAllocator>
struct WheelEncoderStamped_
{
  typedef WheelEncoderStamped_<ContainerAllocator> Type;

  WheelEncoderStamped_()
    : header()
    , data(0)
    , resolution(0)
    , type(0)  {
    }
  WheelEncoderStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , data(0)
    , resolution(0)
    , type(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _data_type;
  _data_type data;

   typedef uint16_t _resolution_type;
  _resolution_type resolution;

   typedef uint8_t _type_type;
  _type_type type;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ENCODER_TYPE_ABSOLUTE)
  #undef ENCODER_TYPE_ABSOLUTE
#endif
#if defined(_WIN32) && defined(ENCODER_TYPE_INCREMENTAL)
  #undef ENCODER_TYPE_INCREMENTAL
#endif

  enum {
    ENCODER_TYPE_ABSOLUTE = 0u,
    ENCODER_TYPE_INCREMENTAL = 1u,
  };


  typedef boost::shared_ptr< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> const> ConstPtr;

}; // struct WheelEncoderStamped_

typedef ::duckietown_msgs::WheelEncoderStamped_<std::allocator<void> > WheelEncoderStamped;

typedef boost::shared_ptr< ::duckietown_msgs::WheelEncoderStamped > WheelEncoderStampedPtr;
typedef boost::shared_ptr< ::duckietown_msgs::WheelEncoderStamped const> WheelEncoderStampedConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator1> & lhs, const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.data == rhs.data &&
    lhs.resolution == rhs.resolution &&
    lhs.type == rhs.type;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator1> & lhs, const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "141b74ec2cc3bde8c38b5e3bdb694d12";
  }

  static const char* value(const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x141b74ec2cc3bde8ULL;
  static const uint64_t static_value2 = 0xc38b5e3bdb694d12ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/WheelEncoderStamped";
  }

  static const char* value(const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Enum: encoder type\n"
"uint8 ENCODER_TYPE_ABSOLUTE = 0\n"
"uint8 ENCODER_TYPE_INCREMENTAL = 1\n"
"\n"
"Header header\n"
"int32 data\n"
"uint16 resolution\n"
"uint8 type\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.data);
      stream.next(m.resolution);
      stream.next(m.type);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelEncoderStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::WheelEncoderStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data);
    s << indent << "resolution: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.resolution);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_WHEELENCODERSTAMPED_H
