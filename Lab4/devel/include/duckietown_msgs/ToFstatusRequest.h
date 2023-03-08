// Generated by gencpp from file duckietown_msgs/ToFstatusRequest.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_TOFSTATUSREQUEST_H
#define DUCKIETOWN_MSGS_MESSAGE_TOFSTATUSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace duckietown_msgs
{
template <class ContainerAllocator>
struct ToFstatusRequest_
{
  typedef ToFstatusRequest_<ContainerAllocator> Type;

  ToFstatusRequest_()
    : sensor_position()  {
    }
  ToFstatusRequest_(const ContainerAllocator& _alloc)
    : sensor_position(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _sensor_position_type;
  _sensor_position_type sensor_position;





  typedef boost::shared_ptr< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> const> ConstPtr;

}; // struct ToFstatusRequest_

typedef ::duckietown_msgs::ToFstatusRequest_<std::allocator<void> > ToFstatusRequest;

typedef boost::shared_ptr< ::duckietown_msgs::ToFstatusRequest > ToFstatusRequestPtr;
typedef boost::shared_ptr< ::duckietown_msgs::ToFstatusRequest const> ToFstatusRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator1> & lhs, const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator2> & rhs)
{
  return lhs.sensor_position == rhs.sensor_position;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator1> & lhs, const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "92d95aecfa07c3669b7ca7c238562a18";
  }

  static const char* value(const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x92d95aecfa07c366ULL;
  static const uint64_t static_value2 = 0x9b7ca7c238562a18ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/ToFstatusRequest";
  }

  static const char* value(const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string sensor_position    #expect tof_fl, tof_fm, tof_fr, tof_sl, tof_sr, tof_bl, tof_br\n"
;
  }

  static const char* value(const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sensor_position);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ToFstatusRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::ToFstatusRequest_<ContainerAllocator>& v)
  {
    s << indent << "sensor_position: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.sensor_position);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_TOFSTATUSREQUEST_H
