// Generated by gencpp from file duckietown_msgs/GetVariableRequest.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_GETVARIABLEREQUEST_H
#define DUCKIETOWN_MSGS_MESSAGE_GETVARIABLEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

namespace duckietown_msgs
{
template <class ContainerAllocator>
struct GetVariableRequest_
{
  typedef GetVariableRequest_<ContainerAllocator> Type;

  GetVariableRequest_()
    : name_json()  {
    }
  GetVariableRequest_(const ContainerAllocator& _alloc)
    : name_json(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _name_json_type;
  _name_json_type name_json;





  typedef boost::shared_ptr< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetVariableRequest_

typedef ::duckietown_msgs::GetVariableRequest_<std::allocator<void> > GetVariableRequest;

typedef boost::shared_ptr< ::duckietown_msgs::GetVariableRequest > GetVariableRequestPtr;
typedef boost::shared_ptr< ::duckietown_msgs::GetVariableRequest const> GetVariableRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator1> & lhs, const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator2> & rhs)
{
  return lhs.name_json == rhs.name_json;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator1> & lhs, const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e62a392e1985c0f620cc4494f046ad84";
  }

  static const char* value(const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe62a392e1985c0f6ULL;
  static const uint64_t static_value2 = 0x20cc4494f046ad84ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/GetVariableRequest";
  }

  static const char* value(const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/String name_json\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/String\n"
"string data\n"
;
  }

  static const char* value(const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name_json);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetVariableRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::GetVariableRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::GetVariableRequest_<ContainerAllocator>& v)
  {
    s << indent << "name_json: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.name_json);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_GETVARIABLEREQUEST_H
