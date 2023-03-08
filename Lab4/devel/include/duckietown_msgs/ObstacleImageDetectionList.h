// Generated by gencpp from file duckietown_msgs/ObstacleImageDetectionList.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_OBSTACLEIMAGEDETECTIONLIST_H
#define DUCKIETOWN_MSGS_MESSAGE_OBSTACLEIMAGEDETECTIONLIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <duckietown_msgs/ObstacleImageDetection.h>

namespace duckietown_msgs
{
template <class ContainerAllocator>
struct ObstacleImageDetectionList_
{
  typedef ObstacleImageDetectionList_<ContainerAllocator> Type;

  ObstacleImageDetectionList_()
    : header()
    , list()
    , imwidth(0.0)
    , imheight(0.0)  {
    }
  ObstacleImageDetectionList_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , list(_alloc)
    , imwidth(0.0)
    , imheight(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >> _list_type;
  _list_type list;

   typedef float _imwidth_type;
  _imwidth_type imwidth;

   typedef float _imheight_type;
  _imheight_type imheight;





  typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> const> ConstPtr;

}; // struct ObstacleImageDetectionList_

typedef ::duckietown_msgs::ObstacleImageDetectionList_<std::allocator<void> > ObstacleImageDetectionList;

typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetectionList > ObstacleImageDetectionListPtr;
typedef boost::shared_ptr< ::duckietown_msgs::ObstacleImageDetectionList const> ObstacleImageDetectionListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator1> & lhs, const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.list == rhs.list &&
    lhs.imwidth == rhs.imwidth &&
    lhs.imheight == rhs.imheight;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator1> & lhs, const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bb443595d23936bacf0f853c0dbaa48c";
  }

  static const char* value(const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbb443595d23936baULL;
  static const uint64_t static_value2 = 0xcf0f853c0dbaa48cULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/ObstacleImageDetectionList";
  }

  static const char* value(const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"duckietown_msgs/ObstacleImageDetection[] list\n"
"float32 imwidth\n"
"float32 imheight\n"
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
"\n"
"================================================================================\n"
"MSG: duckietown_msgs/ObstacleImageDetection\n"
"duckietown_msgs/Rect bounding_box\n"
"duckietown_msgs/ObstacleType type\n"
"================================================================================\n"
"MSG: duckietown_msgs/Rect\n"
"# all in pixel coordinate\n"
"# (x, y, w, h) defines a rectangle\n"
"int32 x\n"
"int32 y\n"
"int32 w\n"
"int32 h\n"
"\n"
"================================================================================\n"
"MSG: duckietown_msgs/ObstacleType\n"
"uint8 DUCKIE=0\n"
"uint8 CONE=1\n"
"uint8 type\n"
;
  }

  static const char* value(const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.list);
      stream.next(m.imwidth);
      stream.next(m.imheight);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObstacleImageDetectionList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::ObstacleImageDetectionList_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "list[]" << std::endl;
    for (size_t i = 0; i < v.list.size(); ++i)
    {
      s << indent << "  list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::duckietown_msgs::ObstacleImageDetection_<ContainerAllocator> >::stream(s, indent + "    ", v.list[i]);
    }
    s << indent << "imwidth: ";
    Printer<float>::stream(s, indent + "  ", v.imwidth);
    s << indent << "imheight: ";
    Printer<float>::stream(s, indent + "  ", v.imheight);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_OBSTACLEIMAGEDETECTIONLIST_H
