// Generated by gencpp from file duckietown_msgs/SignalsDetection.msg
// DO NOT EDIT!


#ifndef DUCKIETOWN_MSGS_MESSAGE_SIGNALSDETECTION_H
#define DUCKIETOWN_MSGS_MESSAGE_SIGNALSDETECTION_H


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
struct SignalsDetection_
{
  typedef SignalsDetection_<ContainerAllocator> Type;

  SignalsDetection_()
    : header()
    , front()
    , right()
    , left()
    , traffic_light_state()  {
    }
  SignalsDetection_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , front(_alloc)
    , right(_alloc)
    , left(_alloc)
    , traffic_light_state(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _front_type;
  _front_type front;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _right_type;
  _right_type right;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _left_type;
  _left_type left;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _traffic_light_state_type;
  _traffic_light_state_type traffic_light_state;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(NO_CAR)
  #undef NO_CAR
#endif
#if defined(_WIN32) && defined(SIGNAL_A)
  #undef SIGNAL_A
#endif
#if defined(_WIN32) && defined(SIGNAL_B)
  #undef SIGNAL_B
#endif
#if defined(_WIN32) && defined(SIGNAL_C)
  #undef SIGNAL_C
#endif
#if defined(_WIN32) && defined(SIGNAL_PRIORITY)
  #undef SIGNAL_PRIORITY
#endif
#if defined(_WIN32) && defined(SIGNAL_SACRIFICE_FOR_PRIORITY)
  #undef SIGNAL_SACRIFICE_FOR_PRIORITY
#endif
#if defined(_WIN32) && defined(NO_CARS)
  #undef NO_CARS
#endif
#if defined(_WIN32) && defined(CARS)
  #undef CARS
#endif
#if defined(_WIN32) && defined(NO_TRAFFIC_LIGHT)
  #undef NO_TRAFFIC_LIGHT
#endif
#if defined(_WIN32) && defined(STOP)
  #undef STOP
#endif
#if defined(_WIN32) && defined(GO)
  #undef GO
#endif
#if defined(_WIN32) && defined(YIELD)
  #undef YIELD
#endif


  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> NO_CAR;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> SIGNAL_A;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> SIGNAL_B;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> SIGNAL_C;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> SIGNAL_PRIORITY;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> SIGNAL_SACRIFICE_FOR_PRIORITY;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> NO_CARS;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> CARS;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> NO_TRAFFIC_LIGHT;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> STOP;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> GO;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> YIELD;

  typedef boost::shared_ptr< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> const> ConstPtr;

}; // struct SignalsDetection_

typedef ::duckietown_msgs::SignalsDetection_<std::allocator<void> > SignalsDetection;

typedef boost::shared_ptr< ::duckietown_msgs::SignalsDetection > SignalsDetectionPtr;
typedef boost::shared_ptr< ::duckietown_msgs::SignalsDetection const> SignalsDetectionConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::NO_CAR =
        
          "'no_car_detected'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::SIGNAL_A =
        
          "'car_signal_A'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::SIGNAL_B =
        
          "'car_signal_B'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::SIGNAL_C =
        
          "'car_signal_C'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::SIGNAL_PRIORITY =
        
          "'car_signal_priority'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::SIGNAL_SACRIFICE_FOR_PRIORITY =
        
          "'car_signal_sacrifice_for_priority'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::NO_CARS =
        
          "'no_cars_detected'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::CARS =
        
          "'cars_detected'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::NO_TRAFFIC_LIGHT =
        
          "'no_traffic_light'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::STOP =
        
          "'tl_stop'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::GO =
        
          "'tl_go'"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
      SignalsDetection_<ContainerAllocator>::YIELD =
        
          "'tl_yield'"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::duckietown_msgs::SignalsDetection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::duckietown_msgs::SignalsDetection_<ContainerAllocator1> & lhs, const ::duckietown_msgs::SignalsDetection_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.front == rhs.front &&
    lhs.right == rhs.right &&
    lhs.left == rhs.left &&
    lhs.traffic_light_state == rhs.traffic_light_state;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::duckietown_msgs::SignalsDetection_<ContainerAllocator1> & lhs, const ::duckietown_msgs::SignalsDetection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace duckietown_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7a3bb73ea77191f1c0ddd7e196f27c75";
  }

  static const char* value(const ::duckietown_msgs::SignalsDetection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7a3bb73ea77191f1ULL;
  static const uint64_t static_value2 = 0xc0ddd7e196f27c75ULL;
};

template<class ContainerAllocator>
struct DataType< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "duckietown_msgs/SignalsDetection";
  }

  static const char* value(const ::duckietown_msgs::SignalsDetection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"# this is what we can see at the intersection:\n"
"string front\n"
"string right\n"
"string left\n"
"\n"
"# For the first backoff approach\n"
"# string led_detected\n"
"# string no_led_detected\n"
"\n"
"# Each of these can be:\n"
"string NO_CAR='no_car_detected'\n"
"string SIGNAL_A='car_signal_A'\n"
"string SIGNAL_B='car_signal_B'\n"
"string SIGNAL_C='car_signal_C'\n"
"string SIGNAL_PRIORITY='car_signal_priority'\n"
"string SIGNAL_SACRIFICE_FOR_PRIORITY='car_signal_sacrifice_for_priority'\n"
"\n"
"string NO_CARS='no_cars_detected'\n"
"string CARS   ='cars_detected'\n"
"\n"
"\n"
"# Plus we can see the traffic light\n"
"\n"
"# for the moment we assume that no traffic light exists\n"
"\n"
"string traffic_light_state\n"
"\n"
"string NO_TRAFFIC_LIGHT='no_traffic_light'\n"
"string STOP='tl_stop'\n"
"string GO='tl_go'\n"
"string YIELD='tl_yield'\n"
"\n"
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

  static const char* value(const ::duckietown_msgs::SignalsDetection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.front);
      stream.next(m.right);
      stream.next(m.left);
      stream.next(m.traffic_light_state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SignalsDetection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::duckietown_msgs::SignalsDetection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::duckietown_msgs::SignalsDetection_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "front: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.front);
    s << indent << "right: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.right);
    s << indent << "left: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.left);
    s << indent << "traffic_light_state: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.traffic_light_state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DUCKIETOWN_MSGS_MESSAGE_SIGNALSDETECTION_H
