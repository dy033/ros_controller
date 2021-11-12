// Generated by gencpp from file hunter_msgs/HunterStatus.msg
// DO NOT EDIT!


#ifndef HUNTER_MSGS_MESSAGE_HUNTERSTATUS_H
#define HUNTER_MSGS_MESSAGE_HUNTERSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include "HunterMotorState.h"


namespace hunter_msgs
{
template <class ContainerAllocator>
struct HunterStatus_
{
  typedef HunterStatus_<ContainerAllocator> Type;

  HunterStatus_()
    : header()
    , linear_velocity(0.0)
    , steering_angle(0.0)
    , base_state(0)
    , control_mode(0)
    , fault_code(0)
    , battery_voltage(0.0)
    , motor_states()  {
    }
  HunterStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , linear_velocity(0.0)
    , steering_angle(0.0)
    , base_state(0)
    , control_mode(0)
    , fault_code(0)
    , battery_voltage(0.0)
    , motor_states()  {
  (void)_alloc;
      motor_states.assign( ::hunter_msgs::HunterMotorState_<ContainerAllocator> (_alloc));
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _linear_velocity_type;
  _linear_velocity_type linear_velocity;

   typedef double _steering_angle_type;
  _steering_angle_type steering_angle;

   typedef uint8_t _base_state_type;
  _base_state_type base_state;

   typedef uint8_t _control_mode_type;
  _control_mode_type control_mode;

   typedef uint16_t _fault_code_type;
  _fault_code_type fault_code;

   typedef double _battery_voltage_type;
  _battery_voltage_type battery_voltage;

   typedef boost::array< ::hunter_msgs::HunterMotorState_<ContainerAllocator> , 3>  _motor_states_type;
  _motor_states_type motor_states;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(MOTOR_ID_FRONT)
  #undef MOTOR_ID_FRONT
#endif
#if defined(_WIN32) && defined(MOTOR_ID_REAR_LEFT)
  #undef MOTOR_ID_REAR_LEFT
#endif
#if defined(_WIN32) && defined(MOTOR_ID_REAR_RIGHT)
  #undef MOTOR_ID_REAR_RIGHT
#endif

  enum {
    MOTOR_ID_FRONT = 0,
    MOTOR_ID_REAR_LEFT = 1,
    MOTOR_ID_REAR_RIGHT = 2,
  };


  typedef boost::shared_ptr< ::hunter_msgs::HunterStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hunter_msgs::HunterStatus_<ContainerAllocator> const> ConstPtr;

}; // struct HunterStatus_

typedef ::hunter_msgs::HunterStatus_<std::allocator<void> > HunterStatus;

typedef boost::shared_ptr< ::hunter_msgs::HunterStatus > HunterStatusPtr;
typedef boost::shared_ptr< ::hunter_msgs::HunterStatus const> HunterStatusConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hunter_msgs::HunterStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hunter_msgs::HunterStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hunter_msgs::HunterStatus_<ContainerAllocator1> & lhs, const ::hunter_msgs::HunterStatus_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.linear_velocity == rhs.linear_velocity &&
    lhs.steering_angle == rhs.steering_angle &&
    lhs.base_state == rhs.base_state &&
    lhs.control_mode == rhs.control_mode &&
    lhs.fault_code == rhs.fault_code &&
    lhs.battery_voltage == rhs.battery_voltage &&
    lhs.motor_states == rhs.motor_states;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hunter_msgs::HunterStatus_<ContainerAllocator1> & lhs, const ::hunter_msgs::HunterStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hunter_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hunter_msgs::HunterStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hunter_msgs::HunterStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hunter_msgs::HunterStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "efc4e158e9d941f4c24da59466caa5b6";
  }

  static const char* value(const ::hunter_msgs::HunterStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xefc4e158e9d941f4ULL;
  static const uint64_t static_value2 = 0xc24da59466caa5b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hunter_msgs/HunterStatus";
  }

  static const char* value(const ::hunter_msgs::HunterStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"int8 MOTOR_ID_FRONT = 0\n"
"int8 MOTOR_ID_REAR_LEFT = 1\n"
"int8 MOTOR_ID_REAR_RIGHT = 2\n"
"\n"
"# motion state\n"
"float64 linear_velocity\n"
"float64 steering_angle\n"
"\n"
"# base state\n"
"uint8 base_state\n"
"uint8 control_mode\n"
"uint16 fault_code\n"
"float64 battery_voltage\n"
"\n"
"# motor state\n"
"HunterMotorState[3] motor_states\n"
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
"\n"
"================================================================================\n"
"MSG: hunter_msgs/HunterMotorState\n"
"float64 current\n"
"float64 rpm\n"
"float64 temperature\n"
;
  }

  static const char* value(const ::hunter_msgs::HunterStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.linear_velocity);
      stream.next(m.steering_angle);
      stream.next(m.base_state);
      stream.next(m.control_mode);
      stream.next(m.fault_code);
      stream.next(m.battery_voltage);
      stream.next(m.motor_states);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HunterStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hunter_msgs::HunterStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hunter_msgs::HunterStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "linear_velocity: ";
    Printer<double>::stream(s, indent + "  ", v.linear_velocity);
    s << indent << "steering_angle: ";
    Printer<double>::stream(s, indent + "  ", v.steering_angle);
    s << indent << "base_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.base_state);
    s << indent << "control_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.control_mode);
    s << indent << "fault_code: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.fault_code);
    s << indent << "battery_voltage: ";
    Printer<double>::stream(s, indent + "  ", v.battery_voltage);
    s << indent << "motor_states[]" << std::endl;
    for (size_t i = 0; i < v.motor_states.size(); ++i)
    {
      s << indent << "  motor_states[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >::stream(s, indent + "    ", v.motor_states[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HUNTER_MSGS_MESSAGE_HUNTERSTATUS_H
