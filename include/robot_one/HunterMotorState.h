// Generated by gencpp from file hunter_msgs/HunterMotorState.msg
// DO NOT EDIT!


#ifndef HUNTER_MSGS_MESSAGE_HUNTERMOTORSTATE_H
#define HUNTER_MSGS_MESSAGE_HUNTERMOTORSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hunter_msgs
{
template <class ContainerAllocator>
struct HunterMotorState_
{
  typedef HunterMotorState_<ContainerAllocator> Type;

  HunterMotorState_()
    : current(0.0)
    , rpm(0.0)
    , temperature(0.0)  {
    }
  HunterMotorState_(const ContainerAllocator& _alloc)
    : current(0.0)
    , rpm(0.0)
    , temperature(0.0)  {
  (void)_alloc;
    }



   typedef double _current_type;
  _current_type current;

   typedef double _rpm_type;
  _rpm_type rpm;

   typedef double _temperature_type;
  _temperature_type temperature;





  typedef boost::shared_ptr< ::hunter_msgs::HunterMotorState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hunter_msgs::HunterMotorState_<ContainerAllocator> const> ConstPtr;

}; // struct HunterMotorState_

typedef ::hunter_msgs::HunterMotorState_<std::allocator<void> > HunterMotorState;

typedef boost::shared_ptr< ::hunter_msgs::HunterMotorState > HunterMotorStatePtr;
typedef boost::shared_ptr< ::hunter_msgs::HunterMotorState const> HunterMotorStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hunter_msgs::HunterMotorState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hunter_msgs::HunterMotorState_<ContainerAllocator1> & lhs, const ::hunter_msgs::HunterMotorState_<ContainerAllocator2> & rhs)
{
  return lhs.current == rhs.current &&
    lhs.rpm == rhs.rpm &&
    lhs.temperature == rhs.temperature;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hunter_msgs::HunterMotorState_<ContainerAllocator1> & lhs, const ::hunter_msgs::HunterMotorState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hunter_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hunter_msgs::HunterMotorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hunter_msgs::HunterMotorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hunter_msgs::HunterMotorState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9380628b50ebdc90ce46d4147360680d";
  }

  static const char* value(const ::hunter_msgs::HunterMotorState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9380628b50ebdc90ULL;
  static const uint64_t static_value2 = 0xce46d4147360680dULL;
};

template<class ContainerAllocator>
struct DataType< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hunter_msgs/HunterMotorState";
  }

  static const char* value(const ::hunter_msgs::HunterMotorState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 current\n"
"float64 rpm\n"
"float64 temperature\n"
;
  }

  static const char* value(const ::hunter_msgs::HunterMotorState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current);
      stream.next(m.rpm);
      stream.next(m.temperature);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HunterMotorState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hunter_msgs::HunterMotorState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hunter_msgs::HunterMotorState_<ContainerAllocator>& v)
  {
    s << indent << "current: ";
    Printer<double>::stream(s, indent + "  ", v.current);
    s << indent << "rpm: ";
    Printer<double>::stream(s, indent + "  ", v.rpm);
    s << indent << "temperature: ";
    Printer<double>::stream(s, indent + "  ", v.temperature);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HUNTER_MSGS_MESSAGE_HUNTERMOTORSTATE_H
