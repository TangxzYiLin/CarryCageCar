// Generated by gencpp from file chassis_drive/chassis_cmd.msg
// DO NOT EDIT!


#ifndef CHASSIS_DRIVE_MESSAGE_CHASSIS_CMD_H
#define CHASSIS_DRIVE_MESSAGE_CHASSIS_CMD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace chassis_drive
{
template <class ContainerAllocator>
struct chassis_cmd_
{
  typedef chassis_cmd_<ContainerAllocator> Type;

  chassis_cmd_()
    : chassis_vel_cmd_(0)
    , chassis_angle_cmd_(0)
    , chassis_indicator_cmd_(0)
    , chassis_brake_cmd_(0)  {
    }
  chassis_cmd_(const ContainerAllocator& _alloc)
    : chassis_vel_cmd_(0)
    , chassis_angle_cmd_(0)
    , chassis_indicator_cmd_(0)
    , chassis_brake_cmd_(0)  {
  (void)_alloc;
    }



   typedef int16_t _chassis_vel_cmd__type;
  _chassis_vel_cmd__type chassis_vel_cmd_;

   typedef int16_t _chassis_angle_cmd__type;
  _chassis_angle_cmd__type chassis_angle_cmd_;

   typedef int16_t _chassis_indicator_cmd__type;
  _chassis_indicator_cmd__type chassis_indicator_cmd_;

   typedef int16_t _chassis_brake_cmd__type;
  _chassis_brake_cmd__type chassis_brake_cmd_;





  typedef boost::shared_ptr< ::chassis_drive::chassis_cmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chassis_drive::chassis_cmd_<ContainerAllocator> const> ConstPtr;

}; // struct chassis_cmd_

typedef ::chassis_drive::chassis_cmd_<std::allocator<void> > chassis_cmd;

typedef boost::shared_ptr< ::chassis_drive::chassis_cmd > chassis_cmdPtr;
typedef boost::shared_ptr< ::chassis_drive::chassis_cmd const> chassis_cmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chassis_drive::chassis_cmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chassis_drive::chassis_cmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::chassis_drive::chassis_cmd_<ContainerAllocator1> & lhs, const ::chassis_drive::chassis_cmd_<ContainerAllocator2> & rhs)
{
  return lhs.chassis_vel_cmd_ == rhs.chassis_vel_cmd_ &&
    lhs.chassis_angle_cmd_ == rhs.chassis_angle_cmd_ &&
    lhs.chassis_indicator_cmd_ == rhs.chassis_indicator_cmd_ &&
    lhs.chassis_brake_cmd_ == rhs.chassis_brake_cmd_;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::chassis_drive::chassis_cmd_<ContainerAllocator1> & lhs, const ::chassis_drive::chassis_cmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace chassis_drive

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chassis_drive::chassis_cmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chassis_drive::chassis_cmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chassis_drive::chassis_cmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8c01b3f6487ca68f987b16e7e5933bc6";
  }

  static const char* value(const ::chassis_drive::chassis_cmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8c01b3f6487ca68fULL;
  static const uint64_t static_value2 = 0x987b16e7e5933bc6ULL;
};

template<class ContainerAllocator>
struct DataType< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chassis_drive/chassis_cmd";
  }

  static const char* value(const ::chassis_drive::chassis_cmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 chassis_vel_cmd_  \n"
"int16 chassis_angle_cmd_\n"
"int16 chassis_indicator_cmd_\n"
"int16 chassis_brake_cmd_\n"
;
  }

  static const char* value(const ::chassis_drive::chassis_cmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.chassis_vel_cmd_);
      stream.next(m.chassis_angle_cmd_);
      stream.next(m.chassis_indicator_cmd_);
      stream.next(m.chassis_brake_cmd_);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct chassis_cmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chassis_drive::chassis_cmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::chassis_drive::chassis_cmd_<ContainerAllocator>& v)
  {
    s << indent << "chassis_vel_cmd_: ";
    Printer<int16_t>::stream(s, indent + "  ", v.chassis_vel_cmd_);
    s << indent << "chassis_angle_cmd_: ";
    Printer<int16_t>::stream(s, indent + "  ", v.chassis_angle_cmd_);
    s << indent << "chassis_indicator_cmd_: ";
    Printer<int16_t>::stream(s, indent + "  ", v.chassis_indicator_cmd_);
    s << indent << "chassis_brake_cmd_: ";
    Printer<int16_t>::stream(s, indent + "  ", v.chassis_brake_cmd_);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CHASSIS_DRIVE_MESSAGE_CHASSIS_CMD_H
