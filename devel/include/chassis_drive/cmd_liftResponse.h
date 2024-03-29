// Generated by gencpp from file chassis_drive/cmd_liftResponse.msg
// DO NOT EDIT!


#ifndef CHASSIS_DRIVE_MESSAGE_CMD_LIFTRESPONSE_H
#define CHASSIS_DRIVE_MESSAGE_CMD_LIFTRESPONSE_H


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
struct cmd_liftResponse_
{
  typedef cmd_liftResponse_<ContainerAllocator> Type;

  cmd_liftResponse_()
    {
    }
  cmd_liftResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> const> ConstPtr;

}; // struct cmd_liftResponse_

typedef ::chassis_drive::cmd_liftResponse_<std::allocator<void> > cmd_liftResponse;

typedef boost::shared_ptr< ::chassis_drive::cmd_liftResponse > cmd_liftResponsePtr;
typedef boost::shared_ptr< ::chassis_drive::cmd_liftResponse const> cmd_liftResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::chassis_drive::cmd_liftResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace chassis_drive

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::chassis_drive::cmd_liftResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "chassis_drive/cmd_liftResponse";
  }

  static const char* value(const ::chassis_drive::cmd_liftResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::chassis_drive::cmd_liftResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cmd_liftResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::chassis_drive::cmd_liftResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::chassis_drive::cmd_liftResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // CHASSIS_DRIVE_MESSAGE_CMD_LIFTRESPONSE_H
