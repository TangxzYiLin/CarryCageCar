# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from chassis_drive/chassis_cmd.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class chassis_cmd(genpy.Message):
  _md5sum = "8c01b3f6487ca68f987b16e7e5933bc6"
  _type = "chassis_drive/chassis_cmd"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int16 chassis_vel_cmd_  
int16 chassis_angle_cmd_
int16 chassis_indicator_cmd_
int16 chassis_brake_cmd_
"""
  __slots__ = ['chassis_vel_cmd_','chassis_angle_cmd_','chassis_indicator_cmd_','chassis_brake_cmd_']
  _slot_types = ['int16','int16','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       chassis_vel_cmd_,chassis_angle_cmd_,chassis_indicator_cmd_,chassis_brake_cmd_

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(chassis_cmd, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.chassis_vel_cmd_ is None:
        self.chassis_vel_cmd_ = 0
      if self.chassis_angle_cmd_ is None:
        self.chassis_angle_cmd_ = 0
      if self.chassis_indicator_cmd_ is None:
        self.chassis_indicator_cmd_ = 0
      if self.chassis_brake_cmd_ is None:
        self.chassis_brake_cmd_ = 0
    else:
      self.chassis_vel_cmd_ = 0
      self.chassis_angle_cmd_ = 0
      self.chassis_indicator_cmd_ = 0
      self.chassis_brake_cmd_ = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_4h().pack(_x.chassis_vel_cmd_, _x.chassis_angle_cmd_, _x.chassis_indicator_cmd_, _x.chassis_brake_cmd_))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.chassis_vel_cmd_, _x.chassis_angle_cmd_, _x.chassis_indicator_cmd_, _x.chassis_brake_cmd_,) = _get_struct_4h().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_4h().pack(_x.chassis_vel_cmd_, _x.chassis_angle_cmd_, _x.chassis_indicator_cmd_, _x.chassis_brake_cmd_))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.chassis_vel_cmd_, _x.chassis_angle_cmd_, _x.chassis_indicator_cmd_, _x.chassis_brake_cmd_,) = _get_struct_4h().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4h = None
def _get_struct_4h():
    global _struct_4h
    if _struct_4h is None:
        _struct_4h = struct.Struct("<4h")
    return _struct_4h