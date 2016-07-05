#ifndef ROS_MACROS_H
#define ROS_MACROS_H

#define MACRO_XSTR(s) MACRO_STR(s)
#define MACRO_STR(s) #s

#define GET_ROS_PARAM( _param_name_, _param_member_, _default_value_, _priv_nh_, _nh_ )                             \
    if ( _priv_nh_.getParam ( MACRO_XSTR(_param_name_), _param_member_ ) )                                          \
{                                                                                                                   \
    ROS_INFO_STREAM ( MACRO_XSTR(_param_name_) << " is: " << _param_member_);                                       \
    }                                                                                                               \
    else if( _nh_.getParam ( MACRO_XSTR(_param_name_), _param_member_ ) )                                           \
{                                                                                                                   \
    ROS_WARN_STREAM ("Non-private " << MACRO_XSTR(_param_name_) << " parameter is DEPRECATED: " << _param_member_); \
    }                                                                                                               \
    else {                                                                                                          \
    _param_member_ = _default_value_;                                                                               \
    ROS_INFO_STREAM ( MACRO_XSTR(_param_name_) << " is: " << _param_member_ << "\t[default-value]");                \
    }

#endif // ROS_MACROS_H
