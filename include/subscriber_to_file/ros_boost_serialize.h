#ifndef ROS_BOOST_SERIALIZE_H
#define ROS_BOOST_SERIALIZE_H

#include "sensor_msgs/Imu.h"

// include headers that implement a archive in simple text format
//
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>


namespace ros_boost_serialize {

// exemple : T = ::sensor_msgs::Imu
template<typename T>
class vector3_ros_serialize_type
{
    friend class boost::serialization::access;
    double x, y, z;

protected:
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_NVP(x) & \
                BOOST_SERIALIZATION_NVP(y) & \
                BOOST_SERIALIZATION_NVP(z);
    }

public:
    typedef boost::shared_ptr< T const> TConstPtr;

    vector3_ros_serialize_type(const TConstPtr & msg_) :
        x(msg_->orientation.x), y(msg_->orientation.y), z(msg_->orientation.z)
    {}
};

template<typename T>
class vector4_ros_serialize_type :
        public vector3_ros_serialize_type<T>
{
    friend class boost::serialization::access;

    double w;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        vector3_ros_serialize_type<T>::serialize(ar, version);
        ar & BOOST_SERIALIZATION_NVP(w);
    }

public:
    typedef boost::shared_ptr< T const> TConstPtr;

    vector4_ros_serialize_type(const TConstPtr & msg_) :
        vector3_ros_serialize_type<T>(msg_),
        w(msg_->orientation.w)
    {}
};

}

BOOST_CLASS_VERSION(ros_boost_serialize::vector3_ros_serialize_type<sensor_msgs::Imu>, 1);
BOOST_CLASS_VERSION(ros_boost_serialize::vector4_ros_serialize_type<sensor_msgs::Imu>, 1);

#endif // ROS_BOOST_SERIALIZE_H
