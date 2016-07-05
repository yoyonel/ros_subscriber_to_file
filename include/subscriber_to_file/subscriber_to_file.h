#ifndef ROS_SUB_TO_FILE_H
#define ROS_SUB_TO_FILE_H

#include <ros/node_handle.h>
//
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
//
#include <boost/archive/xml_oarchive.hpp>


/**
 * exemple :
 * - TMsg = ::sensor_msgs::Imu
 * - TMsgSerial = _sensor_msgs_serialize_type
 */
template<typename TMsg,
         typename TMsgSerial>
class SubToFile {
    //
    typedef boost::shared_ptr< TMsg const> TMsgConstPtr;

public:
    SubToFile(const uint32_t& _size_buffer=1,
              ros::NodeHandle _priv_nh = ros::NodeHandle("~")
            );

    virtual void run();
    virtual bool dump(const std::string & _filename);

    virtual std::string build_filename() const = 0;

protected:
    virtual void sub_cb(const TMsgConstPtr& _msg);

    template<typename T, class Archive> static void dump(const std::vector<T> & _vector, Archive & _ar) {
        for (typename std::vector<T>::const_iterator it = _vector.begin() ; it != _vector.end(); ++it)
            _ar << *it;
    }
    inline void dump(boost::archive::xml_oarchive & _ar) const { _ar & BOOST_SERIALIZATION_NVP(v_msgs_); }
    inline void dump(boost::archive::binary_oarchive & _ar) const { dump<TMsgSerial, boost::archive::binary_oarchive>(v_msgs_, _ar); }
    inline void dump(boost::archive::text_oarchive & _ar) const { dump<TMsgSerial, boost::archive::text_oarchive>(v_msgs_, _ar); }

protected:
    // ------------------------------------------------------------------------------------------------------------------
    ros::NodeHandle nh_;
    std::string     topic_name_;
    ros::Subscriber sub_;
    // ------------------------------------------------------------------------------------------------------------------

    // ------------------------------------------------------------------------------------------------------------------
    std::vector<TMsgSerial> v_msgs_;
    size_t v_size_buffer_;

    // ------------------------------------------------------------------------------------------------------------------

    // ------------------------------------------------------------------------------------------------------------------
    //
    bool binary_;
    bool xml_;
    //
    bool show_msg_;
    // ------------------------------------------------------------------------------------------------------------------
};

// url: http://stackoverflow.com/a/8752879
#include "subscriber_to_file.inl"

#endif // ROS_SUB_TO_FILE_H
