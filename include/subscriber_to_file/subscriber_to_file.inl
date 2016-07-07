#ifndef ROS_SUB_TO_FILE_INL
#define ROS_SUB_TO_FILE_INL

#include <ros_macros.h>
#include <fstream>

// Note: template definition separate from implementation.
// To work, we use explicit instations of this template
// inside a .inl file (ros_subl_to_file.inl).
// -> A other solution could be to put the implementation directly
// into the header (declaration) (with a (other) inl file
// fill with the implementation of this template).
// Both solutions have pro/con.


//// Explicit intantiation of template
//#include "ros_boost_serialize_sensors.h"
//using namespace ros_boost_serialize::sensors;
//template class SubToFile< ::sensor_msgs::Imu, _sensor_msgs_serialize_type >;


#define DECL_TEMPLATE_SUBTOFILE_CONSTRUCTOR()  \
    template<typename TMsg, typename TMsgSerial>\
    SubToFile<TMsg, TMsgSerial>::SubToFile

#define DECL_TEMPLATE_SUBTOFILE(_return_typ, _member_name)  \
    template<typename TMsg, typename TMsgSerial>            \
    _return_typ SubToFile<TMsg, TMsgSerial>::_member_name


DECL_TEMPLATE_SUBTOFILE_CONSTRUCTOR() (
        const uint32_t& _size_buffer,
        ros::NodeHandle _priv_nh
        ) :
    v_size_buffer_(_size_buffer)
{
    v_msgs_.reserve(v_size_buffer_);

    GET_ROS_PARAM(binary, binary_, true, _priv_nh, nh_);
    GET_ROS_PARAM(xml, xml_, false, _priv_nh, nh_);
    GET_ROS_PARAM(sensors_topic, topic_name_, "/android/imu", _priv_nh, nh_);
}

DECL_TEMPLATE_SUBTOFILE(void, run)()
{
    // On lance le subscriber
    sub_ = nh_.subscribe(topic_name_, 1, &SubToFile::sub_cb, this);

    ROS_INFO ("Listening for incoming data on topic %s",
              nh_.resolveName (topic_name_).c_str ());
}

DECL_TEMPLATE_SUBTOFILE(bool, dump)(const std::string & _filename)
{
    bool retour = true;

    std::ofstream ofs(_filename, std::ios::out | std::ofstream::binary);
    if(ofs.is_open()) {
        // save data to archive
        try {
            if(xml_) {
                boost::archive::xml_oarchive oa(ofs);
                dump(oa);
            }
            else {
                if(binary_) {
                    boost::archive::binary_oarchive oa(ofs);
                    // write vector to archive
                    dump(oa);
                    // archive and stream closed when destructors are called
                }
                else {
                    boost::archive::text_oarchive oa(ofs);
                    // write vector to archive
                    dump(oa);
                    // archive and stream closed when destructors are called
                }
            }
        }
        catch(std::exception &exc){
            ROS_ERROR_STREAM("Exception lors de la serialization ! -> " << exc.what());
            retour = false;
        }

        ROS_INFO_STREAM("Write data into: " << _filename);
    }
    else {
        ROS_ERROR_STREAM("Probleme d'ouverture du stream: " << _filename);
        retour = false;
    }

    return retour;
}

DECL_TEMPLATE_SUBTOFILE(void, sub_cb)(const TMsgConstPtr & _msg)
{
    // Serialize the message
    TMsgSerial serial_msg(_msg);
    // Save it
    v_msgs_.push_back(serial_msg);

    // Time to write ?
    if (v_msgs_.size() == v_msgs_.capacity()) {
        // dump the vector messages into file
        dump(build_filename());
        // clear the vector
        v_msgs_.clear();
    }
}

#endif // ROS_SUB_TO_FILE_INL
