#include <ros/ros.h>
#include "gps_common/GPSFix.h"
#include "sensor_msgs/NavSatFix.h"

void fixExtendedCallback(const gps_common::GPSFix::ConstPtr& msg);
void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

gps_common::GPSFix fix_extended_msg;

ros::Publisher *merged_fix_pub_ptr;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "xsens_gps_msg_merger_node");

    ROS_INFO("xsens_gps_msg_merger_node for ROS v0.1");

    ros::NodeHandle pn("~");
    // subscribers and publishers
    ros::Subscriber ext_sub = pn.subscribe<gps_common::GPSFix>("/fix_extended", 10, fixExtendedCallback);
    ros::Subscriber fix_sub = pn.subscribe<sensor_msgs::NavSatFix>("/fix", 10, fixCallback);
    ros::Publisher merged_fix_pub = pn.advertise<sensor_msgs::NavSatFix>("/fix_merged", 10);

    merged_fix_pub_ptr = &merged_fix_pub;

    ros::spin();

    return 0;
}

void fixExtendedCallback(const gps_common::GPSFix::ConstPtr& msg)
{
    fix_extended_msg = *msg;
}

void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{

    static double err_h = 0, err_v = 0;
    sensor_msgs::NavSatFix msg_merged = *msg;

    if(fix_extended_msg.err_horz != 0)
    {
        err_h = fix_extended_msg.err_horz;
    }
    if(fix_extended_msg.err_vert != 0)
    {
        err_v = fix_extended_msg.err_vert;
    }

    msg_merged.position_covariance[0] = err_h;
    msg_merged.position_covariance[4] = err_h;
    msg_merged.position_covariance[8] = err_v;

    merged_fix_pub_ptr->publish(msg_merged);
}
