/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Joao Sousa on 18/04/2014
*********************************************************************/
#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

std::string kml_filename, coordinates_utm_filename;
std::ofstream coordinates_utm_file, kml_file;
std::ifstream footer_kml;

void callbackFix(const sensor_msgs::NavSatFixConstPtr& fix);
void callbackOdom(const nav_msgs::OdometryConstPtr& odom);
void sigintHandler(int a);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kml_extractor_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    signal(SIGINT, sigintHandler);

    std::string header_file_path, footer_file_path;
    pn.param<std::string>("kml_file", kml_filename, "out.kml");
    pn.param<std::string>("coordinates_filename", coordinates_utm_filename, "coordinates_utm.txt");
    pn.param<std::string>("header_file_path", header_file_path, "/home/joao/catkin_ws_isrobotcar/src/kml_extractor/src/header.yaml");
    pn.param<std::string>("footer_file_path", footer_file_path, "/home/joao/catkin_ws_isrobotcar/src/kml_extractor/src/footer.yaml");

    kml_file.open(kml_filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    coordinates_utm_file.open(coordinates_utm_filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    std::ifstream header_kml(header_file_path.c_str());
    footer_kml.open(footer_file_path.c_str(), std::ios_base::in);

    while(header_kml.good())
    {
        char c = header_kml.get();       // get character from file
            if (header_kml.good())
             kml_file << c;
    }
    kml_file << std::endl;

    ros::Subscriber fix_sub = n.subscribe("fix", 30, callbackFix);
    ros::Subscriber odom_sub = n.subscribe("odom", 30, callbackOdom);

    ros::spin();

    return 0;
}


void callbackFix(const sensor_msgs::NavSatFixConstPtr& fix)
{
    kml_file << std::setprecision(12) << fix->longitude  << "," << fix->latitude << "," << fix->altitude << ",";
}

void callbackOdom(const nav_msgs::OdometryConstPtr& odom)
{
    coordinates_utm_file << std::setprecision(12) << odom->pose.pose.position.x;
    coordinates_utm_file << ",";
    coordinates_utm_file << odom->pose.pose.position.y;
    coordinates_utm_file << ",";
    coordinates_utm_file << odom->pose.pose.position.z;
    coordinates_utm_file << std::endl;
}

void sigintHandler(int a)
{
    while(footer_kml.good())
    {
        char c = footer_kml.get();       // get character from file
            if (footer_kml.good())
             kml_file << c;
    }
    kml_file << std::endl;

    ros::requestShutdown();
}
