#include "ros/ros.h"
#include "spawn_objects/spawn_objects.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SpawnModel.h"

#include <sstream>

// A lot of help from https://github.com/JenniferBuehler/gazebo-pkgs/blob/master/gazebo_test_tools/src/cube_spawner.cpp!

bool spawncube(float x, float y, float z, uint id)
{
    //This does not check if there is already a model present with the same name
    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = x;
    initial_pose.position.y = y;
    initial_pose.position.z = z;
    // initial_pose.orientation = ...;

    gazebo_msgs::SpawnModel model;
    model.request.model_name = "block_" + std::to_string(id);
    model.request.robot_namespace = "/blocks";
    model.request.initial_pose = initial_pose;
    model.request.reference_frame = "";

    float size[] = {0.07, 0.07, 0.07}; //m robotiq stroke is 0.085
    float mass = 1; //kg
    float inertia = mass*size[0]*size[0] / 0.6;
    std::string model_name = "my_model";

    std::stringstream _s;

    // http://gazebosim.org/tutorials?tut=build_model (size[1]/2.0)
    _s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<model_name<<"'>\
        <pose>0 0 "<<0<<" 0 0 0</pose>\
        <static>false</static>\
        <link name='link'>\
        <inertial>\
            <mass>"<<mass<<"</mass>\
            <inertia>\
            <ixx>"<<inertia<<"</ixx>\
            <ixy>0.0</ixy>\
            <ixz>0.0</ixz>\
            <iyy>"<<inertia<<"</iyy>\
            <iyz>0.0</iyz>\
            <izz>"<<inertia<<"</izz>\
            </inertia>\
        </inertial>\
        <collision name='collision'>\
            <geometry>\
            <box>\
                <size>"<<size[0]<<" "<<size[1]<<" "<<size[2]<<"</size>\
            </box>\
            </geometry>\
        </collision>\
        <visual name='visual'>\
            <geometry>\
            <box>\
                <size>"<<size[0]<<" "<<size[1]<<" "<<size[2]<<"</size>\
            </box>\
            </geometry>\
        </visual>\
        </link>\
    </model>\
    </sdf>";

    model.request.model_xml = _s.str();

    if (ros::service::call("/gazebo/spawn_sdf_model", model))
    {
        ROS_INFO("Success!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return false;
    }

    return true;
}

bool callback(spawn_objects::spawn_objects::Request &request, spawn_objects::spawn_objects::Response &response)
{
    return spawncube(0,-0.1,1.2,0) && spawncube(0,-0.2,1.2,1) && spawncube(0,-0.3,1.2,2) && spawncube(0,-0.4,1.2,3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawn_objects_server");
    ros::NodeHandle n;

    // Start client for gazebo model spawner
    // ros::ServiceClient client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

    // Start our own server
    ros::ServiceServer service = n.advertiseService("spawn_objects_service", callback);
    ROS_INFO("spawn_objects server started.");
    ros::spin();
    return 0;
}