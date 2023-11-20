#include <vector>
#include <queue>
#include <map>
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"

// Global count
int count;
// The received orders
std::queue<osrf_gear::Order> order_vector;
// The map of bin:kits
std::map<std::string, std::vector<osrf_gear::Model>> kits_map;
// The current state of joints of the robot
sensor_msgs::JointState joint_states;

//structure of pose of a kit, if_found shows whether the kit is found or not
struct kit_pose{
	bool if_found;
	std::string bin;
	geometry_msgs::Pose found_pose;
};

//Retrieve the transformation
geometry_msgs::TransformStamped transform(tf2_ros::Buffer& tfBuffer, std::string arm, std::string camera_frame){
	geometry_msgs::TransformStamped tfStamped;
	try {
		tfStamped = tfBuffer.lookupTransform(arm, camera_frame, ros::Time(0.0), ros::Duration(1.0));
		ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
	} catch (tf2::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
	}
	return tfStamped;
}

//Store the current joint states
void joint_callback(const sensor_msgs::JointState::ConstPtr& msg){
	joint_states = *msg;
}


//Store the order to order_vector
void order_callback(const osrf_gear::Order::ConstPtr& msg){
    ROS_INFO("%s Received!", msg->order_id.c_str());
    order_vector.push(*msg);
}

//Store the kits to kits_order
void camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, const std::string camera){
	//ROS_INFO("captured by camera: %s", camera.c_str());
	kits_map[camera]=msg->models;
	/*for (const auto &model : kits_map[camera]){
		ROS_INFO("several model!!!!!!!!!!!!!!!!!!!!!!!!!!%s", model.type.c_str());
	}*/
}

//Helper method to print out a Pose into a useful string.
void print_pose(const geometry_msgs::Pose pose){
    ROS_INFO("position xyz = (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("orientation wxyz = (%f, %f, %f, %f)", pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

//Find the kit
kit_pose find_kit(ros::NodeHandle n, const std::string product_type){
	//Initialize the kit
	kit_pose kit;
	kit.if_found=false;
	//Set the request
	ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	osrf_gear::GetMaterialLocations product_srv;
    product_srv.request.material_type = product_type;
    //Get the response and process
    if(material_location_client.call(product_srv)){
    	for(const auto &unit : product_srv.response.storage_units){
    		ROS_INFO("Found %s in %s!", product_type.c_str(), unit.unit_id.c_str());
    		if(unit.unit_id!="belt"){
    			for (const auto &model : kits_map[unit.unit_id]){
					ROS_INFO("several model!!!!!!!!!!!!%s", model.type.c_str());
					if (model.type==product_type){
						kit.if_found=true;
						kit.bin=unit.unit_id;
						kit.found_pose=model.pose;
						break;
					}
				}
				break;
    		}
    	}
    }
    return kit;
}

//Publish the trajectory messages
trajectory_msgs::JointTrajectory find_trajectory(const geometry_msgs::Pose desired){
	// Instantiate variables for use with the kinematic system.
	double T_pose[4][4], T_des[4][4];
	double q_pose[6], q_des[8][6];
	/****************/
	// Where is the end effector given the joint angles.
	// joint_states.position[0] is the linear_arm_actuator_joint
	q_pose[0] = joint_states.position[1];
	q_pose[1] = joint_states.position[2];
	q_pose[2] = joint_states.position[3];
	q_pose[3] = joint_states.position[4];
	q_pose[4] = joint_states.position[5];
	q_pose[5] = joint_states.position[6];
	ur_kinematics::forward(&q_pose[0], &T_pose[0][0]);
	/****************/
	// What joint angles put the end effector at a specific place.
	// Desired pose of the end effector wrt the base_link.
	T_des[0][3] = desired.position.x;
	T_des[1][3] = desired.position.y;
	T_des[2][3] = desired.position.z + 0.3; // above part
	T_des[3][3] = 1.0;
	// The orientation of the end effector so that the end effector is down.
	T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
	T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
	T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
	T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
	int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
	// Declare a variable for generating and publishing a trajectory.
	trajectory_msgs::JointTrajectory joint_trajectory;
	// Fill out the joint trajectory header.
	// Each joint trajectory should have an non-monotonically increasing sequence number.
	joint_trajectory.header.seq = count++;
	joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
	joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
	
	// Set the names of the joints being used. All must be present.
	joint_trajectory.joint_names.clear();
	joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
	joint_trajectory.joint_names.push_back("shoulder_pan_joint");
	joint_trajectory.joint_names.push_back("shoulder_lift_joint");
	joint_trajectory.joint_names.push_back("elbow_joint");
	joint_trajectory.joint_names.push_back("wrist_1_joint");
	joint_trajectory.joint_names.push_back("wrist_2_joint");
	joint_trajectory.joint_names.push_back("wrist_3_joint");
	// Set a start and end point.
	joint_trajectory.points.resize(2);
	// Set the start point to the current position of the joints from joint_states.
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
	// Must select which of the num_sols solutions to use. Just start with the first.
	int q_des_indx = 0;
	// Set the end point for the movement
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	// Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
	joint_trajectory.points[1].positions[0] = joint_states.position[1];
	// The actuators are commanded in an odd order, enter the joint positions in the correct positions
	for (int indy = 0; indy < 6; indy++) {
		joint_trajectory.points[1].positions[indy + 1] = q_des[q_des_indx][indy];
	}
	// How long to take for the movement.
	joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
	return joint_trajectory;
}


//Show the content of the order
void parse_order(ros::NodeHandle n, const osrf_gear::Order& order, tf2_ros::Buffer& tfBuffer){
	for (const auto &shipment : order.shipments){
		std::cout<<"---------------------------------------------------------------------"<<std::endl;
		ROS_INFO("shipment type: %s", shipment.shipment_type.c_str());
		ROS_INFO("agv_id: %s", shipment.agv_id.c_str());
		for (const auto &product : shipment.products){
			std::cout<<"----------------------need the following product---------------------"<<std::endl;
			ROS_INFO("type: %s", product.type.c_str());
			kit_pose kit=find_kit(n, product.type);
			if (kit.if_found){
				ROS_INFO("---------------kit pose from logical_camera_%s----------------", kit.bin.c_str());
				print_pose(kit.found_pose);
				//ROS_INFO("----------final pose-----------");
				//print_pose(product.pose);
				//get the retrieved pose
				geometry_msgs::TransformStamped transformStamped=transform(tfBuffer, "arm1_base_link", "logical_camera_"+kit.bin+"_frame");
				geometry_msgs::PoseStamped part_pose, goal_pose;
				// Copy pose from the logical camera.
				part_pose.pose = kit.found_pose;
				tf2::doTransform(part_pose, goal_pose, transformStamped);
				ROS_INFO("---------------kit pose to arm1_base_link----------------");
				print_pose(goal_pose.pose);
				auto goal_trajectory = find_trajectory(goal_pose.pose);
			}
		}
	}
}

int main(int argc, char **argv){
	//Initialize count
	count=0;

	//Initialize ros:
    ros::init(argc, argv, "ariac_comp_node");
    
	//Initialize node handle
    ros::NodeHandle n;

    //Create trigger for the beginning of the competition:
    std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    
    //Test whether the competition is successfully started
    bool service_call_succeeded;
    service_call_succeeded = begin_client.call(begin_comp);
    if (service_call_succeeded)
    {
        if (begin_comp.response.success)
        {
        	//Service was called and competition started successfully
            ROS_INFO("Start Success: %s", begin_comp.response.message.c_str());
        }
        else
        {
        	//Service was called, but could not be started
            ROS_WARN("Start Failure: %s", begin_comp.response.message.c_str());
        }
    }
    else
    {
        //Something went wrong calling the service itself
        ROS_ERROR("Competition service call failed!");
        ros::shutdown();
    }
    
    //Subscribe to incoming orders
    //order_vector.clear();
    ros::Subscriber sub_orders = n.subscribe("/ariac/orders", 10, order_callback);
    
    //Subscribe to cameras over product bins:
    //ITEMS:
    ros::Subscriber sub_bin1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 10, boost::bind(camera_callback, _1, "bin1"));
    ros::Subscriber sub_bin2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 10, boost::bind(camera_callback, _1, "bin2"));
    ros::Subscriber sub_bin3 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 10, boost::bind(camera_callback, _1, "bin3"));
    ros::Subscriber sub_bin4 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 10, boost::bind(camera_callback, _1, "bin4"));
    ros::Subscriber sub_bin5 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 10, boost::bind(camera_callback, _1, "bin5"));
    ros::Subscriber sub_bin6 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 10, boost::bind(camera_callback, _1, "bin6"));
    //AGV:
    ros::Subscriber sub_agv1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 10, boost::bind(camera_callback, _1, "agv1"));
    ros::Subscriber sub_agv2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 10, boost::bind(camera_callback, _1, "agv2"));
    //Quality Check:
    ros::Subscriber sub_qc1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 10, boost::bind(camera_callback, _1, "qc1"));
    ros::Subscriber sub_qc2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 10, boost::bind(camera_callback, _1, "qc2"));
    
    // Declare the transformation buffer to maintain a list of transformations
	tf2_ros::Buffer tfBuffer;
	// Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
	tf2_ros::TransformListener tfListener(tfBuffer);
	
	// The subscriber for receiving states of the joints
	ros::Subscriber joint_states_h = n.subscribe("ariac/arm1/joint_states", 10, joint_callback);
    
    //Set the frequency and start processing
    ros::Rate rate(10);
    while (ros::ok()){
    	if(order_vector.empty()){
    		ROS_WARN("No available order!");
    		ros::spinOnce();
    		rate.sleep();
    		continue;
    	}
    	//Give the information of the first order in order_vector
    	parse_order(n, order_vector.front(), tfBuffer);
    	order_vector.pop();
    	
    	//Wait for messages
    	ros::spinOnce();
    	rate.sleep();
    }

	return 0;
}
