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

std::queue<osrf_gear::Order> order_vector;
std::map<std::string, std::vector<osrf_gear::Model>> kits_map;

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

//Store the order to order_vector.
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
			}
		}
	}
}

int main(int argc, char **argv){
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
