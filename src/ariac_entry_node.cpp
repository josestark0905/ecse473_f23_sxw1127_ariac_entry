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
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"
#include "osrf_gear/SubmitShipment.h"
#include "osrf_gear/AGVControl.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

// Global count
int count_joint_trajectory;
int count_action_goal;
bool gripper_check;
// pointer to action_client
actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* trajectory_client;
// gripper client
ros::ServiceClient gripper_client;
// The received orders
std::queue <osrf_gear::Order> order_vector;
// The map of bin:kits
std::map <std::string, std::vector<osrf_gear::Model>> kits_map;
// The current state of joints of the robot
sensor_msgs::JointState joint_states;
//BIN LOCATIONS <ID, y location>
std::map<std::string, double> bin_pos = {
        {"bin4", 0.383},//0.383
        {"bin5", 1.15},//1.15
        {"bin6", 1.916},//1.4//1.15
        {"agv1", 2.2},
        {"agv2", -2.2}
};
//kit heights<name, heights>
std::map<std::string, double> kit_height = {
		{"gear_part", 0.015},
        {"piston_rod_part", 0.015}
};

//structure of pose of a kit, if_found shows whether the kit is found or not
struct kit_pose {
    bool if_found;
    std::string bin;
    geometry_msgs::Pose found_pose;
};

//Retrieve the transformation
geometry_msgs::TransformStamped transform(tf2_ros::Buffer &tfBuffer, std::string arm, std::string camera_frame) {
    geometry_msgs::TransformStamped tfStamped;
    try {
        tfStamped = tfBuffer.lookupTransform(arm, camera_frame, ros::Time(0.0), ros::Duration(1.0));
        ROS_INFO("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }
    return tfStamped;
}

//Get the actuator
double actuator_position(double position, std::string product_location) {
	if(product_location == "agv1" || product_location == "agv1"){
		return bin_pos[product_location];
	}
    double limit = 2.5;
    double offset = 0.72;//0.75
    return (position > 0) ? std::min(bin_pos[product_location] + offset + 0.08, limit) : std::max(
            bin_pos[product_location] - offset + 0.08, -1 * limit);
}

//Store the current joint states
void joint_callback(const sensor_msgs::JointState::ConstPtr &msg) {
    joint_states = *msg;
}


//Store the order to order_vector
void order_callback(const osrf_gear::Order::ConstPtr &msg) {
    ROS_INFO("%s Received!", msg->order_id.c_str());
    order_vector.push(*msg);
}

//Store the kits to kits_order
void camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr &msg, const std::string camera) {         
    kits_map[camera] = msg->models;
}

// check the status of vaccum gripper
void gripper_callback(const osrf_gear::VacuumGripperState::ConstPtr& msg){
	gripper_check = msg->attached;
}

//Print the pose
void print_pose(const geometry_msgs::Pose& pose) {
    ROS_INFO("position xyz = (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z);
    ROS_INFO("orientation wxyz = (%f, %f, %f, %f)", pose.orientation.w, pose.orientation.x, pose.orientation.y,
             pose.orientation.z);
}

//Print the trajectory
void print_trajectory(trajectory_msgs::JointTrajectory& goal_trajectory){
	std::cout<<"---------------------------------------------------------"<<std::endl;
	ROS_INFO("actuator:       %f", goal_trajectory.points[0].positions[0]);
    ROS_INFO("shoulder_pan:   %f", goal_trajectory.points[0].positions[1]);
    ROS_INFO("shoulder_lift:  %f", goal_trajectory.points[0].positions[2]);
    ROS_INFO("elbow:          %f", goal_trajectory.points[0].positions[3]);
    ROS_INFO("wrist_1:        %f", goal_trajectory.points[0].positions[4]);
    ROS_INFO("wrist_2:        %f", goal_trajectory.points[0].positions[5]);
    ROS_INFO("wrist_3:        %f", goal_trajectory.points[0].positions[6]);
    std::cout<<"---------------------------------------------------------"<<std::endl;
}

//Check whether the arm is moving
bool if_moving() {
    for (auto &vel: joint_states.velocity) {
        if (vel > 0) {
            return true;
        }
    }
    return false;
}

// turn the status of the gripper
void turn_gripper(bool status){
    osrf_gear::VacuumGripperControl srv;
    srv.request.enable = status;
    
    if(gripper_client.call(srv)) {
        while(!srv.response.success) {
            gripper_client.call(srv);
        }
        ROS_INFO("VACUUM %s", status ? "ENABLED" : "DISABLED");     
    }
    else {
        ROS_WARN("Could not reach the vacuum service.");
    }
}

//Find the kit
kit_pose find_kit(ros::NodeHandle& n, const std::string& product_type) {
    //Initialize the kit
    kit_pose kit;
    kit.if_found = false;
    //Set the request
    ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>(
            "/ariac/material_locations");
    //ensure that the server is successfully connected
    if (!ros::service::waitForService("/ariac/material_locations", ros::Duration(30.0))) {
    	ROS_ERROR("Material location service is not available after 30s.");
	}
    osrf_gear::GetMaterialLocations product_srv;
    product_srv.request.material_type = product_type;
    //Get the response and process
    if (material_location_client.call(product_srv)) {
        for (const auto &unit: product_srv.response.storage_units) {
            ROS_INFO("Found %s in %s!", product_type.c_str(), unit.unit_id.c_str());
            if (unit.unit_id != "belt") {
            	// read the camera again and again
            	while(kits_map[unit.unit_id].empty()){
            		ros::spinOnce();
            	}
            	
                for (const auto &model: kits_map[unit.unit_id]) {
                    ROS_INFO("several model!!!!!!!!!!!!%s", model.type.c_str());
                    if (model.type == product_type) {
                        kit.if_found = true;
                        kit.bin = unit.unit_id;
                        kit.found_pose = model.pose;
                        break;
                    }
                }
                break;
            }
        }
    }
    return kit;
}

// Filter out certain angles depending on where it is.
int optimal_solution_index(double possible_sol[8][6]) {
    for (int i = 0; i < 8; i++) {
        ROS_INFO("%f, %f, %f, %f, %f, %f", possible_sol[i][0], possible_sol[i][1], possible_sol[i][2],
                 possible_sol[i][3], possible_sol[i][4], possible_sol[i][5]);
    }
    double pi = 3.1415;
    for (int i = 0; i < 8; i++) {
        double shoulder_pan = possible_sol[i][0];
        double shoulder_lift = possible_sol[i][1];
        double elbow = possible_sol[i][2];
        double wrist1 = possible_sol[i][3];
        double wrist2 = possible_sol[i][4];
        double wrist3 = possible_sol[i][5];
        bool valid_s = false;
        //shoulder pan
        if (shoulder_pan < pi / 2 || shoulder_pan > 3 * pi / 2) {
            ROS_INFO("sp1");
            if (shoulder_lift < 3 * pi / 2 && elbow > pi) {
                ROS_INFO("sp2");
                if (wrist2 > pi) {
                    valid_s = true;
                }
            }
        } else {
        	ROS_INFO("sp3");
            if (shoulder_lift > 3 * pi / 2 && elbow < pi) {
                ROS_INFO("sp4");
                if (wrist2 > pi) {
                    valid_s = true;
                }
            }
        }
        if (valid_s) {
            return i;
        }
    }
    return -1;
}

//Publish the trajectory messages
trajectory_msgs::JointTrajectory find_trajectory(const geometry_msgs::Pose& camera_view, const std::string& bin_name, bool default_pose) {
    // Declare a variable for generating and publishing a trajectory.
    trajectory_msgs::JointTrajectory joint_trajectory;
    // Fill out the joint trajectory header.
    // Each joint trajectory should have an non-monotonically increasing sequence number.
    joint_trajectory.header.seq = count_joint_trajectory++;
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.

    // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.emplace_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_pan_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_lift_joint");
    joint_trajectory.joint_names.emplace_back("elbow_joint");
    joint_trajectory.joint_names.emplace_back("wrist_1_joint");
    joint_trajectory.joint_names.emplace_back("wrist_2_joint");
    joint_trajectory.joint_names.emplace_back("wrist_3_joint");
    // Set a start and end point.
    joint_trajectory.points.resize(1);
    // Set the start point to the current position of the joints from joint_states.
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    if(default_pose){
        joint_trajectory.points[0].positions[1] = 3.14;
        joint_trajectory.points[0].positions[2] = 3.14;
        joint_trajectory.points[0].positions[3] = 2.14;
        joint_trajectory.points[0].positions[4] = 3.27;
        joint_trajectory.points[0].positions[5] = 3.14;
        joint_trajectory.points[0].positions[6] = 0.0;
		joint_trajectory.points[0].positions[0] = joint_states.position[1];
		// When to start (immediately upon receipt).
    	joint_trajectory.points[0].time_from_start = ros::Duration(3.0);
    }else{
		/*for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
		    for (int indz = 0; indz < joint_states.name.size(); indz++) {
		        if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
		            joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
		            std::cout<<indy<<" "<< joint_states.position[indz]<<std::endl;
		            break;
		        }
		    }
		}*/
		joint_trajectory.points[0].positions[1] = 3.14;
        joint_trajectory.points[0].positions[2] = 3.14;
        joint_trajectory.points[0].positions[3] = 2.14;
        joint_trajectory.points[0].positions[4] = 3.27;
        joint_trajectory.points[0].positions[5] = 3.14;
        joint_trajectory.points[0].positions[6] = 0.0;
		joint_trajectory.points[0].positions[0] = actuator_position(camera_view.position.y, bin_name);
		// When to start (immediately upon receipt).
		double time = std::abs(joint_states.position[1] - joint_trajectory.points[0].positions[0]) / 0.5 + 0.05;
    	joint_trajectory.points[0].time_from_start = ros::Duration(time);
    }
    return joint_trajectory;
}

//Publish the trajectory messages
trajectory_msgs::JointTrajectory find_trajectory_kit(const geometry_msgs::Pose& desired, const double height, const double time) {
    // Instantiate variables for use with the kinematic system.
    double T_des[4][4];
    double q_des[8][6];
    /****************/
    // What joint angles put the end effector at a specific place.
    // Desired pose of the end effector wrt the base_link.
    T_des[0][3] = desired.position.x;
    T_des[1][3] = desired.position.y;
    T_des[2][3] = desired.position.z + height; // above part
    T_des[3][3] = 1.0;
    // The orientation of the end effector so that the end effector is down.
    T_des[0][0] = 0.0;
    T_des[0][1] = -1.0;
    T_des[0][2] = 0.0;
    T_des[1][0] = 0.0;
    T_des[1][1] = 0.0;
    T_des[1][2] = 1.0;
    T_des[2][0] = -1.0;
    T_des[2][1] = 0.0;
    T_des[2][2] = 0.0;
    T_des[3][0] = 0.0;
    T_des[3][1] = 0.0;
    T_des[3][2] = 0.0;
    int num_sols = ur_kinematics::inverse((double *) &T_des, (double *) &q_des, 0.0);
    // Must select which of the num_sols solutions to use. Just start with the first.
    int q_des_indx = 0;
    // Try to find wrist_2_joint of pi/2 and shoulder_lift_joint under pi/2
    q_des_indx = optimal_solution_index(q_des);
    // Declare a variable for generating and publishing a trajectory.
    trajectory_msgs::JointTrajectory joint_trajectory;
    // Fill out the joint trajectory header.
    // Each joint trajectory should have an non-monotonically increasing sequence number.
    joint_trajectory.header.seq = count_joint_trajectory++;
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.

    // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.emplace_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_pan_joint");
    joint_trajectory.joint_names.emplace_back("shoulder_lift_joint");
    joint_trajectory.joint_names.emplace_back("elbow_joint");
    joint_trajectory.joint_names.emplace_back("wrist_1_joint");
    joint_trajectory.joint_names.emplace_back("wrist_2_joint");
    joint_trajectory.joint_names.emplace_back("wrist_3_joint");
    // Set a start and end point.
    joint_trajectory.points.resize(1);
    // Set the end point for the movement
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) {
        joint_trajectory.points[0].positions[indy + 1] = q_des[q_des_indx][indy];
    }
    // The base stay
    joint_trajectory.points[0].positions[0] = joint_states.position[1];
    // Set the duration
    joint_trajectory.points[0].time_from_start = ros::Duration(time);
    ROS_INFO("GOT THE TRAJECTORY %i", num_sols);
    return joint_trajectory;
}

//publish method to operate the arm
void publish_mode(const trajectory_msgs::JointTrajectory& trajectory, const ros::Publisher& publisher){
	publisher.publish(trajectory);
    ROS_INFO("arm running...");
    double last = 0;
    while (if_moving() && ros::ok()) {
        if (joint_states.position[1] != last) {
        	publisher.publish(trajectory);
            ROS_INFO("%f", joint_states.position[1]);
            last = joint_states.position[1];
        }
    }
}

//action library method to operate the arm
void action_mode(trajectory_msgs::JointTrajectory& trajectory, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& client, bool force_move){
	control_msgs::FollowJointTrajectoryAction joint_trajectory;
	//fill out the information of trajectory and header
	joint_trajectory.action_goal.goal.trajectory = trajectory;
	//send the goal
	actionlib::SimpleClientGoalState act_state = client.sendGoalAndWait(joint_trajectory.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("Action Server returned with status [%i] %s", act_state.state_, act_state.toString().c_str());
    if(act_state.state_ != 6 && force_move){
    	ros::Duration(0.5).sleep();
    	trajectory.header.seq = count_joint_trajectory++;
    	trajectory.header.stamp = ros::Time::now();
    	action_mode(trajectory, client, force_move);
    }
    ROS_INFO("wait a moment...");
    ros::Duration(0.5).sleep();
}

//Show the content of the order
void parse_order(ros::NodeHandle n, const osrf_gear::Order &order, tf2_ros::Buffer &tfBuffer) {
    // The publisher for the trajectory
    ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 10);
    
    // parse shipments
    for (const auto &shipment: order.shipments) {
        std::cout << "---------------------------------------------------------------------" << std::endl;
        ROS_INFO("shipment type: %s", shipment.shipment_type.c_str());
        ROS_INFO("agv_id: %s", shipment.agv_id.c_str());
        for (const auto &product: shipment.products) {
            std::cout << "----------------------need the following product---------------------" << std::endl;
            ROS_INFO("type: %s", product.type.c_str());
            kit_pose kit = find_kit(n, product.type);
            if (kit.if_found) {
                ROS_INFO("---------------kit pose from logical_camera_%s----------------", kit.bin.c_str());
                print_pose(kit.found_pose);
                //move the actuator to the correct position
				auto move_base = find_trajectory(kit.found_pose, kit.bin, false);
                action_mode(move_base, *trajectory_client, true);
                
                //get the retrieved pose
                geometry_msgs::TransformStamped transformStamped = transform(tfBuffer, "arm1_base_link",
                                                                             "logical_camera_" + kit.bin + "_frame");
                geometry_msgs::PoseStamped part_pose, goal_pose;
                // Copy pose from the logical camera.
                part_pose.pose = kit.found_pose;
                tf2::doTransform(part_pose, goal_pose, transformStamped);
                ROS_INFO("---------------kit pose to arm1_base_link----------------");
                goal_pose.pose.orientation.w = 0.707;
                goal_pose.pose.orientation.x = 0.0;
                goal_pose.pose.orientation.y = 0.707;
                goal_pose.pose.orientation.z = 0.0;
                print_pose(goal_pose.pose);
                
                //find the above trajectory
                auto above_trajectory = find_trajectory_kit(goal_pose.pose, 0.13, 2);
                action_mode(above_trajectory, *trajectory_client, true);
                
                //pick up
                //turn on the gripper
                turn_gripper(true);
                auto goal_trajectory = find_trajectory_kit(goal_pose.pose, 0.015, 0.5);
                print_trajectory(goal_trajectory);
                action_mode(goal_trajectory, *trajectory_client, false);
                while(!gripper_check){
                //back to above trajectory
                auto back_above_trajectory = find_trajectory_kit(goal_pose.pose, 0.13, 0.5);
                action_mode(back_above_trajectory, *trajectory_client, true);
                goal_trajectory.header.seq = count_joint_trajectory++;
    			goal_trajectory.header.stamp = ros::Time::now();
                action_mode(goal_trajectory, *trajectory_client, false);
				}
				
                //publish_mode(goal_trajectory, trajectory_pub);
                auto back_pose = find_trajectory(goal_pose.pose, kit.bin, true);
                action_mode(back_pose, *trajectory_client, true);
                /*
                // put the part to certain agv
                std::string agv_name;
                if(shipment.agv_id == "any"){
                	agv_name = "agv1";
                }else{
                	agv_name = shipment.agv_id;
                }
                //move to agv
                move_base = find_trajectory(kit.found_pose, agv_name, false);
                action_mode(move_base, *trajectory_client, true);
                //get the retrieved pose
                transformStamped = transform(tfBuffer, "arm1_base_link", "logical_camera_" + agv_name + "_frame");
                // Copy pose from the logical camera.
                part_pose.pose = product.pose;
                tf2::doTransform(part_pose, goal_pose, transformStamped);
                ROS_INFO("---------------%s pose to arm1_base_link----------------", agv_name.c_str());
                goal_pose.pose.orientation.w = 0.707;
                goal_pose.pose.orientation.x = 0.0;
                goal_pose.pose.orientation.y = 0.707;
                goal_pose.pose.orientation.z = 0.0;
                auto prefix = find_trajectory(goal_pose.pose, kit.bin, true);
                prefix.points[0].positions[2] = 5;
                prefix.points[0].positions[3] = 0;
                action_mode(prefix, *trajectory_client, true);
                print_pose(goal_pose.pose);
                goal_trajectory = find_trajectory_kit(goal_pose.pose, 0.2, 0.5);
                if (goal_trajectory.points[0].positions[2] > 3.14) {
					goal_trajectory.points[0].positions[2] -= 6.28;
				}

				// Prevents excessive turning
				if (goal_trajectory.points[0].positions[3] > 3.14) {
					goal_trajectory.points[0].positions[3] -= 6.28;
				}
                print_trajectory(goal_trajectory);
                action_mode(goal_trajectory, *trajectory_client, true);*/
                
                //turn off the gripper
                turn_gripper(false);
            }
        }
    }
}

int main(int argc, char **argv) {
    //Initialize count
    count_joint_trajectory = 0;
    count_action_goal = 0;

    //Initialize ros:
    ros::init(argc, argv, "ariac_entry_node");

    //Initialize node handle
    ros::NodeHandle n;

    //Create trigger for the beginning of the competition:
    std_srvs::Trigger begin_comp;
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    //Test whether the competition is successfully started
    bool service_call_succeeded;
    service_call_succeeded = begin_client.call(begin_comp);
    if (service_call_succeeded) {
        if (begin_comp.response.success) {
            //Service was called and competition started successfully
            ROS_INFO("Start Success: %s", begin_comp.response.message.c_str());
        } else {
            //Service was called, but could not be started
            ROS_WARN("Start Failure: %s", begin_comp.response.message.c_str());
        }
    } else {
        //Something went wrong calling the service itself
        ROS_ERROR("Competition service call failed!");
        ros::shutdown();
    }
    
    // The action client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_action_client("/ariac/arm1/arm/follow_joint_trajectory", true);
    trajectory_action_client.waitForServer();
    ROS_INFO("Successfully connected to action server!");
    trajectory_client = &trajectory_action_client;
	
	// start gripper client
	gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("ariac/arm1/gripper/control");
	
    //Subscribe to incoming orders
    order_vector={};
    ros::Subscriber sub_orders = n.subscribe("/ariac/orders", 10, order_callback);
    
    // The subscriber for receiving states of the joints
    ros::Subscriber joint_states_h = n.subscribe("/ariac/arm1/joint_states", 10, joint_callback);
    
    // The subscriber for Vaccum State
    ros::Subscriber sub_vacuum = n.subscribe("/ariac/arm1/gripper/state", 10, gripper_callback);

    ros::AsyncSpinner spinner(3); // start 3 threads
    spinner.start();

    //Subscribe to cameras over product bins:
    //kits_map={{"bin1", {}}, {"bin2", {}}, {"bin3", {}}, {"bin4", {}}, {"bin5", {}}, {"bin6", {}}};
    //ITEMS:
    ros::Subscriber sub_bin1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin1", 10,
                                                                          boost::bind(camera_callback, _1, "bin1"));
    ros::Subscriber sub_bin2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin2", 10,
                                                                          boost::bind(camera_callback, _1, "bin2"));
    ros::Subscriber sub_bin3 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin3", 10,
                                                                          boost::bind(camera_callback, _1, "bin3"));
    ros::Subscriber sub_bin4 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin4", 10,
                                                                          boost::bind(camera_callback, _1, "bin4"));
    ros::Subscriber sub_bin5 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin5", 10,
                                                                          boost::bind(camera_callback, _1, "bin5"));
    ros::Subscriber sub_bin6 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_bin6", 10,
                                                                          boost::bind(camera_callback, _1, "bin6"));
    //AGV:
    ros::Subscriber sub_agv1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 10,
                                                                          boost::bind(camera_callback, _1, "agv1"));
    ros::Subscriber sub_agv2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 10,
                                                                          boost::bind(camera_callback, _1, "agv2"));
    //Quality Check:
    ros::Subscriber sub_qc1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 10,
                                                                         boost::bind(camera_callback, _1, "qc1"));
    ros::Subscriber sub_qc2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 10,
                                                                         boost::bind(camera_callback, _1, "qc2"));

    // Declare the transformation buffer to maintain a list of transformations
    tf2_ros::Buffer tfBuffer;
    // Instantiate a listener that listens to the tf and tf_static topics and to update the buffer.
    tf2_ros::TransformListener tfListener(tfBuffer);

    //Set the frequency and start processing
    ros::Rate rate(10);
    while (ros::ok()) {
    	ros::spinOnce();
        if (order_vector.empty()) {
            ROS_WARN("No available order!");
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        //Give the information of the first order in order_vector
        parse_order(n, order_vector.front(), tfBuffer);
        order_vector.pop();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
