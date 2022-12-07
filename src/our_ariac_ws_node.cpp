#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include <vector>
#include "ros/ros.h"
#include <string>
//#include <ecse_373_ariac/osrf_gear/Order.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/LogicalCameraImage.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/JointState.h"
#include "ur_kinematics/ur_kin.h"
#include<array> 
#include <iostream>
#include <experimental/iterator>
#include "trajectory_msgs/JointTrajectory.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
tf2_ros::Buffer tfBuffer;
std_srvs::Trigger begin_comp;
std_srvs::SetBool my_bool_var;
osrf_gear::GetMaterialLocations materialLocation;
std::vector<osrf_gear::Order> orders;
ros::ServiceClient materialLocationClient;
std::string binLocation;
osrf_gear::LogicalCameraImage requestedBin;

std::vector<osrf_gear::LogicalCameraImage> bin_cams;
std::vector<osrf_gear::LogicalCameraImage> agv_cams;

geometry_msgs::PoseStamped part_pose, goal_pose, home_pose;
sensor_msgs::JointState joint_states;
sensor_msgs::JointState arm_home;
trajectory_msgs::JointTrajectory joint_trajectory;
std::string cameraName = "None";
bool partFlag = false;
std::vector<int> validQs;
double  bestQ[6];
double q_sols[8][6];
double best_solution[6];
bool jointStatesCalled = false;
int count = 0;

void t_matrix(double* matrix, double x, double y, double z) {

	*(matrix + 3) = x;
	*(matrix + 7) = y;
	*(matrix + 11) = z;

}

void camera_bins_callback(osrf_gear::LogicalCameraImage cameraResponse, int bin) {
	bin_cams[bin] = cameraResponse;
}

void camera_bin_1_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_bins_callback(cameraResponse, 0);
}

void camera_bin_2_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_bins_callback(cameraResponse, 1);
}

void camera_bin_3_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_bins_callback(cameraResponse, 2);
}

void camera_bin_4_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_bins_callback(cameraResponse, 3);
}

void camera_bin_5_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_bins_callback(cameraResponse, 4);
}

void camera_bin_6_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_bins_callback(cameraResponse, 5);
}

void camera_agvs_callback(osrf_gear::LogicalCameraImage cameraResponse, int agv) {
	agv_cams[agv] = cameraResponse;
}

void camera_agv_1_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_agvs_callback(cameraResponse, 0);
}

void camera_agv_2_callback(osrf_gear::LogicalCameraImage cameraResponse) {
	camera_agvs_callback(cameraResponse, 1);
}


void order_callback(const osrf_gear::Order in_order) {
	orders.push_back(in_order);
	ROS_INFO("%s\n", in_order.order_id.c_str());
	ROS_INFO_STREAM("order size in callback " << orders.size());
}

void joint_callback(sensor_msgs::JointState current_joint) {
	if (jointStatesCalled == false) {
		arm_home = current_joint;
	}
	jointStatesCalled = true;
	joint_states = current_joint;
}

void best_q(int numSols) {
	int q_sol_counter = 0;
	int which_array = 0;
	bool isSecondValid = false;
	for (int i = 0; i < numSols; i++) {
		for (int j = 0; j < 6; j++) {
			ROS_INFO_STREAM("Element at   " << i << " array  " << q_sols[i][j]);
			if (j == 1) {
				if ((3*M_PI/2< q_sols[i][j]) && (q_sols[i][j] < (2*M_PI))) {
					ROS_INFO_STREAM("Valid shoulder joint   " << q_sols[i][j]);
					validQs.push_back(i);
				}
			}
		}
	}
	for (int i : validQs) {
		ROS_INFO_STREAM("Valid q solutions " << i);

	}
}


void populateTrajectory(trajectory_msgs::JointTrajectory* p_joint_trajectory) {
	ROS_ERROR("inside the joint trajectory function");
	p_joint_trajectory->header.seq = count++;
	p_joint_trajectory->header.stamp = ros::Time::now() + ros::Duration(1);
	p_joint_trajectory->header.frame_id = "/world";
	//joint_trajectory_as.action_goal.header = p_joint_trajector->header;
	p_joint_trajectory->joint_names.clear();
	p_joint_trajectory->joint_names.push_back("linear_arm_actuator_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_pan_joint");
	p_joint_trajectory->joint_names.push_back("shoulder_lift_joint");
	p_joint_trajectory->joint_names.push_back("elbow_joint");
	p_joint_trajectory->joint_names.push_back("wrist_1_joint");
	p_joint_trajectory->joint_names.push_back("wrist_2_joint");
	p_joint_trajectory->joint_names.push_back("wrist_3_joint");
	p_joint_trajectory->points.resize(3);
	p_joint_trajectory->points[0].positions.resize(p_joint_trajectory->joint_names.size());
	for (int indy = 0; indy < p_joint_trajectory->joint_names.size(); indy++) {
		for (int indz = 0; indz < joint_states.name.size(); indz++) {
			if (p_joint_trajectory->joint_names[indy] == joint_states.name[indz]) {
				p_joint_trajectory->points[0].positions[indy] = joint_states.position[indz];
				break;
			}
		}
	}
	// When to start (immediately upon receipt).
	p_joint_trajectory->points[0].time_from_start = ros::Duration(4.0);
	p_joint_trajectory->points[1].positions.resize(p_joint_trajectory->joint_names.size());
	p_joint_trajectory->points[1].positions[0] = joint_states.position[1];
	for (int indy = 0; indy < 3; indy++) {
		p_joint_trajectory->points[1].positions[indy + 1] = best_solution[indy];
	}
	p_joint_trajectory->points[1].positions[4] = joint_states.position[3];
	p_joint_trajectory->points[1].positions[5] = joint_states.position[4];
	p_joint_trajectory->points[1].positions[6] = joint_states.position[5];

	p_joint_trajectory->points[1].time_from_start = ros::Duration(6.0);
	p_joint_trajectory->points[2].positions.resize(p_joint_trajectory->joint_names.size());
	
	p_joint_trajectory->points[2].positions[0] = joint_states.position[1];
	ROS_INFO_STREAM("Past where we first call second point");
	p_joint_trajectory->points[2].positions[1] = joint_states.position[0];
	p_joint_trajectory->points[2].positions[2] = joint_states.position[1];
	p_joint_trajectory->points[2].positions[3] = joint_states.position[2];
	for (int indy = 3; indy < 6; indy++) {
		p_joint_trajectory->points[2].positions[indy + 1] = best_solution[indy];
	}
	p_joint_trajectory->points[2].time_from_start = ros::Duration(7.0);

}


int main(int argc, char** argv) {

	ros::init(argc, argv, "ariac_lab");
	ros::NodeHandle n;
	orders.clear();
	int service_call_succeeded;
	double q[] = { 3.14, -1.13, 1.51, 3.77, -1.51, 0 };
	double T_pose[4][4], T_des[4][4];
	trajectory_msgs::JointTrajectory desired;
	double q_pose[6], q_des[8][6];
	double T[4][4];
	control_msgs::FollowJointTrajectoryAction joint_trajectory_as;


	bin_cams.resize(6);
	agv_cams.resize(2);


	int num_sol = 0;
	my_bool_var.request.data = true;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
	ros::Subscriber agv1 = n.subscribe("/ariac/logical_camera_agv1", 10, camera_agv_1_callback);
	ros::Subscriber agv2 = n.subscribe("/ariac/logical_camera_agv2", 10, camera_agv_2_callback);
	ros::Subscriber bin1 = n.subscribe("/ariac/logical_camera_bin1", 10, camera_bin_1_callback);
	ros::Subscriber bin2 = n.subscribe("/ariac/logical_camera_bin2", 10, camera_bin_2_callback);
	ros::Subscriber bin3 = n.subscribe("/ariac/logical_camera_bin3", 10, camera_bin_3_callback);
	ros::Subscriber bin4 = n.subscribe("/ariac/logical_camera_bin4", 10, camera_bin_4_callback);
	ros::Subscriber bin5 = n.subscribe("/ariac/logical_camera_bin5", 10, camera_bin_5_callback);
	ros::Subscriber bin6 = n.subscribe("/ariac/logical_camera_bin6", 10, camera_bin_6_callback);
	// ros::Subscriber faulty1 = n.subscribe("/ariac/quality_control_sensor_1", 300, camera_callback9);
	// ros::Subscriber faulty2 = n.subscribe("/ariac/quality_control_sensor_2", 300, camera_callback10);
	ros::Subscriber orderSubscriber = n.subscribe<osrf_gear::Order>("/ariac/orders", 1, order_callback);
	ros::Subscriber jointState = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1, joint_callback);

	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
	materialLocationClient = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	double  stand_in[4][4] = { {0.0, -1.0, 0.0, -100}, \
		{0.0, 0.0, 1.0, -100}, \
		{-1.0, 0.0, 0.0 , -100}, \
		{0.0, 0.0, 0.0, 1.0} };
	double x = 3;
	double y = 4;
	double z = 5;
	t_matrix((double*)stand_in, x, y, z);

	if (!begin_client.waitForExistence(ros::Duration(30.0))) {
		ROS_ERROR("Competition service unavailable");
		ros::shutdown();
		exit(0);
	}

	service_call_succeeded = begin_client.call(begin_comp);

	if (!service_call_succeeded) {
		ROS_ERROR("Competition service call failed! Oh me oh my");
	}

	if (begin_comp.response.success) {
		ROS_INFO("Competition service called successfully: %s", \
			begin_comp.response.message.c_str());
	}
	if (!begin_comp.response.success) {
		ROS_WARN("Competition service returned failure %s", begin_comp.response.message.c_str());
	}

	ros::Duration(1.0).sleep();

	while (ros::ok()) {
		if (jointStatesCalled) {
			q_pose[0] = joint_states.position[1];
			q_pose[1] = joint_states.position[2];
			q_pose[2] = joint_states.position[3];
			q_pose[3] = joint_states.position[4];
			q_pose[4] = joint_states.position[5];
			q_pose[5] = joint_states.position[6];
			
			for (auto element : q_pose) {
				ROS_INFO_STREAM("Q POSE IS " << element);
			}
			populateTrajectory(&joint_trajectory);
			joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
			actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
			ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
			
		}
		//ur_kinematics::forward(&q_pose[0], &T[0][0]);
	;
		if (orders.size() > 0) {
			ROS_INFO_STREAM("We have orders");
			for (osrf_gear::Shipment currentShip : orders[0].shipments) {
				ROS_INFO("Got a shipment");
				for (osrf_gear::Product currentProduct : currentShip.products) {
					ROS_INFO("Part Type Sent %s\n", currentProduct.type.c_str());
					//std::string gear_name = "Gear Part";
					materialLocation.request.material_type = currentProduct.type;
					//materialLocation.request.material_type = gear_name;
					ROS_INFO_STREAM("Service succeeeded" << materialLocationClient.call(materialLocation));
					ROS_INFO("Part type recieved %s\n", materialLocation.request.material_type.c_str());
					ROS_INFO_STREAM("See that we got a response\n" << materialLocation.response);
					ROS_INFO_STREAM(" the number of bins is " << materialLocation.response.storage_units.size());

					int bin_indx;
					std::string bin_name;
					for (osrf_gear::StorageUnit unit : materialLocation.response.storage_units) {
						ROS_INFO("made it inside for loop");
						ROS_INFO_STREAM("this storage unit is " << unit);
						binLocation = unit.unit_id;
						bin_name = unit.unit_id;
						if (unit.unit_id == "bin1")
							bin_indx = 0;
						if (unit.unit_id == "bin2")
							bin_indx = 1;
						if (unit.unit_id == "bin3")
							bin_indx = 2;
						if (unit.unit_id == "bin4")
							bin_indx = 3;
						if (unit.unit_id == "bin5")
							bin_indx = 4;
						if (unit.unit_id == "bin6")
							bin_indx = 5;
						if (unit.unit_id == "belt")
							continue;

						break;
					}
					ROS_INFO("Bin Assigned [%i]", bin_indx);
					part_pose.pose = bin_cams[bin_indx].models[0].pose;
					ROS_INFO_STREAM("PART POSE IS " << part_pose);

					geometry_msgs::TransformStamped tfStamped;
					//tf2_ros::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait_for_transform");
					try {
						tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_" + bin_name + "_frame", ros::Time(0.0), ros::Duration(1.0));
						ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
					}
					catch (tf2::TransformException& ex) {
						ROS_ERROR("%s", ex.what());
						continue;
					}
					ROS_INFO_STREAM("Transform is " << tfStamped);
					tf2::doTransform(part_pose, goal_pose, tfStamped);

					T_des[0][3] = goal_pose.pose.position.x;
					T_des[1][3] = goal_pose.pose.position.y;
					T_des[2][3] = goal_pose.pose.position.z + 0.3; // above part
					T_des[3][3] = 1.0;
					T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
					T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
					T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
					T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

					ROS_INFO("Getting IK");
					num_sol = ur_kinematics::inverse((double*)&T_des, (double*)&q_sols, 0.0);

					if (num_sol == 0) {
						ROS_ERROR("THERE ARE ZERO SOLUTIONS");
					}
					else {

						ROS_INFO_STREAM("Number of solutions " << num_sol);
						ROS_INFO_STREAM("The q sol " << q_sols[1][1]);
						best_q(num_sol);
						int best_location = validQs[0];
						ROS_INFO_STREAM("Best location index is " << best_location);
						for (int i = 0; i < 6; i++) {
							best_solution[i] = q_sols[best_location][i];
							ROS_INFO_STREAM("Best solution at position " << i << " is " << best_solution[i]);
						}
						goal_pose.pose.position.z += 0.10; // 10 cm above the part
			// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
						goal_pose.pose.orientation.w = 0.707;
						goal_pose.pose.orientation.x = 0.0;
						goal_pose.pose.orientation.y = 0.707;
						goal_pose.pose.orientation.z = 0.0;
						// from here

						// Here*/
						populateTrajectory(&joint_trajectory);
						joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
						actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
						ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
						

						best_solution[0] = 0;
						best_solution[1] = 3.14;
						best_solution[2] = 3.14;
						best_solution[3] = 3.26;
						best_solution[4] = 3.14;
						best_solution[5] = 0;

						populateTrajectory(&joint_trajectory);
						joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
						state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
						ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
						// When to start (immediately upon receipt).

						ros::Duration(10.0).sleep();
					}
				}
			}


			//To put the whole image message into a map, have ot make it not a constant pointer, copy to dynamic pointer



			// Create variables

			// Copy pose from the logical camera.
			ROS_INFO_STREAM("Erasing an element");
			orders.erase(orders.begin());
		}


		//ROS_INFO_STREAM(jointState);
	}

	return  0;
}
