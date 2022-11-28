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
osrf_gear::LogicalCameraImage binImage1;
osrf_gear::LogicalCameraImage binImage2;
osrf_gear::LogicalCameraImage binImage3;
osrf_gear::LogicalCameraImage binImage4;
osrf_gear::LogicalCameraImage binImage5;
osrf_gear::LogicalCameraImage binImage6;
osrf_gear::LogicalCameraImage binImage7;
osrf_gear::LogicalCameraImage binImage8;
osrf_gear::LogicalCameraImage binImage9;
osrf_gear::LogicalCameraImage binImage10;
geometry_msgs::PoseStamped part_pose, goal_pose;
sensor_msgs::JointState joint_states;
trajectory_msgs::JointTrajectory joint_trajectory;
std::string cameraName = "None";
bool partFlag = false;
std::vector<int> validQs;
double  bestQ[6];
double q_sols[8][6];
bool jointStatesCalled = false;
int count = 0;

void t_matrix(double* matrix, double x, double y, double z) {
	
	*(matrix + 3) = x;
	*(matrix + 7) = y;
	*(matrix + 11) = z;
	
}
void camera_callback1(osrf_gear::LogicalCameraImage cameraResponse) {
		binImage1 == cameraResponse;
		partFlag = true;
		cameraName = "logical_camera_agv1_frame";
		//ROS_INFO_STREAM("Camera Response" << binImage1);
}

void camera_callback2(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage2 = cameraResponse;
	partFlag = true;
	cameraName = "logical_camera_agv2_frame";
	//ROS_INFO_STREAM("Camera Response" << binImage2);
}

void camera_callback3(osrf_gear::LogicalCameraImage cameraResponse) {

		binImage3 = cameraResponse;
		partFlag = true;
		cameraName = "logical_camera_bin1_frame";
		//ROS_INFO_STREAM("Camera Response" << binImage3);

}

void camera_callback4(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage4 = cameraResponse;
	cameraName = "logical_camera_bin2_frame";
	partFlag = true;
}

void camera_callback5(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage5 = cameraResponse;
	cameraName = "logical_camera_bin3_frame";
	partFlag = true;
}

void camera_callback6(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage6 = cameraResponse;
	cameraName = "logical_camera_bin4_frame";
	partFlag = true;
	//ROS_INFO_STREAM("Camera Response6" << binImage6);
}

void camera_callback7(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage7 = cameraResponse;
	cameraName = "logical_camera_bin5_frame";
	partFlag = true;
}

void camera_callback8(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage8 = cameraResponse;
	cameraName = "logical_camera_bin6_frame";
	partFlag = true;
}

void camera_callback9(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage9 = cameraResponse;
	cameraName = "quality_control_sensor_1_frame";
	partFlag = true;
}

void camera_callback10(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage10 = cameraResponse;
	cameraName = "quality_control_sensor_2_frame";
	partFlag = true;
}

void order_callback(const osrf_gear::Order in_order) {
	orders.push_back(in_order);
	ROS_INFO("%s\n", in_order.order_id.c_str());
	ROS_INFO_STREAM("order size in callback " <<orders.size());
}

void joint_callback(sensor_msgs::JointState current_joint) {
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
				if ((3.14 < q_sols[i][j]) && (q_sols[i][j] < 6.28)) {
					ROS_INFO_STREAM("Valid shoulder joint   " <<  q_sols[i][j]);
					validQs.push_back(i);
				}
			}
		}
	}
	for (int i : validQs) {
		ROS_INFO_STREAM("Valid q solutions " << i);
	}
}



int main(int argc, char** argv) {
	
	ros::init(argc, argv, "ariac_lab");
	ros::NodeHandle n;
	orders.clear();
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
	int service_call_succeeded;
	double q[] = { 3.14, -1.13, 1.51, 3.77, -1.51, 0 };
	double T_pose[4][4], T_des[4][4];
	trajectory_msgs::JointTrajectory desired;
	double q_pose[6], q_des[8][6];
	double T[4][4];
	ur_kinematics::forward(&q[0],  &T[0][0]);

	int num_sol = 0;
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
	materialLocationClient = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
	my_bool_var.request.data = true;
	tf2_ros::TransformListener tfListener(tfBuffer);
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
	ros::Subscriber agv1 = n.subscribe("/ariac/logical_camera_agv1", 3, camera_callback1);
	ros::Subscriber agv2 = n.subscribe("/ariac/logical_camera_agv2", 300, camera_callback2);
	ros::Subscriber bin1 = n.subscribe("/ariac/logical_camera_bin1", 300, camera_callback3);
	ros::Subscriber bin2 = n.subscribe("/ariac/logical_camera_bin2", 300, camera_callback4);
	ros::Subscriber bin3 = n.subscribe("/ariac/logical_camera_bin3", 300, camera_callback5);
	ros::Subscriber bin4 = n.subscribe("/ariac/logical_camera_bin4", 300, camera_callback6);
	ros::Subscriber bin5 = n.subscribe("/ariac/logical_camera_bin5", 300, camera_callback7);
	ros::Subscriber bin6 = n.subscribe("/ariac/logical_camera_bin6", 300, camera_callback8);
	ros::Subscriber faulty1 = n.subscribe("/ariac/quality_control_sensor_1", 300, camera_callback9);
	ros::Subscriber faulty2 = n.subscribe("/ariac/quality_control_sensor_2", 300, camera_callback10);
	ros::Subscriber orderSubscriber = n.subscribe<osrf_gear::Order>("/ariac/orders", 1, order_callback);
	ros::Subscriber jointState = n.subscribe<sensor_msgs::JointState>("/ariac/arm1/joint_states", 1, joint_callback);
	control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

	double  stand_in[4][4] = { {0.0, -1.0, 0.0, -100}, \
{0.0, 0.0, 1.0, -100}, \
{-1.0, 0.0, 0.0 , -100}, \
{0.0, 0.0, 0.0, 1.0} };
	double x = 3;
	double y = 4;
	double z = 5;
	t_matrix((double*)stand_in, x, y, z);

	ros::AsyncSpinner spinner(1);
	spinner.start();
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
		}
		//ur_kinematics::forward(&q_pose[0], &T[0][0]);
		
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

					for (osrf_gear::StorageUnit unit : materialLocation.response.storage_units) {
						ROS_INFO("made it inside for loop");
						ROS_INFO_STREAM("this storage unit is " << unit);
						binLocation = unit.unit_id;
						if (unit.unit_id == "bin1") {
							ros::Duration(1.0).sleep();
							/*
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							 */
							//ROS_WARN_STREAM("POSE IS " << binImage3);
							if (binImage3.models.size() > 0) {
								part_pose.pose = binImage3.models[0].pose;
							}
							
						}
						 if (unit.unit_id == "bin2") {
							ros::Duration(1.0).sleep();
							/*
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							*/
							//ROS_WARN_STREAM("POSE IS " << binImage4);
							if (binImage4.models.size() > 0) {
								part_pose.pose = binImage4.models[0].pose;
							}
							
						}
						if (unit.unit_id == "bin3") {
							ros::Duration(1.0).sleep();
							/* 
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage5);
							*/
							if (binImage5.models.size() > 0) {
								part_pose.pose = binImage5.models[0].pose;
							}
							
						}
						if (unit.unit_id == "bin4") {
							ros::Duration(1.0).sleep();
							/* 
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage6);
							*/
							if (binImage6.models.size() > 0) {
								part_pose.pose = binImage6.models[0].pose;
							}
							
						}
						if (unit.unit_id == "bin5") {
							ros::Duration(1.0).sleep();
							/*
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage7);
							*/
							if (binImage7.models.size() > 0) {
								part_pose.pose = binImage7.models[0].pose;
							}
						}
						if (unit.unit_id == "bin6") {
							ros::Duration(1.0).sleep();
							/*
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage8);
							 */
							if (binImage8.models.size() > 0) {
								part_pose.pose = binImage8.models[0].pose;
							}
							
						}
					ros::Duration(5.0).sleep();
					if (!(part_pose.pose.position.x == 0 && part_pose.pose.position.y == 0 && part_pose.pose.position.z == 0) ){
						ROS_ERROR("WE HAVE ENTERED THE PART FLAG CHECK");
						T_des[0][3] = part_pose.pose.position.x;
						T_des[1][3] = part_pose.pose.position.y;
						T_des[2][3] = part_pose.pose.position.z + 0.3; // above part
						T_des[3][3] = 1.0;
						T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
						T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
						T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
						T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;
						ROS_INFO_STREAM("PART POSE IS " << part_pose);


						ROS_INFO_STREAM("PART POSE  X IS " << T_des[0][3]);
						ROS_INFO_STREAM("PART POSE  Y IS " << T_des[1][3]);
						ROS_INFO_STREAM("PART POSE  Z IS " << T_des[2][3]);


						num_sol = ur_kinematics::inverse((double*)&T_des, (double*)&q_sols, 0.0);
					}
					if (num_sol != 0) {
						ROS_INFO_STREAM("Number of solutions " << num_sol);
						ROS_INFO_STREAM("The q sol " << q_sols[1][1]);
						best_q(num_sol);
						double best_solution[6];
						int best_location = validQs[0];
						ROS_INFO_STREAM("Best location index is " << best_location);
						for (int i = 0; i < 6; i++) {
							best_solution[i] = q_sols[best_location][i];
							ROS_INFO_STREAM("Best solution at position " << i << " is " << best_solution[i]);
						}
						goal_pose = part_pose;
						goal_pose.pose.position.z += 0.10; // 10 cm above the part
			// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
						goal_pose.pose.orientation.w = 0.707;
						goal_pose.pose.orientation.x = 0.0;
						goal_pose.pose.orientation.y = 0.707;
						goal_pose.pose.orientation.z = 0.0;
						geometry_msgs::TransformStamped tfStamped;
						tf2::doTransform(part_pose, goal_pose, tfStamped);
						//tf2_ros::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait_for_transform");
						try {
							tfStamped = tfBuffer.lookupTransform("arm1_base_link", cameraName, ros::Time::now(), ros::Duration(1.0));
							ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
						}
						catch (tf2::TransformException& ex) {
							ROS_ERROR("%s", ex.what());
						}
						joint_trajectory.header.seq = count++;
						joint_trajectory.header.stamp = ros::Time::now() + ros::Duration(1);
						joint_trajectory.header.frame_id = "/world";
						joint_trajectory_as.action_goal.header = joint_trajectory.header;
						joint_trajectory.joint_names.clear();
						joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
						joint_trajectory.joint_names.push_back("shoulder_pan_joint");
						joint_trajectory.joint_names.push_back("shoulder_lift_joint");
						joint_trajectory.joint_names.push_back("elbow_joint");
						joint_trajectory.joint_names.push_back("wrist_1_joint");
						joint_trajectory.joint_names.push_back("wrist_2_joint");
						joint_trajectory.joint_names.push_back("wrist_3_joint"); 
						joint_trajectory.points.resize(2);
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
						joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
						joint_trajectory.points[1].positions[0] = joint_states.position[1];
						for (int indy = 0; indy < 6; indy++) {
							joint_trajectory.points[1].positions[indy + 1] = best_solution[indy];
						}
						joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
						joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
						actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
						ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());


					}
					else {
						ROS_ERROR("THERE ARE ZERO SOLUTIONS");
					}
					
					}
				}
			}
			

			//To put the whole image message into a map, have ot make it not a constant pointer, copy to dynamic pointer
		
		


			// Create variables

			// Copy pose from the logical camera.
			}
			

		//ROS_INFO_STREAM(jointState);
	}

	return  0;
}
