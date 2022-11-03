#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include <vector>
#include "ros/ros.h"
//#include <ecse_373_ariac/osrf_gear/Order.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/GetMaterialLocations.h>
#include <osrf_gear/LogicalCameraImage.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
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

void camera_callback1(osrf_gear::LogicalCameraImage cameraResponse) {
		binImage1 == cameraResponse;
		//ROS_INFO_STREAM("Camera Response" << binImage1);
}

void camera_callback2(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage2 = cameraResponse;
	//ROS_INFO_STREAM("Camera Response" << binImage2);
}

void camera_callback3(osrf_gear::LogicalCameraImage cameraResponse) {

		binImage3 = cameraResponse;
		//ROS_INFO_STREAM("Camera Response" << binImage3);

}

void camera_callback4(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage4 = cameraResponse;
}

void camera_callback5(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage5 = cameraResponse;
}

void camera_callback6(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage6 = cameraResponse;
	//ROS_INFO_STREAM("Camera Response6" << binImage6);
}

void camera_callback7(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage7 = cameraResponse;
}

void camera_callback8(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage8 = cameraResponse;
}

void camera_callback9(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage9 = cameraResponse;
}

void camera_callback10(osrf_gear::LogicalCameraImage cameraResponse) {
	binImage10 = cameraResponse;
}

void order_callback(const osrf_gear::Order in_order) {
	orders.push_back(in_order);
	ROS_INFO("%s\n", in_order.order_id.c_str());
	ROS_INFO_STREAM("order size in callback " <<orders.size());
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "ariac_lab");
	ros::NodeHandle n;
	orders.clear();
	ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("ariac/start_competition");
	int service_call_succeeded;
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
	ros::Subscriber agv1 = n.subscribe("/ariac/logical_camera_agv1", 15, camera_callback1);
	ros::Subscriber agv2 = n.subscribe("/ariac/logical_camera_agv2", 15, camera_callback2);
	ros::Subscriber bin1 = n.subscribe("/ariac/logical_camera_bin1", 15, camera_callback3);
	ros::Subscriber bin2 = n.subscribe("/ariac/logical_camera_bin2", 15, camera_callback4);
	ros::Subscriber bin3 = n.subscribe("/ariac/logical_camera_bin3", 15, camera_callback5);
	ros::Subscriber bin4 = n.subscribe("/ariac/logical_camera_bin4", 15, camera_callback6);
	ros::Subscriber bin5 = n.subscribe("/ariac/logical_camera_bin5", 15, camera_callback7);
	ros::Subscriber bin6 = n.subscribe("/ariac/logical_camera_bin6", 15, camera_callback8);
	ros::Subscriber faulty1 = n.subscribe("/ariac/quality_control_sensor_1", 15, camera_callback9);
	ros::Subscriber faulty2 = n.subscribe("/ariac/quality_control_sensor_2", 15, camera_callback10);
	ros::Subscriber orderSubscriber = n.subscribe<osrf_gear::Order>("/ariac/orders", 1, order_callback);
	while (ros::ok()) {
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
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage3);
							if (binImage3.models.size() > 0) {
								part_pose.pose = binImage3.models[0].pose;
							}
							
						}
						 if (unit.unit_id == "bin2") {
							ros::Duration(1.0).sleep();
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage4);
							if (binImage4.models.size() > 0) {
								part_pose.pose = binImage4.models[0].pose;
							}
							
						}
						if (unit.unit_id == "bin3") {
							ros::Duration(1.0).sleep();
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage5);
							if (binImage5.models.size() > 0) {
								part_pose.pose = binImage5.models[0].pose;
							}
							
						}
						if (unit.unit_id == "bin4") {
							ros::Duration(1.0).sleep();
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage6);
							if (binImage6.models.size() > 0) {
								part_pose.pose = binImage6.models[0].pose;
							}
							
						}
						if (unit.unit_id == "bin5") {
							ros::Duration(1.0).sleep();
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage7);
							if (binImage7.models.size() > 0) {
								part_pose.pose = binImage7.models[0].pose;
							}
						}
						if (unit.unit_id == "bin6") {
							ros::Duration(1.0).sleep();
							ROS_INFO_STREAM("Part type is" << materialLocation.request.material_type.c_str());
							ROS_INFO_STREAM("BIN IS" << unit.unit_id);
							ROS_WARN_STREAM("POSE IS " << binImage8);
							if (binImage8.models.size() > 0) {
								part_pose.pose = binImage8.models[0].pose;
							}
							
						}
					}
				}
			}
			

			//To put the whole image message into a map, have ot make it not a constant pointer, copy to dynamic pointer
			geometry_msgs::TransformStamped tfStamped;
			try {
				tfStamped = tfBuffer.lookupTransform("arm1_base_frame", "logical_camera_frame", ros::Time(0.0), ros::Duration(1.0));
				ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
			}
			catch (tf2::TransformException& ex) {
				ROS_ERROR("%s", ex.what());
			}

			// tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent",
			//"how_long_to_wait_for_transform");

			// Create variables

			// Copy pose from the logical camera.
			goal_pose = part_pose;
			goal_pose.pose.position.z += 0.10; // 10 cm above the part
// Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
			goal_pose.pose.orientation.w = 0.707;
			goal_pose.pose.orientation.x = 0.0;
			goal_pose.pose.orientation.y = 0.707;
			goal_pose.pose.orientation.z = 0.0;

			tf2::doTransform(part_pose, goal_pose, tfStamped);
			}
			
		ros::Duration(5.0).sleep();
		ros::spinOnce();
	}

	return  0;
}
