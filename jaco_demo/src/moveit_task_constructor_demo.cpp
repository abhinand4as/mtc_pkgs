/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <jaco_demo/pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>

// Mesh
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_extents.h>

//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
float mesh_height = 0;
float pose[3];
ros::Subscriber box_pose_sub;

void poseCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	ROS_INFO("INSIDE CB");
	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
		pose[i] = *it;
	//	ROS_INFO("Position[%d]: [%f]",i, pose[i]);
		i++;
		
	}

	return;
}


double computeMeshHeight(const shape_msgs::Mesh& mesh) {
	double x,y,z;
	geometric_shapes::getShapeExtents(mesh, x, y, z);
	return z;
}

void collisionObjectFromResource(moveit_msgs::CollisionObject& msg, const std::string& id, const std::string& resource) {
	msg.meshes.resize(1);

	// load mesh
	const Eigen::Vector3d scaling(1, 1, 1);
	shapes::Shape* shape = shapes::createMeshFromResource(resource, scaling);
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(shape, shape_msg);
	msg.meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);

	// set pose
	msg.mesh_poses.resize(1);
	msg.mesh_poses[0].orientation.w = 1.0;

	// fill in details for MoveIt
	msg.id = id;
	msg.operation = moveit_msgs::CollisionObject::ADD;
}

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable() {
	ros::NodeHandle pnh("~");
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}
	
void createmesh() {
	const double table_height= 0;

	tf2::Quaternion orientation;
	orientation.setRPY(0, 0 , 3.14);

	geometry_msgs::PoseStamped bottle;
	bottle.header.frame_id= "world";
	bottle.pose.position.x= pose[0];//0.4;
	bottle.pose.position.y= pose[1];//0.15;
	bottle.pose.position.z= pose[2];//table_height;
	// bottle.pose.orientation= tf2::toMsg(orientation);
	bottle.pose.orientation= tf2::toMsg(orientation);


	moveit::planning_interface::PlanningSceneInterface psi;
	std::vector<moveit_msgs::CollisionObject> objects;
	// mtc_pour::setupObjects(objs, bottle, glass, "package://mtc_pour/meshes/small_bottle.stl");
	// const std::string bottle_mesh="package://jaco_demo/meshes/bottle.stl";
	const std::string bottle_mesh="package://jaco_demo/meshes/camera5.stl";

	{
		// add bottle
		objects.emplace_back();
		collisionObjectFromResource(
			objects.back(),
			"bottle",
			bottle_mesh);
		objects.back().header= bottle.header;
		objects.back().mesh_poses[0]= bottle.pose;

		// The input pose is interpreted as a point *on* the table
		mesh_height = computeMeshHeight(objects.back().meshes[0]);
		objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;
		ROS_INFO_STREAM("DEBUG: mesh height: "<< computeMeshHeight(objects.back().meshes[0]));
	}
	psi.applyCollisionObjects(objects);
}


int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init moveit_task_constructor_demo");
	ros::init(argc, argv, "moveit_task_constructor_demo");
	ros::NodeHandle nh;

	box_pose_sub = nh.subscribe("centroid", 10, poseCallback);

	ros::AsyncSpinner spinner(2);
	spinner.start();

	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");

	if (pnh.param("spawn_table", true))
		spawnObject(psi, createTable());
	// spawnObject(psi, createObject());
    // spawnObject(psi, createcube());

	do {
		ROS_INFO("INSIDE DO");
		ros::Duration(1.0).sleep();
	} while(pose[0] ==0 && pose[1] ==0 && pose[2] ==0 );

	ROS_INFO_STREAM("POSE X="<<pose[0]<<"\n    Y="<<pose[1]<<"\n    Z="<<pose[2]);

	createmesh();

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", nh);
	pick_place_task.mesh_height = mesh_height;
	pick_place_task.loadParameters();
	pick_place_task.init();
	ROS_INFO("DEBUG: INIT COMPLETED");
	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			pick_place_task.execute();
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	ROS_INFO_NAMED(LOGNAME, "Execution complete");

// Keep introspection alive
ros::waitForShutdown();
return 0;
}
