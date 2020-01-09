/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
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

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
*/

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <tf2/LinearMath/Quaternion.h>
using namespace moveit::task_constructor;
float x, y;
Task createTask() {
        Task t;
        t.stages()->setName("Cartesian Path");

        const std::string group = "panda_arm";
        const std::string hand = "hand";
	const std::string object = "object";
	
	Eigen::Isometry3d grasp_frame_transform_ = Eigen::Isometry3d::Identity();
	grasp_frame_transform_.translation() << 0.0, 0.0, 0.1;
//	grasp_frame_transform_.rotate(Eigen::AngleAxisf (0.5*M_PI, Eigen::Vector3f::UnitX()));
	
	tf2::Quaternion q;
	q.setRPY(x, y, 0.0);

	Eigen::Quaterniond quat(q.getW(), q.getX(), q.getY(), q.getZ());
	grasp_frame_transform_.rotate(quat);

	ROS_INFO_STREAM("ROT : "<< grasp_frame_transform_.rotation());
        // create Cartesian interpolation "planner" to be used in stages
        auto cartesian = std::make_shared<solvers::CartesianPath>();

	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);      

        auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

        // start from a fixed robot state
        t.loadRobotModel();


	t.setProperty("group", std::string("panda_arm"));
	t.setProperty("eef", std::string("hand"));


	Stage* initial_stage = nullptr;
	{
		auto initial = std::make_unique<stages::CurrentState>("current state");
		initial_stage = initial.get();
		t.add(std::move(initial));
	}


//Generator Stage : Fixed State

//        auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
//        {
//                auto& state = scene->getCurrentStateNonConst();
//                state.setToDefaultValues(state.getJointModelGroup(group), "ready");
//
//                auto fixed = std::make_unique<stages::FixedState>("initial state");
//                fixed->setState(scene);
//                t.add(std::move(fixed));
//        }

//Propagtor Stage : Move Relative
//	{
//                auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
//                stage->setGroup(group);
        

//                geometry_msgs::Vector3Stamped direction;
//                direction.header.frame_id = "world";
//                direction.vector.x = 0.2;
//		direction.vector.y = 0.1;
//		direction.vector.z = 0.05;
//                stage->setDirection(direction);
//                t.add(std::move(stage));
//        }

//        {
//                auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
//                stage->setGroup(group);
//             	geometry_msgs::TwistStamped direction;
//                direction.header.frame_id = "world";
//                direction.twist.linear.x = 0.2;
//               // direction.twist.linear.y = 0.1;
//               // direction.twist.linear.z = 0.05;
//               direction.twist.angular.x = 3,14;
//		direction.twist.angular.y = 1.57;
//	        stage->setDirection(direction);
//                t.add(std::move(stage));
//        }
//
//
//        {
//                auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
//                stage->setGroup(group);
//                geometry_msgs::TwistStamped direction;
//                direction.header.frame_id = "world";
//                direction.twist.linear.x = 0.2;
//               // direction.twist.linear.y = 0.1;
//               // direction.twist.linear.z = 0.05;
//               direction.twist.angular.x = 3,14;
//                direction.twist.angular.y = 1.57;
//                stage->setDirection(direction);
//                t.add(std::move(stage));
//        }

//	{       
//  		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
//                stage->setGroup(group); 
//                std::map<std::string, double> joint_deltas;
//		joint_deltas = {{ "panda_joint7", 1.5}};			
//                stage->setDirection(joint_deltas);
//		t.add(std::move(stage));
//
//	}
	
//	{
//		auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
//		stage->setGroup(group);
//		geometry_msgs::PoseStamped pose;
//		pose.header.frame_id = "world";
//		pose.pose.position.x = 0.4;
//		pose.pose.position.y = 0.1;
//		pose.pose.position.z = 0.4;
//		pose.pose.orientation.x = 0.923956;
//		pose.pose.orientation.y = -0.382499;
//		pose.pose.orientation.z = 0.0;
//	        pose.pose.orientation.w = 0.0;
//		stage->setGoal(pose);
//		t.add(std::move(stage));
//	}
//
//        {
//                auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
//                stage->setGroup(group);
//                geometry_msgs::PointStamped pose;
//                pose.header.frame_id = "world";
//                pose.point.x = 0.25;
//                pose.point.y = 0.2;
//                pose.point.z = 0.4;
////                pose.pose.orientation.x = 0.923956;
////                pose.pose.orientation.y = -0.382499;
////                pose.pose.orientation.z = 0.0;
////                pose.pose.orientation.w = 0.0;
//                stage->setGoal(pose);
//                t.add(std::move(stage));
//        }
	
//	{
//		auto stage = std::make_unique<stages::MoveTo>("home", joint_interpolation);
//		stage->setGroup(group);
//		stage->setGoal("ready");
//		t.add(std::move(stage));
//	}
	
//	{
//		auto stage = std::make_unique<stages::MoveTo>("joints", joint_interpolation);
//		std::map<std::string, double> joints;
//		joints["panda_joint7"] = 1.8;
//		stage->setGroup(group);
//		stage->setGoal(joints);
//		t.add(std::move(stage));
//	}
	
//	Stage* attach_object_stage = nullptr;
//        {
//       		auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
//       		stage->attachObject("target", "panda_link8");
//       		attach_object_stage = stage.get();
//       		t.add(std::move(stage));
//        }
//
//               {
//                auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
//                stage->setGroup(group);
//                geometry_msgs::PoseStamped pose;
//                pose.header.frame_id = "world";
//                pose.pose.position.x = 0.4;
//                pose.pose.position.y = 0.1;
//                pose.pose.position.z = 0.4;
//                pose.pose.orientation.x = 0.923956;
//                pose.pose.orientation.y = -0.382499;
//                pose.pose.orientation.z = 0.0;
//                pose.pose.orientation.w = 0.0;
//                stage->setGoal(pose);
//                t.add(std::move(stage));
//        }
//
//
//	{
//		auto stage = std::make_unique<stages::ModifyPlanningScene>("dettach object");
//		stage->detachObject("target", "panda_link8");
//		t.add(std::move(stage));
//	}

 	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("allow object collision");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

		move->allowCollisions(
		    "target", t.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(move));
	}

       

	{
		stages::Connect::GroupPlannerVector planners = { { "panda_arm", sampling_planner }, { "hand", sampling_planner } };
		auto stage = std::make_unique<stages::Connect>(
		    "connect", planners);
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

//	{
//		auto move = std::make_unique<stages::MoveRelative>("approach object", cartesian);
//		move->restrictDirection(stages::MoveRelative::BACKWARD);
//		move->properties().set("link", "panda_link8");
//		move->properties().configureInitFrom(Stage::PARENT, { "group" });
//		move->properties().set("marker_ns", std::string("approach"));
//		move->setIKFrame("panda_link8");
//		move->setMinMaxDistance(0.05, 0.1);
//
//		geometry_msgs::Vector3Stamped direction;
//		direction.header.frame_id = "panda_link8";
//		direction.vector.z = 1;
//		move->setDirection(direction);
//		t.add(std::move(move));
//	}
//
//	{
//                std::vector<std::string> Names1 = {"panda_leftfinger", "panda_rightfinger", "panda_hand"};
//                std::vector<std::string> Names2 = {"target"};
//                auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,hand)");
//                stage->allowCollisions(Names1, Names2, true);
//                t.add(std::move(stage));
//        }
	{
		auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		stage->setEndEffector("hand");
		stage->setPreGraspPose("open");
		stage->setAngleDelta(.2);
		stage->setObject("target");
//		stage->setGraspPose("close");
		stage->setMonitoredStage(initial_stage);
// Compute IK
		auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
		wrapper->setMaxIKSolutions(1);
		wrapper->setMinSolutionDistance(1.0);
		wrapper->setIKFrame( grasp_frame_transform_, "panda_link8");
		wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}


 	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("allow object collision");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

		move->allowCollisions(
		    "target", t.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(move));
	}


	{
		auto move = std::make_unique<stages::MoveTo>("close gripper", sampling_planner);
//		move->restrictDirection(stages::MoveTo::FORWARD);
		move->properties().property("group").configureInitFrom(Stage::PARENT, "eef");
		move->setGoal("close");
		t.add(std::move(move));
	}	

      Stage* attach_object_stage = nullptr;
        {
                      auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
                      stage->attachObject("target", "panda_link8");
                      attach_object_stage = stage.get();
                      t.add(std::move(stage));
        }

      {
              auto stage = std::make_unique<stages::MoveTo>("home", joint_interpolation);
              stage->setGroup(group);
              stage->setGoal("ready");
              t.add(std::move(stage));
      }

      {
              auto stage = std::make_unique<stages::ModifyPlanningScene>("dettach object");
              stage->detachObject("target", "panda_link8");
              t.add(std::move(stage));
      }

        {
                auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
                stage->setGroup(group);
                geometry_msgs::TwistStamped direction;
                direction.header.frame_id = "world";
                direction.twist.linear.y = 0.2;
               // direction.twist.linear.y = 0.1;
               // direction.twist.linear.z = 0.05;
              // direction.twist.angular.x = 3,14;
               // direction.twist.angular.y = 1.57;
                stage->setDirection(direction);
                t.add(std::move(stage));
        }
	
	return t;
}

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
	ROS_INFO("APPLIED COLLISION OBJECT");
}


moveit_msgs::CollisionObject createObject() {
	ros::NodeHandle pnh("~");
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions = {0.25, 0.02};
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	

	pose.position.x = 0.4;
	pose.position.y = 0.0;
	pose.position.z = 0.38;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	moveit_msgs::CollisionObject object;
	object.id = "target";
	object.header.frame_id = "world";
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	object.primitive_poses.push_back(pose);




	return object;
}



int main(int argc, char** argv) {
        ros::init(argc, argv, "mtc_tutorial");
        // run an asynchronous spinner to communicate with the move_group node and rviz
	ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);
        spinner.start();
	
	x = atof(argv[1]);
	y = atof(argv[2]);
	
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;	
	ros::NodeHandle pnh("~");
	spawnObject(psi, createObject());
        auto task = createTask();
                                                                                                                             
        try {
                if (task.plan())
                        task.introspection().publishSolution(*task.solutions().front());
        } catch (const InitStageException& ex) {
                std::cerr << "planning failed with exception" << std::endl << ex << task;
        }

        ros::waitForShutdown();  // keep alive for interactive inspection in rviz
        return 0;
}

