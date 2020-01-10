/* mtc_test.cpp

   Author: Abhinand A S
   Desc:   A demo to show MoveIt Task Constructor in action
*/

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace moveit::task_constructor;

Task createTask() {
	Task t;
	t.stages()->setName("MTC Test");

	const std::string group = "panda_arm";
	const std::string eef = "hand";
	
	// Planner initialization
	//Cartesian Interpolation
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	
	//Pipeline Planner
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();

	//Joint Interpolation
	auto joint_interpolation =std::make_shared<solvers::JointInterpolationPlanner>();

	//Load robot model
	t.loadRobotModel();
	
	t.setProperty("group", group);
	t.setProperty("eef", eef);

	/****************************************************
	 *                                                  *
	 *               Generator Stages                   *
	 *                                                  *
	 ***************************************************/
	
	//++++ Fixed state ++++

	// Stage* initial_stage = nullptr;
	// {
	// 	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	// 	auto& state = scene->getCurrentStateNonConst();
	// 	state.setToDefaultValues();
	// 	state.setToDefaultValues(state.getJointModelGroup(group), "ready");
	// 	state.update();
		
	// 	auto initial = std::make_unique<stages::FixedState>();
	// 	initial->setState(scene);
	// 	initial_stage = initial.get();
	// 	t.add(std::move(initial));
	// }

	//	or

    auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
   	{
       auto& state = scene->getCurrentStateNonConst();
       state.setToDefaultValues(state.getJointModelGroup(group), "ready");{}

       auto fixed = std::make_unique<stages::FixedState>("initial state");
       fixed->setState(scene);
       t.add(std::move(fixed));
    }

	//++++ Current State ++++

	// Stage* initial_stage = nullptr;
	// {
	// 	auto initial = std::make_unique<stages::CurrentState>("current state");
	// 	initial_stage = initial.get();
	// 	t.add(std::move(initial));
	// }

	/****************************************************
	 *                                                  *
	 *               Propagator Stages                  *
	 *                                                  *
	 ***************************************************/

	//++++ Move Relative ++++

	/**
	 * @brief Method 1: TwistStamped
	 * @brief This expresses velocity in free space broken into its linear and angular parts.
	 * @brief Vector3  linear (x, y, z)
	 * @brief Vector3  angular (x, y, z)
	 */

  //  	{
		// auto stage = std::make_unique<stages::MoveRelative>("x + 0.2", cartesian);
		// stage->setGroup(group);
		// geometry_msgs::TwistStamped direction;
		// direction.header.frame_id = "world";
		// direction.twist.linear.x = 0.2;
		// direction.twist.angular.x = 1.571;
		// stage->setDirection(direction);
		// t.add(std::move(stage));
  //   }

	/**
	 * @brief Method 2: Vector3Stamped
	 * @brief This represents a Vector3 with reference coordinate frame and timestamp.
	 * @brief It is only meant to represent a direction. Therefore, it does not
	 * @brief make sense to apply a translation to it.
	 * @brief Vector3  vector (x, y, z)
	 */

	// {
	// 	auto stage = std::make_unique<stages::MoveRelative>("x + 0.2", cartesian);
	// 	stage->setGroup(group);

	// 	geometry_msgs::Vector3Stamped direction;
	// 	direction.header.frame_id = "world";
	// 	direction.vector.x = 0.2;
	// 	stage->setDirection(direction);
	// 	t.add(std::move(stage));
 //    }

	/**
	 * @brief Method 3: std::map<std::string, double>& joint_deltas
	 * @brief This is used to control joints of manipulator.
	 * @brief cartesian or joint interpolation can be used.
	 */

	// {       
	// 	auto stage = std::make_unique<stages::MoveRelative>("x +0.2", joint_interpolation);
	// 	stage->setGroup(group);

	// 	std::map<std::string, double> joint_deltas;
	// 	joint_deltas = { { "panda_joint5", 1.5}, { "panda_joint7", 1.57} };			
	// 	stage->setDirection(joint_deltas);
	// 	t.add(std::move(stage));

	// }


	//++++ Move To ++++
	
	/**
	 * @brief Method 1: PoseStamped
	 * @brief A Pose with reference coordinate frame and timestamp.
	 * @brief A representation of pose in free space, composed of position and orientation.
	 * @brief Point  position (x, y, z)
	 * @brief Quaternion orientation (x, y, z, w)
	 */

	// {
	// 	auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
	// 	stage->setGroup(group);

	// 	geometry_msgs::PoseStamped pose;
	// 	pose.header.frame_id = "world";
	// 	pose.pose.position.x = 0.4;
	// 	pose.pose.position.y = 0.1;
	// 	pose.pose.position.z = 0.4;

	// 	tf2::Quaternion orientation;
	// 	orientation.setRPY(0, -M_PI , M_PI/1.5);
	// 	pose.pose.orientation = tf2::toMsg(orientation);
	// 	stage->setGoal(pose);
	// 	t.add(std::move(stage));
 //    }

	/**
	 * @brief Method 1: PointStamped
	 * @brief This represents a Point with reference coordinate frame and timestamp.
	 * @brief This contains the position of a point in free space. 
	 * @brief with same orientation.
	 * @brief Point  point (x, y, z)
	 */

    {
		auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
		stage->setGroup(group);

		geometry_msgs::PointStamped pose;
		pose.header.frame_id = "world";
		pose.point.x = 0.4;
		pose.point.z = 0.4;
		stage->setGoal(pose);
		t.add(std::move(stage));
   	}

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");

	ros::AsyncSpinner spinner(1);
	spinner.start();

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
