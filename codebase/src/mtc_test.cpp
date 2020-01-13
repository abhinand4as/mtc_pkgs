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
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace moveit::task_constructor;

void spawnObject() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = -0.2;
	o.primitive_poses[0].position.y = 0.13;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

//If generator stage is Fixed State, then it should be used following method.
void spawnObjectFixed(const planning_scene::PlanningScenePtr& scene) {
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.45;
	o.primitive_poses[0].position.y = 0.23;
	o.primitive_poses[0].position.z = 0.3;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	scene->processCollisionObjectMsg(o);
}

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
       spawnObjectFixed(scene);  

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

    //++++ Generate Grasp Pose ++++

	// {
	// 	// Sample grasp pose
	// 	auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
	// 	stage->properties().configureInitFrom(Stage::PARENT);
	// 	stage->properties().set("marker_ns", "grasp_pose");
	// 	stage->setPreGraspPose(hand_open_pose_);
	// 	stage->setObject(object);
	// 	stage->setAngleDelta(M_PI / 12);
	// 	stage->setMonitoredStage(current_state);  // Hook into current state

	// 	// Compute IK
	// 	auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
	// 	wrapper->setMaxIKSolutions(8);
	// 	wrapper->setMinSolutionDistance(1.0);
	// 	wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
	// 	wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
	// 	wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
	// 	grasp->insert(std::move(wrapper));
	// }

	// //OR

	// {
	// 	auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
	// 	gengrasp->properties().configureInitFrom(Stage::PARENT);
	// 	gengrasp->setPreGraspPose("open");
	// 	gengrasp->setObject("object");
	// 	gengrasp->setAngleDelta(M_PI / 10.);
	// 	gengrasp->setMonitoredStage(initial_stage);

	// 	auto filter = std::make_unique<stages::PredicateFilter>("filtered");
	// 	gengrasp->properties().exposeTo(filter->properties(), { "eef" });
	// 	filter->properties().configureInitFrom(Stage::PARENT);
	// 	filter->insert(std::move(gengrasp));
	// 	filter->setPredicate([](const SolutionBase& s, std::string& comment) {
	// 		bool accept = s.cost() < 2;
	// 		if (!accept)
	// 			comment += " (rejected)";
	// 		return accept;
	// 	});

	// 	auto ik = std::make_unique<stages::ComputeIK>("compute ik", std::move(filter));
	// 	PropertyMap& props = ik->properties();
	// 	props.configureInitFrom(Stage::PARENT, { "group", "eef", "default_pose" });
	// 	props.configureInitFrom(Stage::INTERFACE, { "target_pose" });  // derived from child's solution
	// 	ik->setIKFrame(Eigen::Translation3d(0, 0, .05) * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY()),
	// 	               "panda_link8");
	// 	ik->setMaxIKSolutions(1);
	// 	t.add(std::move(ik));
	// }

	// //OR

	// {
	// 	// grasp generator
	// 	auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
	// 	grasp_generator->setAngleDelta(.2);
	// 	grasp_generator->setPreGraspPose("open");
	// 	grasp_generator->setGraspPose("close");
	// 	grasp_generator->setMonitoredStage(initial_stage);

	// 	auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	// 	grasp->setIKFrame(Eigen::Translation3d(.03, 0, 0), "panda_link8");
	// 	grasp->setMaxIKSolutions(8);
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
	 * @brief Method 2: PointStamped
	 * @brief This represents a Point with reference coordinate frame and timestamp.
	 * @brief This contains the position of a point in free space. 
	 * @brief with same orientation.
	 * @brief Point  point (x, y, z)
	 */

  //   {
		// auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
		// stage->setGroup(group);

		// geometry_msgs::PointStamped pose;
		// pose.header.frame_id = "world";
		// pose.point.x = 0.4;
		// pose.point.z = 0.4;
		// stage->setGoal(pose);
		// t.add(std::move(stage));
  //  	}

	/**
	 * @brief Method 3:  named_joint_pose
	 * @brief This represents a pose defined in SRDF.
	 */


	// {
	// 	auto move = std::make_unique<stages::MoveTo>("Open gripper", pipeline);
	// 	move->restrictDirection(stages::MoveTo::FORWARD);
	// 	move->properties().property("group").configureInitFrom(Stage::PARENT, "eef");
	// 	move->setGoal("open");
	// 	t.add(std::move(move));
	// }

	/**
	 * @brief Method 4:  std::map<std::string, double>& joints
	 * @brief This represents a vector of joints and values.
	 */

	// {
	// 	auto stage = std::make_unique<stages::MoveTo>("joints", joint_interpolation);
	// 	std::map<std::string, double> joints;
	// 	joints["panda_joint7"] = 1.8;
	// 	stage->setGroup(group);
	// 	stage->setGoal(joints);
	// 	t.add(std::move(stage));
	// }

	//++++ Modify Planning Scene ++++

	/**
	 * @brief Method 1:  attachObject
	 * @brief attachObject(const std::string& object, const std::string& link);
	 * @brief 
	 */

	// {
	// 	auto move = std::make_unique<stages::ModifyPlanningScene>("attach object");
	// 	move->restrictDirection(stages::ModifyPlanningScene::FORWARD);
	// 	move->attachObject("object", "panda_link8");
	// 	t.add(std::move(move));
	// }

//to see the difference of attach and detach
  //   {
		// auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
		// stage->setGroup(group);

		// geometry_msgs::PointStamped pose;
		// pose.header.frame_id = "world";
		// pose.point.x = 0.5;
		// pose.point.z = 0.5;
		// stage->setGoal(pose);
		// t.add(std::move(stage));
  //  	}



	/**
	 * @brief Method 2:  detachObject
	 * @brief detachObject(const std::string& object, const std::string& link);
	 * @brief 
	 */

	// {
 //        auto stage = std::make_unique<stages::ModifyPlanningScene>("dettach object");
 //        stage->detachObject("object", "panda_link8");
 //        t.add(std::move(stage));
 //    }

	/**
	 * @brief Method 3:  allowCollisions
	 * @brief Normal method
	 * @brief allowCollisions(const Names& first, const Names& second, bool allow); 
	 * @brief allowCollisions(const std::string& first, const std::string& second, bool allow)
	 * @brief allowCollisions(const std::string& object, bool allow)
	 */

	// {
	// 	auto move = std::make_unique<stages::ModifyPlanningScene>("allow object collision");
	// 	move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

	// 	move->allowCollisions(
	// 	    "object", t.getRobotModel()->getJointModelGroup(eef)->getLinkModelNamesWithCollisionGeometry(), true);
	// 	t.add(std::move(move));
	// }




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
