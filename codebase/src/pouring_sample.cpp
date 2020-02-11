/* pick_place_tutorial2.cpp

   Author: Abhinand A S
   Desc:   A sample code to show MTC pick and place
   Note: Check grasp frame transform when connection stage is not working.
*/

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace moveit::task_constructor;
float x,y,z;

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {
	
	x = 0.66;
	y = -0.45;
	z = 0.10;
	
	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = x;//0.45;
	o.primitive_poses[0].position.y = -0.18;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);

	moveit_msgs::CollisionObject g;
	g.id = "glass";
	g.header.frame_id = "world";
	g.primitive_poses.resize(1);
	g.primitive_poses[0].position.x = x;
	g.primitive_poses[0].position.y = y;
	g.primitive_poses[0].position.z = z;
	g.primitive_poses[0].orientation.w = 1.0;
	g.primitives.resize(1);
	g.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	g.primitives[0].dimensions.resize(2);
	g.primitives[0].dimensions[0] = 0.18;
	g.primitives[0].dimensions[1] = 0.042;
	psi.applyCollisionObject(g);
}

void spawnTable(moveit::planning_interface::PlanningSceneInterface& psi) {

	moveit_msgs::CollisionObject t;
	t.id = "table";
	t.header.frame_id = "world";
	t.primitive_poses.resize(1);
	t.primitive_poses[0].position.x = 0.55;
	t.primitive_poses[0].position.y = -0.3;
	t.primitive_poses[0].position.z = -0.025;
	t.primitive_poses[0].orientation.w = 1.0;
	t.primitives.resize(1);
	t.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	t.primitives[0].dimensions.resize(3);
	t.primitives[0].dimensions[0] = 0.4;
	t.primitives[0].dimensions[1] = 0.6;
	t.primitives[0].dimensions[2] = 0.05;
	psi.applyCollisionObject(t);
}

Task createTask() {
	Task t;
	t.stages()->setName("Pick and place 2");

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto cartesian = std::make_shared<solvers::CartesianPath>();

	auto joint_interpolation =std::make_shared<solvers::JointInterpolationPlanner>();

	const std::string group = "panda_arm";
	const std::string eef = "hand";
	t.setProperty("group", std::string("panda_arm"));

    Eigen::Isometry3d grasp_frame_transform_ = Eigen::Isometry3d::Identity();
    grasp_frame_transform_.translation() << 0.0, 0.0, 0.1;
	//grasp_frame_transform_.rotate(Eigen::AngleAxisf (0.5*M_PI, Eigen::Vector3f::UnitX()));

    tf2::Quaternion q;
    q.setRPY(0.785, 1.571, 0.0);

    Eigen::Quaterniond quat(q.getW(), q.getX(), q.getY(), q.getZ());
    grasp_frame_transform_.rotate(quat);

    t.loadRobotModel();

	Stage* initial_stage = nullptr;
	{
	 auto initial = std::make_unique<stages::CurrentState>("current state");
		initial_stage = initial.get();
		t.add(std::move(initial));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("move home", pipeline);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal("ready");
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

	{
		stages::Connect::GroupPlannerVector planners = { { "panda_arm", pipeline }, { "hand", pipeline } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));
	}



	{
		// grasp generator
		auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
		grasp_generator->setAngleDelta(M_PI / 10.);
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("close");
		grasp_generator->setMonitoredStage(initial_stage);
	
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
		grasp->setIKFrame(grasp_frame_transform_, "panda_link8");
		grasp->setMaxIKSolutions(8);
	
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "world";
		approach.twist.linear.x = 1.0;
		//approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.2);
		
		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "world";
		lift.twist.linear.z = 0.5;
		pick->setLiftMotion(lift, 0.2, 0.25);

		t.add(std::move(pick));
	}

	{
		auto stage = std::make_unique<stages::MoveTo>("moveto", cartesian);
		stage->setGroup(group);

		geometry_msgs::PointStamped pose;
		pose.header.frame_id = "world";
		pose.point.x = 0.56;
		pose.point.y = -0.35;
		pose.point.z = 0.37;
		stage->setGoal(pose);
		t.add(std::move(stage));
    }  

	{       
		auto stage = std::make_unique<stages::MoveRelative>("joint", joint_interpolation);
		stage->setGroup(group);

		std::map<std::string, double> joint_deltas;
		joint_deltas = { { "panda_joint7", 2.0} };			
		stage->setDirection(joint_deltas);
		t.add(std::move(stage));
		ros::WallDuration(0.1).sleep();

	}

	

	{       
		auto stage = std::make_unique<stages::MoveRelative>("joint", joint_interpolation);
		stage->setGroup(group);

		std::map<std::string, double> joint_deltas;
		joint_deltas = { { "panda_joint7", -2.0} };			
		stage->setDirection(joint_deltas);
		t.add(std::move(stage));

	}


	return t;
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "mtc_tutorial");

	moveit::planning_interface::PlanningSceneInterface psi;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	spawnObject(psi);
	spawnTable(psi);

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
