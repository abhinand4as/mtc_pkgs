/* pick_place_tutorial2.cpp

   Author: Abhinand A S
   Desc:   A sample code to add Orientation constrain in pick
   Note: 
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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/compute_ik.h>
//TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace moveit::task_constructor;
float _x, _y, _z;

Eigen::Isometry3d transformToIsometry(const std::vector<float> &t) {      //<------------ convert pose into isometry -------->

	tf2::Quaternion q;
	q.setRPY(t[3], t[4], t[5]);
	
	return Eigen::Isometry3d(Eigen::Translation3d(t[0], t[1], t[2])
			* Eigen::Quaterniond(q.getW(), q.getX(), q.getY(), q.getZ()));
}

void spawnObject() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.7;
	o.primitive_poses[0].position.y = 0.18;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

Task createTask() {
	Task t;
	t.stages()->setName("Pick and place(Orientation constrain)");

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto cartesian = std::make_shared<solvers::CartesianPath>();

	const std::string group = "panda_arm";
	const std::string eef = "hand";
	t.setProperty("group", group);
	t.setProperty("eef", eef);

    t.loadRobotModel();

//******************* Define Constraints*****************************
    moveit_msgs::Constraints constraint;
	{
        tf2::Quaternion q;
        q.setRPY(0 , -M_PI / 2 , -M_PI);			//frame of panda_hand w.r.t panda_link0

        constraint.name = "dont_spill";
        constraint.orientation_constraints.resize(1);
        moveit_msgs::OrientationConstraint& c = constraint.orientation_constraints[0];
        c.header.frame_id = "panda_link0";
        c.link_name = "panda_hand";
		c.orientation = tf2::toMsg(q);
        c.absolute_x_axis_tolerance = M_PI;  		//tolerance of panda_hand frame
        c.absolute_y_axis_tolerance = 0.2;
        c.absolute_z_axis_tolerance = 0.2;
		c.weight = 1.0;
        
	}


	Stage* initial_stage = nullptr;
	{
	 auto initial = std::make_unique<stages::CurrentState>("current state");
		initial_stage = initial.get();
		t.add(std::move(initial));
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
		grasp_generator->setAngleDelta(M_PI / 2.);
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("close");
		grasp_generator->setMonitoredStage(initial_stage);
	
		auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
		std::vector<float> grasp_frame_transform = {0, 0, 0.1, 0.758, 1.571, 0};
	    grasp->setIKFrame(transformToIsometry(grasp_frame_transform), "panda_link8");
		grasp->setMaxIKSolutions(8);
	
		auto pick = std::make_unique<stages::Pick>(std::move(grasp));
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = "panda_link8";
		approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);
		
		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "world";
		lift.twist.linear.z = 1.0;
		pick->setLiftMotion(lift, 0.03, 0.05);

		t.add(std::move(pick));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x + 0.2", cartesian);
		stage->setGroup(group);

		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "panda_link0";
		direction.vector.z = 0.2;
		stage->setDirection(direction);
        initial_stage = stage.get();
		t.add(std::move(stage));
    }

//******************* Set Constraints*****************************
	{
		stages::Connect::GroupPlannerVector planners = { { "panda_arm", pipeline }, { "hand", pipeline } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
        connect->setTimeout(30.0);					//<---------Tune(increase this to better result)
		connect->setPathConstraints(constraint);    //<---------set constrain------------->
		t.add(std::move(connect));
	}

	{
		auto stage = std::make_unique<stages::GeneratePose>("generate pose");
		stage->properties().configureInitFrom(Stage::PARENT);      
        stage->setMonitoredStage(initial_stage);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "panda_link0";
        pose.pose.position.x = _x;//0.2;
        pose.pose.position.y = _y;//0.1;
        pose.pose.position.z = _z;//0.1;

        tf2::Quaternion q;
        q.setRPY(-M_PI / 2, -M_PI / 4 , -M_PI / 2);

        pose.pose.orientation = tf2::toMsg(q);
    
        stage->setPose(pose);

        auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
        wrapper->setMaxIKSolutions(1);
        wrapper->setIgnoreCollisions(true);
        wrapper->setIKFrame("panda_link8");
        wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		t.add(std::move(wrapper));
	}


	return t;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");

    _x = atof(argv[1]);
    _y = atof(argv[2]);
    _z = atof(argv[3]);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	spawnObject();

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
