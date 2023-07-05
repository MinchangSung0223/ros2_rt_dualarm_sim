/*
 * RTIndy7Client.cpp
 *
 *  Created on: 2023. 06. 06.
 *      Author: Sunhong Kim
 */


#ifndef __XENO__
#define __XENO__
#endif
#include "RTIndy7Client.h"
#include "MR_Indy7.h"
#include "MR_DualArm.h"
#include "MR_Indy7_DualArm.h"
#include "modern_robotics_relative.h"
#include "modern_robotics.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/path.hpp"

#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include "Utils/b3Clock.h"
#include "Indy7.h"
#include "DualArm.h"

#include "MR_Trajectory.h"

//
JointInfo right_info;
JointInfo left_info;
JointInfo info;


MR_Indy7 mr_indy7;
MR_Indy7 mr_indy7_l;
MR_Indy7 mr_indy7_r;
MR_DualArm mr_dualarm;
MR_Indy7_DualArm dualarm;
MR_Trajectory traj;
DualArm* robot;
b3RobotSimulatorClientAPI_NoDirect sim;

// Xenomai RT tasks
RT_TASK RTIndy7_task;
RT_TASK safety_task;
RT_TASK print_task;
int traj_flag = 0;

//BULLET SIM
extern const int CONTROL_RATE;
const int CONTROL_RATE = 1000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;
int end_flag = 0;
double t=0;
double Tf = 20.0;
// RTIndy7_task
RTIME step;
int leftId;
mr::SE3 Trl;
class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
      : Node("joint_state_publisher")
  {

    
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
	desired_joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>("desired_joint_states", 10);
    joint_state_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&JointStatePublisherNode::publish_joint_state, this));
  }

private:
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_joint_state_publisher_;
  void publish_joint_state()
  {
    //ROS TOPIC
	//left_info.act.q<<0,0,0,0,0,0;
	//right_info.act.q<<0,0,0,0,0,0;
    auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
	auto desired_joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
    joint_state_msg->header.stamp = this->now();
	desired_joint_state_msg->header.stamp = this->now();
    joint_state_msg->name = {"l_joint_0", "l_joint_1", "l_joint_2", "l_joint_3", "l_joint_4", "l_joint_5", "r_joint_0", "r_joint_1", "r_joint_2", "r_joint_3", "r_joint_4", "r_joint_5"};
	desired_joint_state_msg->name = {"l_joint_0", "l_joint_1", "l_joint_2", "l_joint_3", "l_joint_4", "l_joint_5", "r_joint_0", "r_joint_1", "r_joint_2", "r_joint_3", "r_joint_4", "r_joint_5"};
    joint_state_msg->position = {left_info.act.q[0], left_info.act.q[1], left_info.act.q[2], 
                                 left_info.act.q[3], left_info.act.q[4], left_info.act.q[5],
                                right_info.act.q[0], right_info.act.q[1], right_info.act.q[2], 
                                right_info.act.q[3], right_info.act.q[4], right_info.act.q[5]};
	joint_state_msg->velocity = {left_info.act.q_dot[0], left_info.act.q_dot[1], left_info.act.q_dot[2], 
                                 left_info.act.q_dot[3], left_info.act.q_dot[4], left_info.act.q_dot[5],
                                right_info.act.q_dot[0], right_info.act.q_dot[1], right_info.act.q_dot[2], 
                                right_info.act.q_dot[3], right_info.act.q_dot[4], right_info.act.q_dot[5]};																							
	desired_joint_state_msg->position = {left_info.des.q[0], left_info.des.q[1], left_info.act.q[2], 
                                 left_info.des.q[3], left_info.des.q[4], left_info.des.q[5],
                                right_info.des.q[0], right_info.des.q[1], right_info.des.q[2], 
                                right_info.des.q[3], right_info.des.q[4], right_info.des.q[5]};							
	desired_joint_state_msg->velocity = {left_info.des.q_dot[0], left_info.des.q_dot[1], left_info.des.q_dot[2], 
                                 left_info.des.q_dot[3], left_info.des.q_dot[4], left_info.des.q_dot[5],
                                right_info.des.q_dot[0], right_info.des.q_dot[1], right_info.des.q_dot[2], 
                                right_info.des.q_dot[3], right_info.des.q_dot[4], right_info.des.q_dot[5]};															
    joint_state_publisher_->publish(*joint_state_msg);
	desired_joint_state_publisher_->publish(*desired_joint_state_msg);
  }


};

void RTIndy7_run(void *arg)
{
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	RTIME now,previous;
	double dt = 0.001;
	now= 0 ;
	previous= 0 ;
	step = 0;
	relmr::JVec eint = relmr::JVec::Zero();
	relmr::JVec q_init;
	q_init<<-0.3532, -1.1644 ,-1.2237, -0.7882, -0.7229,  0.0695,0.3531 , 1.1645  ,1.2237 , 0.788  , 0.723 , -0.0696;
	q_init = -q_init;
	sim.setTimeOut(0.0009);
	robot->reset_q(&sim,q_init);
	sim.stepSimulation();
	
	relmr::JVec max_torque;
	max_torque<<1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000;
	relmr::JVec q_next=relmr::JVec::Random();
	relmr::JVec q_next2=relmr::JVec::Random();
	relmr::JVec qdot_1 =relmr::JVec::Random();
	relmr::JVec qdot_2 =relmr::JVec::Random();
	int trajType = 0; 
	traj.addRelativeJointTrajectory(q_init,q_next,relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),0,2,dt,trajType);
	traj.addScurveRelativeJointTrajectory(q_next,q_init,relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),2,4,dt,trajType);
	// traj.addScurveRelativeJointTrajectory(q_init,q_next2,relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),4,6,dt,trajType);
	// traj.addScurveRelativeJointTrajectory(q_next2,q_init,relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),6,8,dt,trajType);
	// traj.addRelativeJointTrajectory(q_next,q_init,qdot_1,relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),2,4,dt,trajType);
	// traj.addRelativeJointTrajectory(q_init,q_next2,relmr::JVec::Zero(),qdot_2,relmr::JVec::Zero(),relmr::JVec::Zero(),4,6,dt,trajType);
	// traj.addRelativeJointTrajectory(q_next2,q_init,qdot_2,relmr::JVec::Zero(),relmr::JVec::Zero(),relmr::JVec::Zero(),6,8,dt,trajType);
	Tf = 8;
	while (t<Tf)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle
		previous = rt_timer_read();
		relmr::JVec q_des = relmr::JVec::Zero();
		relmr::JVec qdot_des = relmr::JVec::Zero();
		relmr::JVec qddot_des = relmr::JVec::Zero();		
		traj.getRelativeJointTrajectory(t ,q_des,qdot_des,qddot_des );
		relmr::JVec q = robot->get_q(&sim);
		relmr::JVec qdot = robot->get_qdot(&sim);
		dualarm.FKinBody(q);
		relmr::JVec q_rel = dualarm.get_q_rel(q);
		Trl = relmr::FKinBody(dualarm.M, dualarm.Blist, q_rel);
		eint += (q_des-q)*dt;
		relmr::JVec HinfTorq = dualarm.HinfControlSim(q,qdot,q_des,qdot_des,qddot_des,eint);
		relmr::JVec G = dualarm.GravityForces(q);		
		robot->set_torque(&sim,HinfTorq,max_torque);
		sim.stepSimulation();




		// Logging
		right_info.act.q = q.segment<6>(0);
		right_info.act.q_dot = qdot.segment<6>(0);
		right_info.des.q = q_des.segment<6>(0);
		right_info.des.q_dot = qdot_des.segment<6>(0);
		right_info.des.q_ddot = qddot_des.segment<6>(0);
		left_info.act.q=q.segment<6>(6);
		left_info.act.q_dot = qdot.segment<6>(6);
		left_info.des.q = q_des.segment<6>(6);
		left_info.des.q_dot = qdot_des.segment<6>(6);
		left_info.des.q_ddot = qddot_des.segment<6>(6);

		now = rt_timer_read();
		step = now-previous;
		t+=dt;
	}
	traj.saveJointTrajectory("");

	run =0;
	exit(1);
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	while (run)
	{
		
		rt_task_wait_period(NULL); //wait for next cycle
		
		//rt_printf("R: %f -- %f %f %f %f %f %f\n",t,right_info.act.q[0],right_info.act.q[1],right_info.act.q[2],right_info.act.q[3],right_info.act.q[4],right_info.act.q[5]);
		//rt_printf("L: %f -- %f %f %f %f %f %f\n",t,left_info.act.q[0],left_info.act.q[1],left_info.act.q[2],left_info.act.q[3],left_info.act.q[4],left_info.act.q[5]);
		// cout<<"TRL1"<<endl;
		// cout<<mr::TransInv(dualarm.Tbl)*dualarm.Tbr<<endl;
		// cout<<"TRL2"<<endl;
		// cout<<Trl<<endl;
		//rt_printf("loop time : %llu ns \n" , step);
	}
}


/****************************************************************************/
void signal_handler(int signum)
{
	rt_task_delete(&RTIndy7_task);
	rt_task_delete(&safety_task);
	rt_task_delete(&print_task);
	// FTConfigParam[NUM_IO_MODULE+NUM_AXIS]=FT_STOP_DEVICE;
	// FTConfigParamCB[0]=FT_STOP_DEVICE;
	// nrmk_master.writeBuffer(0x70003, FTConfigParam);
	// nrmk_master.writeBuffer(0x71007, FTConfigParamCB);
	// nrmk_master.processRxDomain();

	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                Servo drives Stopped!               ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
	run = 0;
	exit(1);
    
}


/****************************************************************************/
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	// Perform auto-init of rt_print buffers if the task doesn't do so
	std::cout<<"start<"<<std::endl;
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns = 1000000; // nanosecond -> 1kHz
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit

	mr_indy7=MR_Indy7();
    mr_indy7.MRSetup();
	mr_indy7_l=MR_Indy7();
    mr_indy7_l.MRSetup();
	mr_indy7_l.g<<0,8.487,-4.9;
	mr_indy7_r=MR_Indy7();
    mr_indy7_r.MRSetup();
	mr_indy7_r.g<<0,8.487,-4.9;
	mr_dualarm = MR_DualArm();
	mr_dualarm.MRSetup();
	dualarm=MR_Indy7_DualArm();
	dualarm.MRSetup();	
	traj = MR_Trajectory();
	//---------BULLET SETUP START------------------
	b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	if (!b3CanSubmitCommand(client))
	{
		printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
	}	
    	
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_physicsClientHandle = client;
	data.m_guiHelper = 0;
	sim.setInternalData(&data);
	sim.resetSimulation();
	sim.setGravity( btVector3(0 , 0 ,-9.8));
	int bodyId = sim.loadURDF("model/body.urdf");  
	leftId = sim.loadURDF("model/indy7.urdf");  
	int rightId = sim.loadURDF("model/indy7.urdf");  
	btVector3 left_pos(0,0.1563,0.3772);
	btQuaternion left_orn(-0.5,0,0,0.866);
	btVector3 right_pos(0,-0.1563,0.3772);
	btQuaternion right_orn(0.0,0.5,-0.866,0);    
	sim.resetBasePositionAndOrientation(leftId,left_pos, left_orn);
	sim.resetBasePositionAndOrientation(rightId,right_pos, right_orn);	
    robot= new DualArm(&sim,rightId,leftId);


	//---------BULLET SETUP END-----------------
	
	
	// RTIndy7_task: create and start
	
	printf("Now running rt task ...\n");

	// RTIndy7 control
	rt_task_create(&RTIndy7_task, "RTIndy7_task", 0, 2, 0);
	rt_task_start(&RTIndy7_task, &RTIndy7_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 1, 0);
	rt_task_start(&print_task, &print_run, NULL);
	

	// Must pause here
  	auto node = std::make_shared<JointStatePublisherNode>();
	rclcpp::WallRate loop_rate(1000);
	while (rclcpp::ok()&&run)
	{
	    rclcpp::spin_some(node);
	    loop_rate.sleep();
		
	}
	rclcpp::shutdown();  

	// Finalize
	signal_handler(0);
	
    return 0;
}


