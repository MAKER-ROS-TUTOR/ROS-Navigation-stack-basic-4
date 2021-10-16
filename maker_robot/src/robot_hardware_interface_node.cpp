#include <maker_robot/robot_hardware_interface.h>
#include <std_msgs/UInt16.h>
//namesapce i2c_ros

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh),linear_(1),angular_(2) {
    init();
     // joy
     nh_.param("axis_linear", linear_, linear_);
     nh_.param("axis_angular", angular_, angular_);
     nh_.param("scale_angular", a_scale_, a_scale_);
     nh_.param("scale_linear", l_scale_, l_scale_);

     controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
     loop_hz_=10;
     ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

     right_client = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("right_joint_states");
     left_client  = nh_.serviceClient<three_dof_planar_manipulator::Floats_array>("left_joint_states");

     pub_right = nh_.advertise<rospy_tutorials::Floats>("/joint_right_to_aurdino",10);
     pub_left  = nh_.advertise<rospy_tutorials::Floats>("/joint_left_to_aurdino",10);

     //pwm_to_arduino = nh_.advertise<std_msgs::UInt16_t>("/pwm_to_arduino",1);
     pwm_to_arduino = nh_.advertise<std_msgs::UInt16>("/pwm_to_arduino",10);
    
     non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);

     joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &ROBOTHardwareInterface::joyCallback, this);
     //move_base_sub_ = nh_.subscribe<move_base_msgs::MoveBaseAction>("move_base/status",10,&ROBOTHardwareInterface::moveBaseCallback, this);
     move_base_sub_ = nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &ROBOTHardwareInterface::navStatusCallBack,this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
	
	for(int i=0; i< JOINT_NUM; i++)
	{
	// Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);
       
    // Create velocity joint interface
	    hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // Create Joint Limit interface   
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
	    joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
	    velocityJointSaturationInterface.registerHandle(jointLimitsHandle);

	}

  
    
// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

    uint8_t rbuff[1];
    int x;

   right_joint_read.request.req=1.0;
   left_joint_read.request.req=1.0;

/*
   // left_motor.readBytes(rbuff,1);
    x=0;//(int8_t)rbuff[0];
    left_motor_pos+=angles::from_degrees((double)x);
    joint_position_[0]=left_motor_pos;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos+=angles::from_degrees((double)x);
    joint_position_[1]=right_motor_pos;
*/
    //ROS_INFO("pos=%.2f x=%d ",pos,x);

   

    if(right_client.call(right_joint_read))
	{
	   // joint_right_position  = angles::from_degrees(right_joint_read.response.res[0]);
	   // joint_left_velocity   = angles::from_degrees(right_joint_read.response.res[1]);

            right_motor_pos += angles::from_degrees((double)right_joint_read.response.res[0] ); 
            joint_position_[2]=right_motor_pos ; 
            joint_position_[3]=right_motor_pos ; 
	     
         //   ROS_INFO("Right Pos: %.2f,  %.2f  ",right_joint_read.response.res[0],angles::from_degrees(right_joint_read.response.res[0]));
          //  right_motor_pos+=joint_right_position;
          //  joint_position_[0] = right_motor_pos;
          //  joint_position_[1] = joint_right_velocity; 

/*
if more than one joint,
        get values for joint_position_2, joint_velocity_2,......
*/	    
	    
	}
	else
	{
	   // joint_right_position  = 0;
	   // joint_left_position   = 0;
	}


    if(left_client.call(left_joint_read))
	{
	  //  joint_right_position  =  angles::from_degrees(left_joint_read.response.res[0]);
	   // joint_left_velocity   =  angles::from_degrees(left_joint_read.response.res[1]);

              left_motor_pos += angles::from_degrees((double)left_joint_read.response.res[0] ); 
              joint_position_[0]= left_motor_pos; 
              joint_position_[1]= left_motor_pos;         
	    
          //  ROS_INFO("Left Pos: %.2f,  %.2f ",left_joint_read.response.res[0],left_joint_read.response.res[1]);

         //   joint_position_[0] = joint_left_position;
         //   joint_position_[1] = joint_left_velocity; 
//
	    
	    
	}
	else
	{
	   // joint_right_position  = 0;
	   // joint_left_position   = 0;
	}
	
}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   
    
    uint8_t wbuff[2];

    int velocity,result;

    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);

     if( is_goal){

          velocity =0;   
     }

	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;

   // joints_pub.data.clear();
   // joints_pub.data.push_back( velocity );
   // joints_pub.data.push_back( velocity >> 8);

   
    
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(left_prev_cmd!=velocity)
    {
            pub_left.publish(joints_pub);
	    result = 0 ;// left_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    left_prev_cmd=velocity;
            ROS_INFO("Log1-LEFT wheel command %d , %d" , velocity , velocity>>8 );
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[2]);

  if( is_goal){

          velocity =0;   
     }
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;

   // joints_pub.data.clear();
   // joints_pub.data.push_back( velocity );
   // joints_pub.data.push_back( velocity >> 8);

	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity)
    {
            pub_right.publish(joints_pub);
	    result = 0; // right_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd=velocity;
           ROS_INFO("Log1-RIGHT wheel command %d , %d" , velocity , velocity>>8 );
    }

		
}

void ROBOTHardwareInterface::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    int velocity;
    uint8_t wbuff[2];

    wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);

   // joints_pub.data.clear();
   // joints_pub.data.push_back( velocity );
   // joints_pub.data.push_back( velocity >> 8);
    ROS_INFO("JOY Message %d %d", wbuff[0] , wbuff[1]  );
 // turtlesim::Velocity vel;
 // vel.angular = a_scale_*joy->axes[angular_];
 // vel.linear = l_scale_*joy->axes[linear_];
 // vel_pub_.publish(vel);
}
void ROBOTHardwareInterface::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
      int velocity = 255;
      int status_id = 0;
      uint8_t wbuff[2];

      std_msgs::UInt16 msg;
      msg.data = 0;
      
      if (!status->status_list.empty()){
          actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
          status_id = goalStatus.status;
      }
      
      if( status_id == 3 ) // The goal was achieved successfully
	{
          //    is_goal = true;
         // velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
         // wbuff[0]=velocity;
         // wbuff[1]=velocity >> 8;

         // joints_pub.data.clear();
         // joints_pub.data.push_back( velocity );
         // joints_pub.data.push_back( velocity >> 8);

         // pub_left.publish(joints_pub);
         // pub_right.publish(joints_pub);

        //  pwm_to_arduino.publish(msg);

          ROS_INFO("MoveBase -- The goal was achieved successfully"); 

        }
     ROS_INFO("MoveBase Action Message %d" , status_id );
      
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mobile_robot_hardware_interface");
    ros::NodeHandle nh;
    //ros::AsyncSpinner spinner(4);  
    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    ROBOTHardwareInterface ROBOT(nh);
    //spinner.start();
    spinner.spin();
    //ros::spin();
    return 0;
}
