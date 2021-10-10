#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
#include <three_dof_planar_manipulator/Floats_array.h>
#include <maker_robot/i2c_ros.h>
#include <sensor_msgs/Joy.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define JOINT_NUM 4

class ROBOTHardwareInterface : public hardware_interface::RobotHW 
{
	public:
        ROBOTHardwareInterface(ros::NodeHandle& nh);
        ~ROBOTHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);

        ros::Publisher pub_right;
        ros::Publisher pub_left;

        ros::Publisher pwm_to_arduino;

        ros::ServiceClient right_client;
        ros::ServiceClient left_client;

        
        rospy_tutorials::Floats joints_pub;
        three_dof_planar_manipulator::Floats_array right_joint_read , left_joint_read ;
        
    protected:

        bool is_goal = false;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;
        
        std::string     joint_name_[JOINT_NUM]={"left_front_wheel_joint",
                                                "left_rear_wheel_joint",
                                                "right_front_wheel_joint",
                                                "right_rear_wheel_joint"};  
        double joint_position_[JOINT_NUM];
        double joint_velocity_[JOINT_NUM];
        double joint_effort_[JOINT_NUM];
        double joint_velocity_command_[JOINT_NUM];

       
        
        double joint_right_position;
        double joint_right_velocity;
      
        double joint_left_position;
	double joint_left_velocity;

	double left_motor_pos=0,right_motor_pos=0;

        int left_prev_cmd=0, right_prev_cmd=0;
        
       // i2c_ros::I2C right_motor= i2c_ros::I2C(1, 0x09);
       //	i2c_ros::I2C left_motor = i2c_ros::I2C(0, 0x08); // 0x08
        


        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

       
   private:

        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void navStatusCallBack( const actionlib_msgs::GoalStatusArray::ConstPtr &status); 

        ros::Subscriber joy_sub_;
        ros::Subscriber move_base_sub_;
        
        int linear_, angular_;
        double l_scale_, a_scale_;
        
};

