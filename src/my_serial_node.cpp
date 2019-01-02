#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <serial/serial.h>
#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Transform.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

serial::Serial ser;

ros::Publisher Reference_pub;

#define sendBufsize 56 
unsigned char send_buffer[sendBufsize];
typedef union{
	unsigned char cv[4];
	float fv;
}float_union;

float_union reference_posx,reference_posy,reference_posz;

void callback(const nav_msgs::OdometryConstPtr &odom_msg){
float_union position_x,position_y,position_z,velocity_x,velocity_y,velocity_z,
				Quaternion0,Quaternion1,Quaternion2,Quaternion3;


	Quaternion0.fv = odom_msg->pose.pose.orientation.w;
	Quaternion1.fv = odom_msg->pose.pose.orientation.x;
	Quaternion2.fv = odom_msg->pose.pose.orientation.y;
	Quaternion3.fv = odom_msg->pose.pose.orientation.z;
	
	position_x.fv = odom_msg->pose.pose.position.x;
	position_y.fv = odom_msg->pose.pose.position.y;
	position_z.fv = odom_msg->pose.pose.position.z;
	velocity_x.fv = odom_msg->twist.twist.linear.x;
	velocity_y.fv = odom_msg->twist.twist.linear.y;
	velocity_z.fv = odom_msg->twist.twist.linear.z;

	memset(send_buffer,0,sizeof(send_buffer));
	send_buffer[0] = 0x55;
	send_buffer[1] = 0xAA;
	send_buffer[2] = 0x30;
	
	send_buffer[3] = position_x.cv[0];
	send_buffer[4] = position_x.cv[1];
	send_buffer[5] = position_x.cv[2];
	send_buffer[6] = position_x.cv[3];

	send_buffer[7] = position_y.cv[0];
	send_buffer[8] = position_y.cv[1];
	send_buffer[9] = position_y.cv[2];
	send_buffer[10] = position_y.cv[3];

	send_buffer[11] = position_z.cv[0];
	send_buffer[12] = position_z.cv[1];
	send_buffer[13] = position_z.cv[2];
	send_buffer[14] = position_z.cv[3];

	send_buffer[15] = velocity_x.cv[0];
	send_buffer[16] = velocity_x.cv[1];
	send_buffer[17] = velocity_x.cv[2];
	send_buffer[18] = velocity_x.cv[3];

	send_buffer[19] = velocity_y.cv[0];
	send_buffer[20] = velocity_y.cv[1];
	send_buffer[21] = velocity_y.cv[2];
	send_buffer[22] = velocity_y.cv[3];

	send_buffer[23] = velocity_z.cv[0];
	send_buffer[24] = velocity_z.cv[1];
	send_buffer[25] = velocity_z.cv[2];
	send_buffer[26] = velocity_z.cv[3];

	send_buffer[27] = Quaternion0.cv[0];
	send_buffer[28] = Quaternion0.cv[1];
	send_buffer[29] = Quaternion0.cv[2];
	send_buffer[30] = Quaternion0.cv[3];

	send_buffer[31] = Quaternion1.cv[0];
	send_buffer[32] = Quaternion1.cv[1];
	send_buffer[33] = Quaternion1.cv[2];
	send_buffer[34] = Quaternion1.cv[3];

	send_buffer[35] = Quaternion2.cv[0];
	send_buffer[36] = Quaternion2.cv[1];
	send_buffer[37] = Quaternion2.cv[2];
	send_buffer[38] = Quaternion2.cv[3];

	send_buffer[39] = Quaternion3.cv[0];
	send_buffer[40] = Quaternion3.cv[1];
	send_buffer[41] = Quaternion3.cv[2];
	send_buffer[42] = Quaternion3.cv[3];

	send_buffer[43] = reference_posx.cv[0];
	send_buffer[44] = reference_posx.cv[1];
	send_buffer[45] = reference_posx.cv[2];
	send_buffer[46] = reference_posx.cv[3];

	send_buffer[47] = reference_posy.cv[0];
	send_buffer[48] = reference_posy.cv[1];
	send_buffer[49] = reference_posy.cv[2];
	send_buffer[50] = reference_posy.cv[3];

	send_buffer[51] = reference_posz.cv[0];
	send_buffer[52] = reference_posz.cv[1];
	send_buffer[53] = reference_posz.cv[2];
	send_buffer[54] = reference_posz.cv[3];

	send_buffer[55] = 0xAA;

	ser.write(send_buffer,sendBufsize);

}

void callback1(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &trajectory_msg){
  geometry_msgs::PointStamped point_stamped;
	reference_posx.fv = trajectory_msg->points[0].transforms[0].translation.x;
	reference_posy.fv = trajectory_msg->points[0].transforms[0].translation.y;
	reference_posz.fv = trajectory_msg->points[0].transforms[0].translation.z;
  
  point_stamped.header.frame_id = "world";
  point_stamped.header.seq = trajectory_msg->header.seq;
  point_stamped.header.stamp = trajectory_msg->header.stamp;
  
  point_stamped.point.x = reference_posx.fv;
  point_stamped.point.y = reference_posy.fv;
  point_stamped.point.z = reference_posz.fv;
  
  Reference_pub.publish(point_stamped);
 
//	ROS_INFO(" reference_posx.fv: %f", reference_posx.fv);
//	ROS_INFO(" reference_posy.fv: %f", reference_posy.fv);
//	ROS_INFO(" reference_posz.fv: %f \r\n", reference_posz.fv);
}

int main (int argc, char** argv){
     ros::init(argc, argv, "my_serial_node");
     ros::NodeHandle n;
     //订阅主题command
     ros::Subscriber Odometry_sub = n.subscribe("/vins_estimator/odometry", 1000, callback);
     ros::Subscriber Navigator_sub = n.subscribe("/command/trajectory", 1000,callback1);
     
     Reference_pub = n.advertise<geometry_msgs::PointStamped>("/reference/trajectory",10);
     try
     {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(230400);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
     }
     catch (serial::IOException& e)
     {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
     }

     if(ser.isOpen()){
         ROS_INFO_STREAM("Serial Port initialized");
     }else{
         return -1;
     }

     ros::Rate loop_rate(200);
     while(ros::ok()){
        ros::spinOnce();

		loop_rate.sleep();
     }
     return 0;
}
