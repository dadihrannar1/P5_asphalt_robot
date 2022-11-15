#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

//Class to handle changes in world coordinates by reading wheel encoders
class DiffDrive{
private:
    double axle_length;     //Distance between wheel center points, in meters
    double wh_radius;       //Wheel radius, in meters
    double r_angle_per_tick;//Wheel angle per encoder tick, in radians
    double l_angle_per_tick;//Wheel angle per encoder tick, in radians

    //Translation and orientation of trailer in relation to world origin
    double world_x_trans = 0;
    double world_y_trans = 0;
    double world_z_rot = 0;

    //Function for adding angles bounded to [0, 2*PI[
    double angle_add(double angle_1, double angle_2){
        if(angle_1 + angle_2 >= 2*M_PI) return angle_1 + angle_2 - 2*M_PI;
        else if(angle_1 + angle_2 < 0) return angle_1 + angle_2 + 2*M_PI;
        else return angle_1 + angle_2;
    }
public:
    //Constructor for DiffDrive class
    DiffDrive(double wheel_radius, int encoder_increments, double length_between_wheels){
        //Calculate distance per tick for each encoder
        r_angle_per_tick = 2 * M_PI / encoder_increments;
        l_angle_per_tick = 2 * M_PI / encoder_increments;

        //Store length between wheels and wheel radius
        axle_length = length_between_wheels;
        wh_radius = wheel_radius;
    }

    //Method for reading encoder values and calculating new world coordinates
    void get_new_transform(){
        //TODO: Somehow read encoder increments
        int delta_r_encoder = 1;
        int delta_l_encoder = 1;

        //Calculate angle changes
        double delta_r_angle = delta_r_encoder * r_angle_per_tick;
        double delta_l_angle = delta_l_encoder * l_angle_per_tick;

        //Calculate position change in local X
        double delta_x = wh_radius/2 * (delta_l_angle + delta_r_angle);

        //Calculate angular change around local Z
        double delta_z = wh_radius/axle_length * (delta_l_angle - delta_r_angle);

        //Calculate new world coordinates (-Y is forward direction)
        world_x_trans += delta_x * cos(world_z_rot - 0.5 * M_PI);
        world_y_trans += delta_x * sin(world_z_rot - 0.5 * M_PI);
        world_z_rot = angle_add(world_z_rot, delta_z);

        //Debug
        ROS_INFO("\nX:\t%f\nY:\t%f\nAngle:\t%f", world_x_trans, world_y_trans, world_z_rot/M_PI*180);
    }

    //Method returns x translation
    double get_x(){return world_x_trans;}

    //Method returns y translation
    double get_y(){return world_y_trans;}

    //Method returns orientation as quaternion
    geometry_msgs::Quaternion get_quat(){
        //TF odometry orientation is 6DOF quaternion which is created from yaw angle
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(0, 0, world_z_rot);
        quat_tf.normalize(); //

        //Convert quaternion datatype from tf to msg
        geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat_tf);

        //Return quaternion message
        return quat_msg;
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;
    //ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf2_ros::TransformBroadcaster odom_broadcaster;

    //Create differential drive handler
    DiffDrive ddr_position(1/(2*M_PI), 2048, 1);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(100);
    while(n.ok()){
        ros::spinOnce();    // check for incoming messages
        current_time = ros::Time::now();

        //Compute world coordinates
        ddr_position.get_new_transform();

        //Publish the transform over tf2
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = ddr_position.get_x();
        odom_trans.transform.translation.y = ddr_position.get_y();
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = ddr_position.get_quat();

        //Send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //Publish the odometry message over ROS
        //nav_msgs::Odometry odom;
        //odom.header.stamp = current_time;
        //odom.header.frame_id = "odom";

        //Set the position
        //odom.pose.pose.position.x = ddr_position.get_x();
        //odom.pose.pose.position.y = ddr_position.get_y();
        //odom.pose.pose.position.z = 0.0;
        //odom.pose.pose.orientation = ddr_position.get_quat();

        //set the velocity
        //odom.child_frame_id = "base_link";
        //odom.twist.twist.linear.x = vx;
        //odom.twist.twist.linear.y = vy;
        //odom.twist.twist.angular.z = vth;

        //publish the message
        //odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}