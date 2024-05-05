#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <cmath>


float r1,r2,t,dt,r1_dot,r2_dot,r1_ddot,r2_ddot,r1_prev,r2_prev,r1_dprev,r2_dprev;

int main(int argc, char **argv){

    //Node Set up
    ros::init(argc, argv, "ref_data");
    ros::NodeHandle nh;

    //Publisher set up
    ros::Publisher ref_pub = nh.advertise<geometry_msgs::Vector3>("/ref_data",10);
    ros::Publisher refd_pub = nh.advertise<geometry_msgs::Vector3>("/refd_data",10);
    ros::Publisher refdd_pub = nh.advertise<geometry_msgs::Vector3>("/refdd_data",10);

    //Rate setup
    ros::Rate loop_rate(100);
    dt = (1.0/100.0);


    geometry_msgs::Vector3 ref;
    geometry_msgs::Vector3 ref_dot;
    geometry_msgs::Vector3 ref_ddot;

    while (ros::ok()){
        
        t += dt;

        r1 = 1 + 0.3*sin(t);
        r2 = 0.5 + 0.7 * sin(t);

        r1_dot = (r1 - r1_prev)/dt;
        r2_dot = (r2 - r2_prev)/dt;
        r1_ddot = (r1_dot - r1_dprev)/dt;
        r2_ddot = (r2_dot - r2_dprev)/dt;


        r1_prev = r1;
        r2_prev = r2;

        r1_dprev = r1_dot;
        r2_dprev = r2_dot;

        ref.x = r1;
        ref.y = r2;

        ref_dot.x = r1_dot;
        ref_dot.y = r2_dot;
        
        ref_ddot.x = r1_ddot;
        ref_ddot.y = r2_ddot;

        ref_pub.publish(ref);
        refd_pub.publish(ref_dot);
        refdd_pub.publish(ref_ddot);

        ros::spinOnce();

        loop_rate.sleep();



    }
    
}