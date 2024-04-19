#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Float32.h>

//Parameters initialization
float i;
float r,t;


int main(int argc, char **argv){

    //Node Set up
    ros::init(argc, argv, "refdata");
    ros::NodeHandle nh;

    //Publisher set up
    ros::Publisher refdata_pub = nh.advertise<std_msgs::Float32>("/refdata",10);

    //Rate set up
    ros::Rate loop_rate(100);

    std_msgs::Float32 refdata;


    while(ros::ok()){

        // Time 
        t = i * (1.0/100.0);

        //Different setpoints

        r = 1 + 0.3*sin(t);
        //r = M_PI/2;
        //r = 0 ;
        
        i++;

        //Publish on topic
        refdata.data = r;
        refdata_pub.publish(refdata);

        ros::spinOnce();

        loop_rate.sleep();
    }

}
