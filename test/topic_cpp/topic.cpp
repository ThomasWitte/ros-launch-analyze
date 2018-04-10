#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>

using namespace std::string_literals;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic");

    ros::NodeHandle n;

    // Basic publisher and subscriber
    ros::Publisher pub = n.advertise<std_msgs::String>("chatter", 20);
    ros::Subscriber sub = n.subscribe<std_msgs::String>("chatter", 20, [&argc](auto msg) {
        ROS_INFO_STREAM(msg->data);
    });
    
    // Publisher with topic name calculated at runtime
    std::string topic = "chatter" + std::to_string(2);
    ros::Publisher pub2 = n.advertise<std_msgs::String>(topic, 20);

    // latched publisher
    ros::Publisher pub3 = n.advertise<std_msgs::String>("chatter3", 20, true);
    
    // publisher with typedef
    using string_msg_t = std_msgs::String;
    ros::Publisher pub4 = n.advertise<string_msg_t>("chatter4", 20, true);

    // Publisher with topic name from local parameter
    ros::Publisher pub5 = n.advertise<std_msgs::String>(ros::param::param("~param1", "chatter5"s), 20);

    // Publisher with topic name from global parameter
    ros::Publisher pub6 = n.advertise<std_msgs::String>(ros::param::param("param2", "chatter6"s), 20);

    ros::Rate loop_rate(2);
    int count = 0;
    while (ros::ok())
    {
        std_msgs::String str;
        str.data = "hallo " + std::to_string(count);

        pub.publish(str);
        pub2.publish(str);
        pub3.publish(str);
        pub4.publish(str);
        pub5.publish(str);
        pub6.publish(str);

        {
        auto msg = ros::topic::waitForMessage<std_msgs::String>("chatter3");
        if (msg)
            ROS_INFO_STREAM(msg->data);
        }

        {
        auto msg = ros::topic::waitForMessage<string_msg_t>("chatter4", n, ros::Duration(1));
        if (msg)
            ROS_INFO_STREAM(msg->data);
        }

        ROS_INFO("messages received");

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}

