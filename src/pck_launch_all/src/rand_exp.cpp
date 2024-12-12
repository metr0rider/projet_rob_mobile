#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib> // Pour rand() et srand()
#include <ctime>   // Pour initialiser srand() avec le temps actuel

int main(int argc, char **argv)
{
    // Initialisation de ROS
    ros::init(argc, argv, "random_exploration_node");
    ros::NodeHandle nh;
    ros::Publisher teleop_pub;

    teleop_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // Publication sur le topic /cmd_vel
    ros::Rate loop_rate(1); // Fréquence de publication des commandes (1 Hz)

    // Initialisation du générateur de nombres aléatoires
    srand(static_cast<unsigned>(time(0)));

    // Boucle principale
    while (ros::ok())
    {
        geometry_msgs::Twist twist;

        // Générer des vitesses aléatoires
        twist.linear.x = static_cast<double>(rand() % 200 - 100) / 100.0; // Entre -1.0 et 1.0
        twist.angular.z = static_cast<double>(rand() % 200 - 100) / 100.0; // Entre -1.0 et 1.0

        // Publier le message
        ROS_INFO("Exploration aléatoire : linear.x = %f, angular.z = %f", twist.linear.x, twist.angular.z);
        teleop_pub.publish(twist);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

