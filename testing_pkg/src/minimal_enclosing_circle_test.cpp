#include "ros/ros.h"

#include <fpp_ros/geometry_info/minimal_enclosing_circle.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "min_enc_circle_node");
    ros::NodeHandle nh;

    // sleep(15);
    
    geometry_info::MinimalEnclosingCircle min_circle;
    min_circle.calcMinimalEnclosingCircle({ });
    std::cout << "\n";

    std::vector<Eigen::Vector2f> points;
    Eigen::Vector2f point;
    point << 0,1;
    points.push_back(point);
    std::cout<<"point number:" << points.size() <<"\n";
    min_circle.calcMinimalEnclosingCircle(points);
    std::cout << "\n";

    point << 0,0;
    points.push_back(point);
    std::cout<<"point number:" << points.size() <<"\n";
    min_circle.calcMinimalEnclosingCircle(points);
    std::cout << "\n";

    point << 1,0;
    points.push_back(point);
    std::cout<<"point number:" << points.size() <<"\n";
    min_circle.calcMinimalEnclosingCircle(points);

    ros::spin();
    return 0;
}