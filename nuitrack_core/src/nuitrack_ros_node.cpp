#include <ros/ros.h>
#include <nuitrack_core.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nuitrack_core");
    NuitrackCore node(ros::this_node::getName());
    node.init("");
    node.run();

    return 0;
}
