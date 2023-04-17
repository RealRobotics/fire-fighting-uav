#include "fire_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fire_detector_node");
    ros::NodeHandle nh;

    fire_detector fire_detector(&nh);
    fire_detector.run();

    return EXIT_SUCCESS;
}