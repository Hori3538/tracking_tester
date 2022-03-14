#include <tracking_tester/tracking_tester.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracking_tester_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TrackingTester tracking_tester(nh, pnh);

    // object_detector.process();

    ros::spin();
    return 0;
}
