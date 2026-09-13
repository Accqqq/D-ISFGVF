#include <bspline_race/formation_planning.h>


using namespace FLAG_Race;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "formation_planning");
    ros::NodeHandle nh("~");
    
    ROS_INFO("\033[1;32m formation planner initialization complete.\033[0m");

    gvf_manager manager(nh);

    int spinner_threads = 8;
    nh.param("gvf/spinner_threads", spinner_threads, 8);
    if (spinner_threads < 1) spinner_threads = 1;
    ROS_INFO("[GVF] formation planner spinner_threads=%d", spinner_threads);
    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();
    ros::waitForShutdown();
    
    return 0;
}
