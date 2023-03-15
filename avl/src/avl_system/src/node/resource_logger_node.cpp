//==============================================================================
// Autonomous Vehicle Library
//
// Description: Simple node to log CPU and memory usage of a linux system.
//
// Servers:     None
//
// Clients:     None
//
// Publishers:  None
//
// Subscribers: None
//==============================================================================

// Node base class
#include <avl_core/node.h>

// C++ functions
#include <fstream>
#include <iostream>
#include <numeric>
#include <unistd.h>
#include <vector>

#include "sys/types.h"
#include "sys/sysinfo.h"

//==============================================================================
//                             STRUCT DEFINITION
//==============================================================================

struct MemInfo
{
    double mem_total;
    double mem_used;
    double mem_percent;
    double swap_total;
    double swap_used;
    double swap_percent;
};

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ResourceLoggerNode : public Node
{

public:

    //--------------------------------------------------------------------------
    // Name:        ResourceLoggerNode constructor
    //--------------------------------------------------------------------------
    ResourceLoggerNode(int argc, char **argv) : Node(argc, argv)
    {

    }

private:

    // Timer for querying resource usage
    ros::Timer timer;

    // Previous idle and total CPU times
    size_t t_idle_prev = 0;
    size_t t_total_prev = 0;

private:

    //--------------------------------------------------------------------------
    // Name:        get_cpu_percent
    // Description: Reads CPU idle and total times and calculates the CPU usage.
    // Returns:     CPU usage percentage.
    //--------------------------------------------------------------------------
    double get_cpu_percent()
    {

        // Open the stat file
        std::ifstream proc_stat("/proc/stat");

        // Skip the 'cpu' prefix.
        proc_stat.ignore(5, ' ');

        // Get the CPU times from the file
        std::vector<size_t> cpu_times;
        for (size_t time; proc_stat >> time; cpu_times.push_back(time));

        if (cpu_times.size() < 4)
            return -1.0;

        // Get total and idle times
        size_t t_idle = cpu_times.at(3);
        size_t t_total = std::accumulate(cpu_times.begin(), cpu_times.end(), 0);

        // Calculate cpu percent
        double dt_idle = t_idle - t_idle_prev;
        double dt_total = t_total - t_total_prev;
        double cpu_percent = 100.0 * (1.0 - dt_idle / dt_total);

        // Update previous values
        t_idle_prev = t_idle;
        t_total_prev = t_total;

        return cpu_percent;

    }

    //--------------------------------------------------------------------------
    // Name:        get_mem_info
    // Description: Reads CPU idle and total times and calculates the CPU usage.
    // Returns:     CPU usage percentage.
    //--------------------------------------------------------------------------
    MemInfo get_mem_info()
    {

        struct sysinfo sys_info;
        sysinfo(&sys_info);

        MemInfo mem_info;
        mem_info.mem_total = sys_info.totalram * sys_info.mem_unit;
        mem_info.mem_used = (sys_info.totalram - sys_info.freeram) * sys_info.mem_unit;
        mem_info.mem_percent = mem_info.mem_used / mem_info.mem_total * 100.0;
        mem_info.swap_total = sys_info.totalswap * sys_info.mem_unit;
        mem_info.swap_used =  (sys_info.totalswap - sys_info.freeswap) * sys_info.mem_unit;
        mem_info.swap_percent = mem_info.swap_used / mem_info.swap_total * 100.0;
        return mem_info;

    }

    //--------------------------------------------------------------------------
    // Name:        timer_callback
    // Description: Called when the timer triggers.
    // Arguments:   - event: ROS timer event.
    //--------------------------------------------------------------------------
    void timer_callback(const ros::TimerEvent& event)
    {

        double cpu_percent = get_cpu_percent();
        MemInfo mem_info = get_mem_info();

        log_data("[system] %.1f %.1f %.1f %.1f %.1f %.1f %.1f",
            cpu_percent,
            mem_info.mem_total,
            mem_info.mem_used,
            mem_info.mem_percent,
            mem_info.swap_total,
            mem_info.swap_used,
            mem_info.swap_percent);

    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description: Initializes the node. Called when the node is started.
    //--------------------------------------------------------------------------
    void init()
    {

        // Add data headers
        add_data_header("[system] cpu mem_total mem_used mem_percent swap_total swap_used swap_percent");
        add_data_header("[system] %% bytes bytes %% bytes bytes %%");

        // Set up the timer
        auto timer_cb = std::bind(&ResourceLoggerNode::timer_callback,
            this, std::placeholders::_1);
        timer = node_handle->createTimer(
            ros::Duration(1.0 / get_param<double>("~rate")), timer_cb);

    }

    //--------------------------------------------------------------------------
    // Name:        run
    // Description: Main node loop. Called after the node is initialized.
    //--------------------------------------------------------------------------
    void run()
    {
        ros::spin();
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================

// Name and initialize the node
int main(int argc, char **argv)
{
    ResourceLoggerNode node(argc, argv);
    node.start();
    return 0;
}
