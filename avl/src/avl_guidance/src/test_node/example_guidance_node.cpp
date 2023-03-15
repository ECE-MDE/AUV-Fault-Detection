//==============================================================================
// Autonomous Vehicle Library
//
// Description: A simple guidance node example.
//==============================================================================

// Guidance node base class
#include <avl_guidance/guidance_node.h>

// Action definitions
#include <avl_guidance/DiveAction.h>
using namespace avl_guidance;

//==============================================================================
//                              NODE DEFINITION
//==============================================================================

class ExampleGuidanceNode : public PathGuidanceNode
{

public:

    //--------------------------------------------------------------------------
    // Name:        GuidanceNode constructor
    //--------------------------------------------------------------------------
    ExampleGuidanceNode(int argc, char **argv) : public
        GuidanceNode<DiveAction, DiveGoalPtr, DiveFeedback, DiveResult>(argc, argv)
    {

    }

private:

    // Path variables
    int num_points_total;
    int num_points_completed;

private:

    //--------------------------------------------------------------------------
    // Name:        start_new_task
    // Description:
    //--------------------------------------------------------------------------
    void start_new_task(DiveGoalPtr goal)
    {

        num_points_total = goal->points.size() / 3;
        if (num_points_total < 2)
        {
            log_info("path task rejected, must have at least 2 points");
            PathResult result;
            result.success = false;
            finish_task(false, result);
        }
        else
        {
            log_info("path task started with %d points", num_points_total);
            num_points_completed = 0;
        }

    }

    //--------------------------------------------------------------------------
    // Name:        iterate_task
    // Description:
    //--------------------------------------------------------------------------
    void iterate_task()
    {

        log_debug("task iterated");
        num_points_completed++;
        PathFeedback feedback;
        feedback.percent_complete = static_cast<double>(num_points_completed) /
                                    static_cast<double>(num_points_total) * 100.0;
        publish_feedback(feedback);
        if (num_points_completed == num_points_total)
        {
            PathResult result;
            result.success = true;
            finish_task(true, result);
        }

        log_info("%d/%d waypoints completed",
            num_points_completed,
            num_points_total);

    }

    //--------------------------------------------------------------------------
    // Name:        cancel_task
    // Description:
    //--------------------------------------------------------------------------
    PathResult cancel_task()
    {
        log_info("task cancelled");
        PathResult result;
        result.success = false;
        return result;
    }

    //--------------------------------------------------------------------------
    // Name:        init
    // Description:
    //--------------------------------------------------------------------------
    void init()
    {
        log_info("node init");
        init_task_server("guidance/path");
    }

    //--------------------------------------------------------------------------
    // Name:        shutdown
    // Description:
    //--------------------------------------------------------------------------
    void shutdown()
    {
        log_info("node shutdown");
    }

};

//==============================================================================
//                                  MAIN
//==============================================================================
int main(int argc, char **argv)
{
    ExampleGuidanceNode node(argc, argv);
    node.start();
    return 0;
}
