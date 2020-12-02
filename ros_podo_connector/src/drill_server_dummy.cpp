/*for ROS Action msg */
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>

/*Pre-defined Action msg*/
#include <ros_podo_connector/drill_armAction.h>
#include <ros_podo_connector/drill_baseAction.h>

#include <stdio.h>
#include <string.h>


class drill_armAction
{
protected:
    ros::NodeHandle nh_drillArm;
    actionlib::SimpleActionServer<ros_podo_connector::drill_armAction> asDrill_;
    std::string action_name_;

    // create messages that are used to published feedback&result
    ros_podo_connector::drill_armFeedback feedback_;
    ros_podo_connector::drill_armResult result_;




public:

    drill_armAction(std::string name) :
        asDrill_(nh_drillArm, name, boost::bind(&drill_armAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asDrill_.start();
    }

    ~drill_armAction(void)
    {
    }

    /*Call Back function when goal is received from Action client*/
    void executeCB(const ros_podo_connector::drill_armGoalConstPtr &goal)
    {
        std::cout << std::endl << "drill_arm_dummy CB" <<std::endl;

        usleep(500);

        asDrill_.setSucceeded(result_);
    }
};

class drill_baseAction
{
protected:
    ros::NodeHandle nh_drillBase;
    actionlib::SimpleActionServer<ros_podo_connector::drill_baseAction> asDrillB_;
    std::string action_name_;

    // create messages that are used to published feedback&result
    ros_podo_connector::drill_baseFeedback feedback_;
    ros_podo_connector::drill_baseResult result_;

public:

    drill_baseAction(std::string name) :
        asDrillB_(nh_drillBase, name, boost::bind(&drill_baseAction::executeCB, this, _1), false),
        action_name_(name)
    {
        asDrillB_.start();
    }

    ~drill_baseAction(void)
    {
    }

    /*Call Back function when goal is received from Action client*/
    void executeCB(const ros_podo_connector::drill_baseGoalConstPtr &goal)
    {
        std::cout << std::endl << "drill_base_dummy CB" <<std::endl;

        usleep(500);

        asDrillB_.setSucceeded(result_);
    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "drill_server_dummy");

  drill_armAction drill_arm_dummy("drill_arm");
  drill_baseAction drill_base_dummy("drill_base");

  ROS_INFO("Starting drill_server_dummy module");

  ros::spin();
  return 0;

}

