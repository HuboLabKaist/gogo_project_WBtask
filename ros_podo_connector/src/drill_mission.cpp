#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
/*custom defined Action header for robot motion */
#include <ros_podo_connector/RosPODOmotionAction.h>
#include <ros_podo_connector/RosPODO_BaseAction.h>
#include <ros_podo_connector/RosPODO_ArmAction.h>
#include <ros_podo_connector/RosPODO_GripperAction.h>
#include <ros_podo_connector/drill_armAction.h>
#include <ros_podo_connector/drill_baseAction.h>
/*custom defined header to TCP/IP to PODO */
#include "ROSLANData.h"

enum
{
  DRILL_BASE_REQUEST = 0,
  DRILL_BASE_MOVE,
  DRILL_ARM_REQUEST,
  DRILL_ARM_MOVE,
  DRILL_NOTHING,
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "drill_mission");

  //std::string param;
  ros::NodeHandle nh("~");
  double x = 0., y = 0., z = 0.;
  double baseX = 0., baseY = 0.;

  nh.param("x", x, 0.5);
  nh.param("y", y, -0.246403);
  nh.param("z", z, 0.65);
  nh.param("baseX", baseX, 0.20);
  nh.param("baseY", baseY, 0.0);

  nh.deleteParam("x");      nh.deleteParam("y");      nh.deleteParam("z");
  nh.deleteParam("baseX");  nh.deleteParam("baseY");

  std::cout << "hand(x,y,z) = (" << x << ", " << y << ", " << z << ") " << std::endl;
  std::cout << "base(x,y) = (" << baseX << ", " << baseY << ") " << std::endl;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_BaseAction> ac_base("rospodo_base", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_ArmAction> ac_arm("rospodo_arm", true);
  actionlib::SimpleActionClient<ros_podo_connector::RosPODO_GripperAction> ac_gripper("rospodo_gripper", true);

  actionlib::SimpleActionClient<ros_podo_connector::drill_armAction> ac_drillArm("drill_arm", true);
  actionlib::SimpleActionClient<ros_podo_connector::drill_baseAction> ac_drillBase("drill_base", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac_base.waitForServer(); //will wait for infinite time
  ac_arm.waitForServer(); //will wait for infinite time
  ac_gripper.waitForServer(); //will wait for infinite time

  ac_drillArm.waitForServer();   //will wait for infinite time
  ac_drillBase.waitForServer();  //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  ros_podo_connector::RosPODO_BaseGoal goal_base;
  ros_podo_connector::RosPODO_ArmGoal goal_arm;
  ros_podo_connector::RosPODO_GripperGoal goal_gripper;

  ros_podo_connector::drill_armGoal goal_drill_arm;
  ros_podo_connector::drill_baseGoal goal_drill_base;

  int DrillMode = DRILL_BASE_REQUEST;


  int trial = 0;
  int loopFlag = true;
  while(loopFlag)
  {

    switch(DrillMode)
    {
      case DRILL_BASE_REQUEST:
      {
        ROS_INFO(">> DRILL_BASE_REQUEST");

        goal_drill_base.base_request = 1;
        ac_drillBase.sendGoal(goal_drill_base);

        //wait for the action to return
        bool finished_before_timeout_drillBase = ac_drillBase.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout_drillBase){
          actionlib::SimpleClientGoalState state = ac_drillBase.getState();
          ROS_INFO("DRILL BASE Action finished: %s",state.toString().c_str());

          trial = 0;
          DrillMode = DRILL_BASE_MOVE;

          break;
        }
        else{
          ROS_INFO("DRILL BASE Action did not finish before the time out.");

          trial++;

          if(trial == 5)
          {
            ROS_ERROR("CANNOT FIND THE DRILL BASE GOAL");
            DrillMode = DRILL_NOTHING;
            break;
          }
        }

        break;
      }
      case DRILL_BASE_MOVE:
      {
        ROS_INFO(">> DRILL_BASE_MOVE");

        // ============== Base Data Action  ============== //
        goal_base.wheelmove_cmd = 1;
        goal_base.MoveX = baseX;//0.20;
        goal_base.MoveY = baseY;//0.0;
        goal_base.ThetaDeg = 0;
        ac_base.sendGoal(goal_base);


        //wait for the action to return
        bool finished_before_timeout_base = ac_base.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout_base){
          actionlib::SimpleClientGoalState state = ac_base.getState();
          ROS_INFO("BASE Action finished: %s",state.toString().c_str());
          DrillMode = DRILL_ARM_REQUEST;
        }
        else{
          ROS_INFO("BASE Action did notfinish before the time out.");
          DrillMode = DRILL_NOTHING;
        }

        break;
      }
      case DRILL_ARM_REQUEST:
      {
        ROS_INFO(">> DRILL_ARM_REQUEST");

        //REQUEST ARM MOVE
          goal_drill_arm.arm_request = 1;
          ac_drillArm.sendGoal(goal_drill_arm);

          //wait for the action to return
          bool finished_before_timeout_drillArm = ac_drillArm.waitForResult(ros::Duration(30.0));

          if (finished_before_timeout_drillArm){
            actionlib::SimpleClientGoalState state = ac_drillArm.getState();
            ROS_INFO("DRILL ARM Action finished: %s",state.toString().c_str());
            DrillMode = DRILL_ARM_MOVE;
            trial = 0;
            break;
          }
          else{
            ROS_INFO("DRILL ARM Action did not finish before the time out.");

            trial++;
            if(trial == 5)
            {
              ROS_ERROR("CANNOT FIND THE DRILL HANDLE");
              DrillMode = DRILL_NOTHING;
              break;
            }
          }

        break;
      }
      case DRILL_ARM_MOVE:
      {
        ROS_INFO(">> DRILL_ARM_MOVE");

        // ============== Arm Data Action WBIK ==============  //
        //goal_arm.jointmove_cmd = 3;
        goal_arm.jointmove_cmd = 7;
        goal_arm.wbik_ref[RIGHT_HAND].OnOff_position = CONTROL_ON;
        goal_arm.wbik_ref[RIGHT_HAND].goal_position[0] = x;// 0.5;
        goal_arm.wbik_ref[RIGHT_HAND].goal_position[1] = y;//-0.246403;
        goal_arm.wbik_ref[RIGHT_HAND].goal_position[2] = z;// 0.65;
        goal_arm.wbik_ref[RIGHT_HAND].GoalmsTime = 2000;

        ac_arm.sendGoal(goal_arm);
      //  ros::Duration(3).sleep();

        //wait for the action to return
        bool finished_before_timeout_arm = ac_arm.waitForResult(ros::Duration(30.0));

        if (finished_before_timeout_arm)
        {
          actionlib::SimpleClientGoalState state = ac_arm.getState();
          ROS_INFO("ARM Action finished: %s",state.toString().c_str());
          DrillMode = DRILL_NOTHING;
        }
        else{
          ROS_INFO("ARM Action did not finish before the time out.");
          DrillMode = DRILL_NOTHING;
        }

        break;
      }
      case DRILL_NOTHING:
      {
        ROS_INFO(">> DRILL_NOTHING");
        loopFlag = false;
        break;
      }
    }



  }

/*
  //REQUEST BASE MOVE
  int trial = 0;
  while(1)
  {
    goal_drill_base.base_request = 1;
    ac_drillBase.sendGoal(goal_drill_base);

    //wait for the action to return
    bool finished_before_timeout_drillBase = ac_drillBase.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout_drillBase){
      actionlib::SimpleClientGoalState state = ac_drillBase.getState();
      ROS_INFO("DRILL BASE Action finished: %s",state.toString().c_str());
      break;
    }
    else{
      ROS_INFO("DRILL BASE Action did not finish before the time out.");
    }

    trial++;
    if(trial == 5)
    {
      ROS_ERROR("CANNOT FIND THE DRILL BASE GOAL");
      break;
    }

  }

  //REQUEST ARM MOVE
  trial = 0;
  while(1)
  {
    goal_drill_arm.arm_request = 1;
    ac_drillArm.sendGoal(goal_drill_arm);

    //wait for the action to return
    bool finished_before_timeout_drillArm = ac_drillArm.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout_drillArm){
      actionlib::SimpleClientGoalState state = ac_drillArm.getState();
      ROS_INFO("DRILL ARM Action finished: %s",state.toString().c_str());
      break;
    }
    else{
      ROS_INFO("DRILL ARM Action did not finish before the time out.");
    }

    trial++;
    if(trial == 5)
    {
      ROS_ERROR("CANNOT FIND THE DRILL HANDLE");
      break;
    }

  }




  // ============== Base Data Action  ============== //
  goal_base.wheelmove_cmd = 1;
  goal_base.MoveX = baseX;//0.20;
  goal_base.MoveY = baseY;//0.0;
  goal_base.ThetaDeg = 0;
  ac_base.sendGoal(goal_base);


  //wait for the action to return
  bool finished_before_timeout_base = ac_base.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout_base){
    actionlib::SimpleClientGoalState state = ac_base.getState();
    ROS_INFO("BASE Action finished: %s",state.toString().c_str());
  }
  else{
    ROS_INFO("BASE Action did notfinish before the time out.");
  }

  // ============== Arm Data Action WBIK ==============  //
  //goal_arm.jointmove_cmd = 3;
  goal_arm.jointmove_cmd = 7;
  goal_arm.wbik_ref[RIGHT_HAND].OnOff_position = CONTROL_ON;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[0] = x;// 0.5;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[1] = y;//-0.246403;
  goal_arm.wbik_ref[RIGHT_HAND].goal_position[2] = z;// 0.65;
  goal_arm.wbik_ref[RIGHT_HAND].GoalmsTime = 2000;

  ac_arm.sendGoal(goal_arm);
//  ros::Duration(3).sleep();

  //wait for the action to return
  bool finished_before_timeout_arm = ac_arm.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout_arm)
  {
    actionlib::SimpleClientGoalState state = ac_arm.getState();
    ROS_INFO("ARM Action finished: %s",state.toString().c_str());
  }
  else{
    ROS_INFO("ARM Action did not finish before the time out.");
  }*/


  //exit
  return 0;
}
