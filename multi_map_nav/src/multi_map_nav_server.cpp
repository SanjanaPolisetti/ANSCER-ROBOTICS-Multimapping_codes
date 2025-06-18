#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <multi_map_nav/MultiMapNavAction.h>
#include "multi_map_nav/wormhole_db.hpp"
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionServer<multi_map_nav::MultiMapNavAction> Server;

class MultiMapNavAction {
public:
    MultiMapNavAction(std::string name) :
        as_(nh_, name, boost::bind(&MultiMapNavAction::executeCB, this, _1), false),
        action_name_(name),
        db_("/home/sreejan/wormholes.db") // <-- Make sure path is correct
    {
        current_map_ = "room2"; // Initial map
        as_.start();
    }

    void executeCB(const multi_map_nav::MultiMapNavGoalConstPtr &goal) {
        feedback_.current_step = "Goal received.";
        as_.publishFeedback(feedback_);

        if (goal->target_map == current_map_) {
            feedback_.current_step = "Same map navigation.";
            as_.publishFeedback(feedback_);
            sendToMoveBase(goal->goal_pose);
        } else {
            feedback_.current_step = "Navigating to wormhole...";
            as_.publishFeedback(feedback_);

            auto wormhole = db_.getWormhole(current_map_, goal->target_map);
            geometry_msgs::PoseStamped wh_pose;
            wh_pose.header.frame_id = "map";
            wh_pose.header.stamp = ros::Time::now();
            wh_pose.pose.position.x = wormhole.from_x;
            wh_pose.pose.position.y = wormhole.from_y;
            wh_pose.pose.orientation.w = 1.0;

            sendToMoveBase(wh_pose);

            feedback_.current_step = "Switching maps...";
            as_.publishFeedback(feedback_);
            switchMap(goal->target_map);

            // Wait a bit for AMCL + map_server to settle
            ros::Duration(2.0).sleep();

            // Set initial pose in new map at wormhole exit
            publishInitialPose(wormhole.to_x, wormhole.to_y);

            ros::Duration(1.0).sleep(); // Let amcl settle
            current_map_ = goal->target_map;

            feedback_.current_step = "Navigating to final goal.";
            as_.publishFeedback(feedback_);
            sendToMoveBase(goal->goal_pose);
        }

        result_.success = true;
        as_.setSucceeded(result_);
    }

private:
    ros::NodeHandle nh_;
    Server as_;
    std::string action_name_;
    WormholeDB db_;
    std::string current_map_;

    multi_map_nav::MultiMapNavFeedback feedback_;
    multi_map_nav::MultiMapNavResult result_;

    void sendToMoveBase(const geometry_msgs::PoseStamped& pose) {
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        ac.waitForServer();

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = pose;

        ac.sendGoal(goal);
        ac.waitForResult();
    }

    void switchMap(const std::string& map_name) {
        std::string map_path = "/home/sreejan/catkin_ws/src/my_turtlebot3_worlds/maps/" + map_name + ".yaml";
        std::string kill_cmd = "rosnode kill /map_server";
        std::string load_cmd = "rosrun map_server map_server " + map_path + " &";

        system(kill_cmd.c_str());
        ros::Duration(1.0).sleep(); // Allow clean kill
        system(load_cmd.c_str());
    }

    void publishInitialPose(double x, double y) {
        ros::Publisher pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
        geometry_msgs::PoseWithCovarianceStamped init_pose;

        init_pose.header.frame_id = "map";
        init_pose.header.stamp = ros::Time::now();
        init_pose.pose.pose.position.x = x;
        init_pose.pose.pose.position.y = y;
        init_pose.pose.pose.orientation.w = 1.0;

        for (int i = 0; i < 5; ++i) {
            pose_pub.publish(init_pose);
            ros::Duration(0.2).sleep();
        }
    }
};

