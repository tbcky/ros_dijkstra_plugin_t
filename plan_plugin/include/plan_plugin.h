#ifndef PLAN_PLUGIN_H_
#define PLAN_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "dijkstraros/PathPlanningPlugin.h"

namespace plan_plugin {

    class PlanPlugin : public nav_core::BaseGlobalPlanner {
        public:

            PlanPlugin();
            PlanPlugin(std::string name ,costmap_2d::Costmap2DROS* cosmap_ros);
            void initialize(std::string name ,costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped> &plan);

            size_t ToIndex(float x,float y);
            void FromIndex(size_t index ,int &x,int &y);

            void FromWorldToGrid(float &x,float &y);
            void FromGridToWorld(float &x,float &y);

            bool InGridMap(float &x,float &y);

        private:

            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            bool initialized_;

            float origin_x_;
            float origin_y_;

            float resolution_;

            bool path_at_node_center = false;
            float node_center_set = 0;

            int width_;
            int height_;

            int map_size_;

            ros::ServiceClient makeplan_service_client_;

            void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
            ros::Publisher plan_pub_;

    };
}
#endif 