#include <plan_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(plan_plugin::PlanPlugin,nav_core::BaseGlobalPlanner)

namespace plan_plugin{

    PlanPlugin::PlanPlugin(){
        initialized_ = false;
    }

    PlanPlugin::PlanPlugin(std::string name,costmap_2d::Costmap2DROS *costmap_ros){
        initialized_ = false;
        initialize(name,costmap_ros);
    }

    void PlanPlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros){
        if(!initialized_){
            name = "SrvClientPlugin";
            ros::NodeHandle private_nh("~/"+name);//此处命名空间为/move_base/name/

            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            //地图原点，就是地图左下角点的世界坐标
            origin_x_ = costmap_ ->getOriginX();
            origin_y_ = costmap_ ->getOriginY();
            //地图矩阵的宽度和高度
            width_ = costmap_ ->getSizeInCellsX();
            height_ = costmap_ ->getSizeInCellsY();

            resolution_ = costmap_ ->getResolution();

            map_size_ =  width_ * height_;

            makeplan_service_client_ = private_nh.serviceClient<dijkstraros::PathPlanningPlugin>("make_plan");

            makeplan_service_client_.waitForExistence();//堵塞线程，等待目标服务器正常上线

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);

            path_at_node_center = true;
            if(path_at_node_center){
                node_center_set = resolution_ / 2;
            }
            initialized_ = true;
        }
    }

    bool PlanPlugin::makePlan(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped> &plan){

                plan.clear();
                publishPlan(plan);

                std::vector<int> costmap(map_size_);

                for(size_t idx = 0;idx < map_size_;idx++){
                    int x,y;
                    x = idx % width_;
                    y = std::floor(idx / width_);//floor取不大于该数值的整数值
                    //获取指定位置的代价指（一般为unsigned char、uint8_t）使用static_cast转换为int型
                    //代价值0为自由，1～255为障碍物占比
                    costmap.at(idx) = static_cast<int>(costmap_ -> getCost(x,y));
                }

                float start_x = start.pose.position.x;
                float start_y = start.pose.position.y;
                float goal_x = goal.pose.position.x;
                float goal_y = goal.pose.position.y;

                size_t start_index = 0;
                size_t goal_index = 0;

                if(InGridMap(start_x,start_y) && InGridMap(goal_x,goal_y)){
                    FromWorldToGrid(start_x,start_y);
                    FromWorldToGrid(goal_x,goal_y);

                    start_index = ToIndex(start_x,start_y);
                    goal_index = ToIndex(goal_x,goal_y);
                }

                dijkstraros::PathPlanningPlugin makeplan;
                makeplan.request.costmap_ros = costmap;
                makeplan.request.start = start_index;
                makeplan.request.goal = goal_index;
                makeplan.request.width = width_;
                makeplan.request.height = height_;
                //world->Grid->idex

                makeplan_service_client_.call(makeplan);

                std::vector<int> index_plan = makeplan.response.plan;

                ROS_DEBUG("Number of plan points: %d", unsigned(index_plan.size()));
                ROS_DEBUG("Number of points: %d", unsigned(index_plan.size()));

                if(index_plan.size()){
                    index_plan.insert(index_plan.begin(),start_index);
                    index_plan.push_back(goal_index);
                    //index -> Grid -> world
                    for(auto p : index_plan){
                        int x,y;
                        FromIndex(p,x,y);
                        float x_path = static_cast<float>(x);
                        float y_path = static_cast<float>(y);

                        FromGridToWorld(x_path,y_path);
                        geometry_msgs::PoseStamped position;
                        position.header.frame_id = start.header.frame_id;
                        position.pose.position.x = x_path;
                        position.pose.position.y = y_path;
                        position.pose.orientation.x = 0;
                        position.pose.orientation.y = 0;
                        position.pose.orientation.z = 0;
                        position.pose.orientation.w = 1;

                        plan.push_back(position);
                    }
                    plan.push_back(goal);

                    publishPlan(plan);

                    return true;
                }
                else{
                    return false;
                }
            }

    void PlanPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path){
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        gui_path.header.frame_id = "map";
        gui_path.header.stamp = ros::Time::now();

        for(unsigned int i = 0; i < path.size();i++){
            gui_path.poses[i] = path[i];
        }
        plan_pub_.publish(gui_path);
    }
    bool PlanPlugin::InGridMap(float &x,float &y){
        if(x < origin_x_ || y < origin_y_ || x > origin_x_ + (width_ * resolution_) || y > origin_y_ + (height_ * resolution_)){
            return false;
        }
        return true;
    }
    void PlanPlugin::FromWorldToGrid(float &x,float &y){
        x = static_cast<size_t>((x - origin_x_) / resolution_);
        y = static_cast<size_t>((y - origin_y_) / resolution_);
    }
    void PlanPlugin::FromGridToWorld(float &x,float &y){
        x = origin_x_ + (x * resolution_) + node_center_set;
        y = origin_y_ + (y * resolution_) + node_center_set;
    }
    size_t PlanPlugin::ToIndex(float x, float y){
        return y * width_ + x;
    }
    void PlanPlugin::FromIndex(size_t index ,int &x,int &y){
        x = index % width_;
        y = std::floor(index / width_);
    }
}