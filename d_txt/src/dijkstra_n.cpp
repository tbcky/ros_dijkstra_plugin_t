#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <dijkstraros/PathPlanningPlugin.h>
#include <dijkstraros/PathPlanningPluginResponse.h>
#include <openlist.h>
//#include <pp_msgs/PathPlanningPluginRequest.h>




std::vector<std::pair<int,double>> find_neighbors(int index,int width,int height,const std::vector<int>& costmap,double edge_cost){
                                
    std::vector<std::pair<int,double>> neighbors;
    std::vector<std::pair<int,double>> check;
    double diagonal_cost = edge_cost * 1.41421;
    
    
      // 检查相邻位置是否可通行并添加至check列表
    if (index - width > 0) { check.push_back({index - width, edge_cost}); }
    if ((index - 1) % width > 0) { check.push_back({index - 1, edge_cost}); }
    if (index - width - 1 > 0 && (index - width - 1) % width > 0) {
	check.push_back({index - width - 1, diagonal_cost});
    }
    if (index - width + 1 > 0 && (index - width + 1) % width != (width - 1)) {
	check.push_back({index - width + 1, diagonal_cost});
    }
    if ((index + 1) % width != (width + 1)) { check.push_back({index + 1, edge_cost}); }
    if ((index + width - 1) < height * width && (index + width - 1) % width != 0) {
	check.push_back({index + width - 1, diagonal_cost});
    }
    if (index + width <= height * width) { check.push_back({index + width, edge_cost}); }
    if ((index + width + 1) <= height * width &&
	(index + width + 1) % width != (width - 1)) {
	check.push_back({index + width + 1, diagonal_cost});
    }

    for(const auto &elem : check) {

	if(costmap[elem.first] == 0){
	    neighbors.push_back(elem);
	}
    }
    return neighbors;

}
std::vector<int> dijkstra(int start_index,int goal_index,int width,int height
                        ,const std::vector<int>& costmap,double resolution
                        ,const std::vector<double>& origin){
    OpenList open_list;
    open_list.put(start_index,0);
    std::set<int> closed_list;
    std::unordered_map<int,int> parents;
    bool path_found = false;
    std::unordered_map<int,double> costs;
    costs[start_index] = 0;

    std::vector<int> path;

    ROS_INFO("Dijkstra: starting ==========>>>>>>>>>>>>>>>>");
    while(!open_list.is_empty()) {
        int cur_node = open_list.get();
        closed_list.insert(cur_node);

        if(cur_node == goal_index) {
            path_found = true;
            break;
        }
        auto neighbors = find_neighbors(cur_node,width,height,costmap,resolution);

        for(const auto& neighbor : neighbors){
            int neighbor_index = neighbor.first;
            double neighbor_cost = neighbor.second;
            if(closed_list.find(neighbor_index) != closed_list.end()){ continue;}
            double g_cost = costs[cur_node]+neighbor.second;

            if(open_list.have_value(neighbor_index)){
                if(g_cost < open_list.get_key(neighbor_index)){
                    costs[neighbor_index] = g_cost;
                    parents[neighbor_index] = cur_node;
                    open_list.up_key(neighbor_index,g_cost);
                }
            }else{
                costs[neighbor_index] = g_cost;
                parents[neighbor_index] = cur_node;
                open_list.put(neighbor_index,g_cost);
            }
        }
    }
    ROS_INFO("Dijkstra: Done node in open_list===========++++++++++++");

    if(!path_found){
        ROS_WARN("Dijkstra: No path found ??????????");
        return path;
    }
    if(path_found){
	    int node = goal_index;
	    path.push_back(node);
	    while(node != start_index){
		node = parents[node];
		path.push_back(node);
	    }
    }
    std::reverse(path.begin(),path.end());
    
    return path;

}
bool make_plan(dijkstraros::PathPlanningPlugin::Request& req, 				dijkstraros::PathPlanningPlugin::Response& res){
   
    const std::vector<int>& costmap = req.costmap_ros;
    int width = req.width;
    int height = req.height;
    int start_index = req.start;
    int goal_index = req.goal;

    double resolution = 0.1;

    std::vector<double> origin = {-20.0, -20.0 , 0.0};

    auto start_time = ros::Time::now();

    auto path = dijkstra(start_index,goal_index,width,height,costmap,resolution,origin);
    
    if(path.empty()){
        ROS_WARN("Dijkstra: No plan found??????????");
        res.plan = path;
        
    }else{
        auto end_time = ros::Time::now() - start_time;
        ROS_INFO("\n++++++++++++++++dijkstra Path found++++++++++++++++");
        ROS_INFO("++++++++Time: %.6f +++++++++++++", end_time.toSec());
        ROS_INFO("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

        ROS_INFO("Dijkstra : Path sent to navigation stack");
    }
    res.plan = path;
    ROS_INFO("+++++++++++ Set Dijkstra response++++++++++++++++++++++");

    return true;
}

void clean_shutdown(){
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_ =  nh.advertise<geometry_msgs::Twist>("cmd_vel",5);
    cmd_vel_.publish(geometry_msgs::Twist());
    ros::Duration(1.0).sleep();
}
void shutdown_callback(const ros::TimerEvent&){
    ROS_DEBUG("Shutting down");
}
int main(int argc,char** argv) {
    ros::init(argc,argv,"dijkstra_path_planning_service_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer make_plan_server_ =  nh.advertiseService("/move_base/SrvClientPlugin/make_plan",make_plan); //不需要自己添加服务类型，会自动推断回调函数类型
    ros::Publisher cmd_vel_ = nh.advertise<geometry_msgs::Twist>("cmd_vel",5);

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ros::shutdown();
    ros::waitForShutdown();
    clean_shutdown();

    ros::Timer shutdown_timer_ = nh.createTimer(ros::Duration(2.0),shutdown_callback,true,false);
    
    return 0;
}
