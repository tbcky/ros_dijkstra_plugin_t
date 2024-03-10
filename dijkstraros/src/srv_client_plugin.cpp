#include <pluginlib/class_list_macros.h>  //添加插件的头文件
#include <srv_client_plugin.h>   
#include <ros/console.h>   //ros各类信息打印
#include <nav_msgs/Path.h>  //储存规划好的路径

// register this planner as a BaseGlobalPlanner plugin
// 宏：注册插件类。
// 第一个参数是插件类的完全限定名（包括命名空间）。
// 第二个参数是该插件类继承的接口类名。作为全局路径规划器插件使用的类，都应继承自BaseGlobalPlanner并实现其方法。
PLUGINLIB_EXPORT_CLASS(srv_client_plugin::SrvClientPlugin, nav_core::BaseGlobalPlanner)

namespace srv_client_plugin
{

  SrvClientPlugin::SrvClientPlugin()
  {
    initialized_ = false;
  }

  SrvClientPlugin::SrvClientPlugin(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialized_ = false;
    initialize(name, costmap_ros);
  }

  void SrvClientPlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    // 检查插件是否已经被初始化过，避免多次初始化。
    if (!initialized_)
    {// 创建一个私有的节点句柄，它有一个与插件名称相关联的命名空间。
      ros::NodeHandle private_nh("~/" + name);
      // 存储Costmap2DROS对象的指针，以便插件可以与ROS的成本地图交互。
      costmap_ros_ = costmap_ros;
      // 从Costmap2DROS对象中获取底层的成本地图（Costmap2D对象）。
      costmap_ = costmap_ros->getCostmap();
      // 获取成本地图的原点的X和Y坐标。
      origin_x_ = costmap_->getOriginX();
      origin_y_ = costmap_->getOriginY();
      // 获取成本地图的X和Y方向大小，cell单元格
      width_ = costmap_->getSizeInCellsX();
      height_ = costmap_->getSizeInCellsY();
      // 获取成本地图中每个网格单元的分辨率（大小）。
      resolution_ = costmap_->getResolution();
      // 计算成本地图的总大小（以网格单元为单位）。
      map_size_ = width_ * height_;
      name1 = name;

      // create a client for the path planning service
      // 创建一个服务客户端，用于调用路径规划服务。
      makeplan_service_ = private_nh.serviceClient<dijkstraros::PathPlanningPlugin>("make_plan");
      // wait for the service to be advertised and available, blocks until it is.
      // 等待路径规划服务变得可用。这个调用会阻塞，直到服务可用为止。
      makeplan_service_.waitForExistence();
      // create publisher to display the complete trajectory path in RVIZ
      // 创建一个发布者对象，用于将完成的轨迹路径发布到RVIZ中显示。
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      // place path waypoints at the center of each grid cell (vs. at the corners of grid cells)
      // 设置一个布尔变量，以决定路径点是否放在每个网格单元的中心（而不是网格单元的角落）。
      path_at_node_center = true;
      if (path_at_node_center)
      {
        // shift all of the coordinates by half a grid cell
        // 通过将所有坐标偏移半个网格单元的大小，来实现将路径点放在网格单元中心的效果。
        node_center_offset_ = resolution_ / 2;
      }
      // 将插件的初始化状态设置为已初始化。
      initialized_ = true;
    }
  }

    // 接收起点和终点的位姿，以及用于存放规划路径的容器，返回是否成功找到路径的布尔值。
    bool SrvClientPlugin::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
      // 清空之前的规划路径
      plan.clear();
      // clear previously published path in Rviz
      // 在Rviz中清除先前发布的路径
      publishPlan(plan);

      // Fill costmap (costmap is a 1-D array map representation)
      // 将成本地图转换为一维数组表示, 初始化成本地图数组costmap
      std::vector<int> costmap(map_size_);

      // 填充成本地图数组
      for (size_t idx = 0; idx < map_size_; ++idx)
      {
        int x, y;
        x = idx % width_;   // 从索引计算横坐标
        y = std::floor(idx / width_);// 从索引计算纵坐标
        // 获取并设置对应位置的成本
        costmap.at(idx) = static_cast<int>(costmap_->getCost(x, y));
      }

      // 获取起点和终点的世界坐标
      float start_x = start.pose.position.x;
      float start_y = start.pose.position.y;
      float goal_x = goal.pose.position.x;
      float goal_y = goal.pose.position.y;

      size_t start_index = 0;
      size_t goal_index = 0;

      // check if start/goal world coordinates are inside  grid map bounds
      // 检查起点和终点是否在成本地图的边界内
      if (InGridMapBounds(start_x, start_y) && InGridMapBounds(goal_x, goal_y))
      {
        // convert x,y in world coordinates/meters) to x,y in grid map cell coordinates
        // 将世界坐标转换为网格坐标
        FromWorldToGrid(start_x, start_y);
        FromWorldToGrid(goal_x, goal_y);

        // convert 2d representation into flat array index representation
        // 将网格坐标转换为成本地图数组的索引
        start_index = ToIndex(start_x, start_y);
        goal_index = ToIndex(goal_x, goal_y);
      }
      else
      {
        ROS_WARN("Start or goal position outside of the map's boundaries");
        return false;// 如果起点或终点超出边界，返回失败
      }

      // To-Do: check that a start and goal are not obstacles

      // 准备路径规划服务请求
      dijkstraros::PathPlanningPlugin makeplan;
      makeplan.request.costmap_ros = costmap;
      makeplan.request.start = start_index;
      makeplan.request.goal = goal_index;
      makeplan.request.width = width_;
      makeplan.request.height = height_;

      // call path planning service
      // 调用路径规划服务
      makeplan_service_.call(makeplan);

      // 从服务响应中获取规划路径
      std::vector<int> index_plan = makeplan.response.plan;
      //打印路径的字节数
      ROS_DEBUG("Number of points: %d", unsigned(index_plan.size()));
  

      /* Process plan response */
      /* 处理规划路径的响应 */
      if (index_plan.size())
      {
        // insert start node into plan response
        // 在路径响应中插入起点节点
        index_plan.insert(index_plan.begin(), start_index);
        // insert goal node into plan response
        // 在响应末尾插入目标节点
        index_plan.push_back(goal_index);

        for (int p : index_plan)
        {
          int x, y;
          FromIndex(p, x, y); // 将索引转换回网格坐标
          float x_path = static_cast<float>(x);
          float y_path = static_cast<float>(y);

          FromGridToWorld(x_path, y_path);// 将网格坐标转换为世界坐标
          geometry_msgs::PoseStamped position;
          position.header.frame_id = start.header.frame_id;// 设置坐标系
          position.pose.position.x = x_path;
          position.pose.position.y = y_path;
          position.pose.orientation.x = 0;// 设置默认朝向
          position.pose.orientation.y = 0;
          position.pose.orientation.z = 0;
          position.pose.orientation.w = 1;

          plan.push_back(position);// 将位置添加到规划路径中
        }

        plan.push_back(goal);// 确保规划路径以实际目标点结束

        // Publish the path for visualisation
        publishPlan(plan);// 发布路径，以便在RViz中可视化

        return true;
      }
    else
      {
        // no plan found
        return false;
      }
    }

    void SrvClientPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
    {

      // create a message
      // 创建一个nav_msgs::Path类型的消息来存储路径,vector
      nav_msgs::Path gui_path;
      gui_path.poses.resize(path.size());// 调整gui_path的大小以匹配路径的点数
      // 设置消息的frame_id为"map"
      gui_path.header.frame_id = "map"; //基于map进行偏移
      gui_path.header.stamp = ros::Time::now();
      
      // Extract the plan in world coordinates
      // 将路径中的每一个点复制到gui_path消息中
      for (unsigned int i = 0; i < path.size(); i++)
      {
        gui_path.poses[i] = path[i];//gui_path.poses.push_back(path[i]);
      }
      plan_pub_.publish(gui_path); // 发布路径消息
    }

    size_t SrvClientPlugin::ToIndex(float x, float y)
    {
      return y * width_ + x; // 将二维网格坐标转换为一维数组索引
    }

    void SrvClientPlugin::FromIndex(size_t index, int &x, int &y)
    {
      // 将一维数组索引转换回二维网格坐标
      x = index % width_; // 用索引对宽度取模得到x坐标
      y = std::floor(index / width_);// 用索引除以宽度并向下取整得到y坐标
    }

    void SrvClientPlugin::FromWorldToGrid(float &x, float &y)
    {
       // 将世界坐标转换为网格坐标
      x = static_cast<size_t>((x - origin_x_) / resolution_);// 计算相对于原点的x坐标，并除以分辨率
      y = static_cast<size_t>((y - origin_y_) / resolution_);// 计算相对于原点的y坐标，并除以分辨率
    }

    void SrvClientPlugin::FromGridToWorld(float &x, float &y)
    {// 将网格坐标转换为世界坐标
      x = x * resolution_ + origin_x_ + node_center_offset_;// 将网格坐标乘以分辨率，加上原点坐标和节点中心偏移
      y = y * resolution_ + origin_y_ + node_center_offset_;// 将网格坐标乘以分辨率，加上原点坐标和节点中心偏移
    }

    bool SrvClientPlugin::InGridMapBounds(float &x, float &y)
    {// 检查给定的世界坐标是否位于网格地图的边界内
      if (x < origin_x_ || y < origin_y_ || x > origin_x_ + (width_ * resolution_) || y > origin_y_ + (height_ * resolution_))
        return false;// 如果x或y坐标超出边界，返回false
      return true;// 否则，坐标在边界内，返回true
    }

  }; // namespace srv_client_plugin
