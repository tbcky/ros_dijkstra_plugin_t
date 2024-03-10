#ifndef SRV_CLIENT_PLUGIN_H_
#define SRV_CLIENT_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h> //可发布到ROS代价地图
#include <costmap_2d/costmap_2d.h>    //储存的代价地图
#include <nav_core/base_global_planner.h>    //全局规划必须要继承的类

#include <geometry_msgs/PoseStamped.h>  //机器人位置姿态的类型

#include <base_local_planner/world_model.h>  //机器人感知环境变化
#include <base_local_planner/costmap_model.h>  //机器人周围代价地图

#include "dijkstraros/PathPlanningPlugin.h"   //自定义服务消息类型

// 以下是命名空间的开始，将所有的类、函数等包含在一个命名空间内以防止命名冲突。
namespace srv_client_plugin{
  /**
   * @class SrvClientPlugin
   * @brief A global planner that creates service request for a plan and forwards the response to the move_base global planner module
   * 一个全局计划器，用于创建计划的服务请求并将响应转发到move_base全局计划器模块
   *///创建一个自定义规划器类，继承自BaseGlobalPlanner
  class SrvClientPlugin : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the SrvClientPlugin
       */
      SrvClientPlugin();
      /**
       * @brief  Constructor for the SrvClientPlugin
       * @param  name The name of this planner该规划名
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning指向成本图的ROS包装器的指针，用于规划
       */ 
      // 带参构造函数，接收一个规划器名称和一个指向costmap_2d::Costmap2DROS对象的指针。
      SrvClientPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the SrvClientPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       *///用于初始化规划器。它在创建规划器对象之后、使用规划器之前被调用
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      //这是执行路径规划的主要方法。
      // 计算规划路径的函数：
      // 接收起点和终点的位姿，以及用于存放规划路径的容器，返回是否成功找到路径的布尔值。
      bool makePlan(const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
      * @brief Converts a x,y grid cell coordinate value to a linear index value (one-dimensional array index)
      * @param x Grid cell map x coordinate value网格单元映射x坐标值
      * @param y Grid cell map y coordinate value网格单元映射y坐标值
      * @return index value corresponding to the location on the one-dimensional array representation
      * 与一维数组表示上的位置相对应的索引值
      */
      // 工具函数声明：
      // 将x,y网格坐标转换为线性索引
      size_t ToIndex(float x, float y);

      /**
      * @brief Converts a linear index value to a x,y grid cell coordinate value
      * @param index A linear index value, specifying a cell/pixel in an 1-D array
      * 线性索引值，指定一维数组中的单元/像素
      * @param x Grid cell map x coordinate value网格单元映射x坐标值
      * @param y Grid cell map y coordinate value网格单元映射y坐标值
      */
      // 将线性索引转换为x,y网格坐标。
      void FromIndex(size_t index, int &x, int &y);

      /**
      * @brief Converts x,y values in the world frame (in meters) into x,y grid map coordinates
      *        This transformation is derived from the map resolution and adjusts 
      *        w.r.t the location of the map origin
      * 将世界坐标系中的x，y值（以米为单位）转换为x，y网格地图坐标。此转换源自地图分辨率和调整地图原点的位置
      * @param x X-Axis value in the world frame of reference (in meters)世界参考系中的X轴值（以米为单位）
      * @param y Y-Axis value in the world frame of reference (in meters)世界参考系中的y轴值（以米为单位）
      */
     // 将世界坐标转换为网格坐标。
      void FromWorldToGrid(float &x, float &y);

      /**
      * @brief Converts x,y grid cell coordinates to world coordinates (in meters)
      *        This transformation is derived from the map resolution, adjusts
      *        w.r.t the location of the map origin and can include an offset
      *        to place the world coordinate at the center point of a grid cell
      *将x，y网格单元坐标转换为世界坐标（以米为单位）此转换源自地图分辨率，调整地图原点的位置，并可以包括将世界坐标放置在网格单元中心点的偏移量
      * @param x Grid cell map x coordinate value网格单元映射x坐标值
      * @param y Grid cell map y coordinate value网格单元映射y坐标值
      */
     // 将网格坐标转换为世界坐标。
      void FromGridToWorld(float &x, float &y); 

      /**
      * @brief Checks if world coordinates are inside grid map bounds检查世界坐标是否在栅格地图边界内
      * @param x X-Axis value in the world frame of reference (in meters)世界参考系中的X轴值（以米为单位）
      * @param y Y-Axis value in the world frame of reference (in meters)世界参考系中的y轴值（以米为单位）
      * @return true if a index is in map bounds, otherwise false如果索引在映射边界内，则为true，否则为false
      */
     // 判断世界坐标是否在网格地图范围内。
     //确保路径规划算法仅在已知的地图区域内进行计算，避免越界错误，提高规划的可靠性和安全性。
      bool InGridMapBounds(float &x, float &y);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_; // 指向ROS包装的代价地图的指针
      costmap_2d::Costmap2D* costmap_;   // 指向代价地图的指针。
      bool initialized_;                 // 表示规划器是否已初始化的标志。
      // x,y position (in meters) of grid map origin w.r.t world's coordinate origin
      //栅格地图原点相对于世界坐标原点的x，y位置（以米为单位）
      float origin_x_;                 
      float origin_y_;
      // the resolution of the map which is expressed in meters per pixel
      // or the size of each grid cell (pixel) in meters
      // 0.05 means for example 5 centimers for each cell (pixel)
      //地图的分辨率，以米/像素表示
      //或每个网格单元（像素）的大小（以米为单位）
      //0.05意味着例如每个单元（像素）的5厘米
      float resolution_;    // 地图分辨率，以米为单位的每个像素或网格单元。
      // by default path is created along the corners/edges of grid cells
      //默认情况下，路径是沿着网格单元的角/边创建的
      bool path_at_node_center = false;      // 路径是否通过网格单元中心的标志。
      float node_center_offset_ = 0;          // 网格单元中心的偏移量。
      // map dimentions in number of grid cells
      //以网格单元的数量映射维度
      // 地图的宽度和高度，以网格单元数表示。
      int width_;
      int height_;
      // 地图的总大小，以网格单元数表示
      int map_size_;
      // service client declaration
       // ROS服务客户端，用于请求规划服务。
      ros::ServiceClient makeplan_service_;
       /**
       * @brief Publishes the global plan to display it's entire lenght in RVIZ
       * 发布全局平面以在RVIZ中显示其整个长度
       * @param path The plan as filled by the planner
       * 规划器实现规划
       */
      // 函数用于在RVIZ中发布全局路径，以便可视化展示。
      void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
      ros::Publisher plan_pub_;     // ROS发布者，用于发布路径。

      std::string name1;
  };
}  
#endif
