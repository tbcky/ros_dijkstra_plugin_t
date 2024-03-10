#! /usr/bin/env python

"""
ROS service for Dijkstra's shortest path algorithm demo
Author: Roberto Zegers R.
Date: Feb 22, 2021
Usage: roslaunch unit1_pp dijkstra_demo.launch
"""

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse #第一个为服务消息请求部分，第二为服务消息回答部分
from geometry_msgs.msg import Twist
from openlist import OpenList
#旨在为给定位置（通过一维索引表示）找到在网格（或地图）上的相邻位置。
#index：当前位置在一维数组（或称为成本地图）中的索引。
#width和height：网格地图的宽度和高度。
#costmap：一个列表或数组，表示每个单元格的成本（0可能表示不可通行）。
#orthogonal_movement_cost：在网格上进行正交移动（即上下左右移动）的成本。
def find_neighbors(index, width, height, costmap, orthogonal_movement_cost):
  #空列表neighbors，用于存储找到的相邻位置及其移动成本。
  neighbors = []
  #空列表check，用于临时存储可能的相邻位置及其移动成本，以便稍后检查这些位置是否可达。
  check = []
  #计算对角线移动的成本。由于对角线移动的距离等于正交移动距离的根号2倍（约等于1.41421），因此对角线移动成本是正交移动成本的1.41421倍。
  diagonal_movement_cost = orthogonal_movement_cost * 1.41421
  #如果当前位置以上还有行，则它的正上方的位置是相邻的。计算其索引并将其添加到check列表中。
  if index - width > 0:
    check.append([index - width, orthogonal_movement_cost])
  #如果当前位置不是其行的第一个元素，则它的左边有一个相邻位置。
  if (index - 1) % width > 0:
    check.append([index - 1, orthogonal_movement_cost])
  #检查左上对角线位置是否存在并且不越界。
  if index - width - 1 > 0 and (index - width - 1) % width > 0:
    check.append([index - width - 1, diagonal_movement_cost])
  #检查右上对角线位置是否存在并且不越界。
  if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
    check.append([index - width + 1, diagonal_movement_cost])
  #如果当前位置不是其行的最后一个元素，则它的右边有一个相邻位置。
  if (index + 1) % width != (width + 1):
    check.append([index + 1, orthogonal_movement_cost])
  #检查左下对角线位置是否存在并且不越界
  if (index + width - 1) < height * width and (index + width - 1) % width != 0:
    check.append([index + width - 1, diagonal_movement_cost])
  #如果当前位置以下还有行，则它的正下方的位置是相邻的。
  if (index + width) <= height * width:
    check.append([index + width, orthogonal_movement_cost])
  #检查右下对角线位置是否存在并且不越界。
  if (index + width + 1) <= height * width and (index + width + 1) % width != (width - 1):
    check.append([index + width + 1, diagonal_movement_cost])
#遍历check列表中的所有可能相邻位置。如果某位置在costmap中的成本为0（表示可通行），则将该位置及其移动成本添加到neighbors列表中。
  for element in check:
    if costmap[element[0]] == 0: #如果 costmap 中对应位置的成本为 0（表示该位置可通行）
      neighbors.append(element)
#返回找到的所有可通行的相邻位置及其移动成本。
  return neighbors

#函数接收一个名为req的参数，它包含了路径规划请求的所有必要信息。
def make_plan(req):
  # costmap_ros：代价地图，用于表示环境中的障碍物和自由空间。
  # width和height：代价地图的宽度和高度。
  # start和goal：分别代表起点和终点在代价地图上的索引。
  costmap = req.costmap_ros
  width = req.width
  height = req.height
  start_index = req.start
  goal_index = req.goal
  resolution = 0.1  #代表地图的每个单元格表示的实际距离
  origin = [-20.0, -20.0, 0.0]  #地图原点的位置
  start_time = rospy.Time.now() #开始执行的时间，这将用来计算路径规划所需的时间。

  path = dijkstra(start_index, goal_index, width, height, costmap, resolution, origin)

  if not path:
    rospy.logwarn("No path returned by Dijkstra")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time #计算执行时间，通过当前时间减去起始时间。
    print("\n")
    rospy.loginfo('++++++++ Dijkstra execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Dijkstra: Path sent to navigation stack')

  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('Dijkstra: Response sent to navigation stack')
  return resp

#用于在一个基于栅格的地图上寻找从一个点到另一个点的最短路径。
# start_index: 起始点的索引。
# goal_index: 目标点的索引。
# width: 地图的宽度。
# height: 地图的高度。
# costmap: 一个列表或数组，表示地图上每个点的通过成本。
# resolution: 地图的分辨率，即每个单元格代表的实际距离。
# origin: 地图的起点坐标。
def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin):
# open_list: 一个优先队列，用来存储待处理的节点，并根据成本排序。代处理节点
# closed_nodes: 一个集合，用来存储已经处理过的节点。已处理
# parents: 一个字典，用来追踪路径的每个节点的前驱节点。父节点
# path_found: 一个布尔值，表示是否找到了路径。
# costs: 一个字典，用来存储从起始点到每个节点的成本。权重
# path: 一个列表，用来存储最终路径的节点索引。存放路径
  open_list = OpenList()
  open_list.put(start_index, 0)
  closed_nodes = set()
  parents = dict()
  path_found = False
  costs = dict()
  costs[start_index] = 0

  path = []

  rospy.loginfo('Dijkstra: Done with initialization')

  while not open_list.empty():
  #开始取出优先级最高的点
    current_node = open_list.get()
    #加入到已经访问过的列表
    closed_nodes.add(current_node)
  #判断是否为目标点
    if current_node == goal_index:
      path_found = True
      break
    #找出该点四周的点
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)
    #处理四周点看看那个符合路径要求
    for neighbor_index, step_cost in neighbors:
      #判断是否是已经访问过的点
      if neighbor_index in closed_nodes:
        continue
      #算出从起始点到该点的成本
      g_cost  = costs[current_node] + step_cost
      #如果邻居已在open_list中并且新的g_cost更小，则更新其成本和父节点，并调整在open_list中的位置。
      if open_list.contains(neighbor_index):
        if g_cost < open_list.get_priority(neighbor_index):
          costs[neighbor_index] = g_cost
          parents[neighbor_index] = current_node
          open_list.decrease_key(neighbor_index, g_cost)
      else: #如果邻居不在open_list中，则将其加入open_list，同时更新成本和父节点。
        costs[neighbor_index] = g_cost
        parents[neighbor_index] = current_node
        open_list.put(neighbor_index, g_cost)

  rospy.loginfo('Dijkstra: Done traversing nodes in open_list')
  #如果没有找到path
  if not path_found:
    rospy.logwarn('Dijkstra: No path found!')
    return path
  #如果找到path
  if path_found:
      node = goal_index
      path.append(goal_index)  #添加目标节点到path中
      while node != start_index:  #直到碰见起始点才停止
          node = parents[node]   #一直循环查找其父节点并加入path
          path.append(node)   #现在path开始是从目标点到起始点
  #反转列表，让从起始点到目标点
  path = path[::-1]

  return path

def clean_shutdown():
  #创建了一个新的Twist消息实例，其所有速度分量默认为0。这意味着没有线速度也没有角速度
  cmd_vel.publish(Twist())
  rospy.sleep(1)

def shutdown_callback():
    rospy.signal_shutdown('Shutting down')

if __name__ == '__main__':
  #第一个为节点名、节点日记的级别，rospy.INFO将记录节点的信息、最后是节点名是否匿名，false表示独一无二、true表示会在后面加随即数字
  rospy.init_node('dijkstra_path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  #服务名、服务信息类型、最后是服务的处理函数
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  #cmd_vel:这是创建的ROS发布者对象的变量名。
  #把Twist类型的数据发布到/cmd_vel上、消息队列长度为5
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  #它用于注册一个回调函数，该函数会在 ROS 节点关闭时被调用。这主要用于在节点终止时执行清理操作，比如关闭文件、保存状态或停止运动控制器等
  #该函数需要自己定义
  rospy.on_shutdown(clean_shutdown)
  #检查ROS节点是否被请求关闭（比如，通过Ctrl+C或者其他节点发送的关闭命令）。
  #is_shutdown()返回一个布尔值，True表示节点已被请求关闭，False表示节点运行正常。
  #没有关闭节点前都是0.5秒休息一次
  while not rospy.core.is_shutdown():
    #wallsleep()：这是rostime模块的一个函数，用于让节点休眠指定的时间（以秒为单位）。
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), shutdown_callback, oneshot=True)
