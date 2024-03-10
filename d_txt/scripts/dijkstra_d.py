#!/usr/bin/env python
import rospy
from dijkstraros.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from openlist import OpenList

def find_neighbors(index ,width ,height,costmap,edge_cost):
    neighbors = []
    check = []
    diagonal_cost = edge_cost * 1.41421 #对角代价
    #index = y * width + x
  #如果当前位置以上还有行，则它的正上方的位置是相邻的。计算其索引并将其添加到check列表中。
    if index - width > 0:
        check.append([index - width, edge_cost])
    #如果当前位置不是其行的第一个元素，则它的左边有一个相邻位置。
    if (index - 1) % width > 0:
        check.append([index - 1, edge_cost])
    #检查左上对角线位置是否存在并且不越界。
    if index - width - 1 > 0 and (index - width - 1) % width > 0:
        check.append([index - width - 1, diagonal_cost])
    #检查右上对角线位置是否存在并且不越界。
    if index - width + 1 > 0 and (index - width + 1) % width != (width - 1) :
        check.append([index - width + 1, diagonal_cost])
    #如果当前位置不是其行的最后一个元素，则它的右边有一个相邻位置。
    if (index + 1) % width != (width + 1):
        check.append([index + 1, edge_cost])
    #检查左下对角线位置是否存在并且不越界
    if (index + width - 1) < height * width and (index + width - 1) % width != 0:
        check.append([index + width - 1, diagonal_cost])
    #如果当前位置以下还有行，则它的正下方的位置是相邻的。
    if (index + width) <= height * width:
        check.append([index + width, edge_cost])
    #检查右下对角线位置是否存在并且不越界。
    if (index + width + 1) <= height * width and (index + width + 1) % width != (width - 1):
        check.append([index + width + 1, diagonal_cost])
    #遍历check列表中的所有可能相邻位置。如果某位置在costmap中的成本为0（表示可通行），则将该位置及其移动成本添加到neighbors列表中。
    for element in check:
        if costmap[element[0]] == 0: #如果 costmap 中对应位置的成本为 0（表示该位置可通行）
            neighbors.append(element)
    #返回找到的所有可通行的相邻位置及其移动成本。
    return neighbors
def dijkstra(start_index, goal_index,width,height,costmap,resolution,origin):

    openlist = OpenList()
    openlist.put(start_index,0)
    closed_nodes = set()
    parents = dict()
    path_found = False
    costs = dict()
    costs[start_index] = 0

    path = []

    rospy.loginfo("Dijkstra: Done with initialization")

    while not openlist.is_empty():

        cur_node = openlist.get()
        closed_nodes.add(cur_node)

        if cur_node == goal_index:
            path_found = True
            break
        neighbors = find_neighbors(cur_node,width,height,costmap,resolution)
        for neighbor_idex,step_cost in neighbors:
            if neighbor_idex in closed_nodes:
                continue
            g_cost = costs[cur_node] + step_cost

            if openlist.have_value(neighbor_idex):
                if g_cost < openlist.get_key(neighbor_idex):
                    costs[neighbor_idex] = g_cost
                    parents[neighbor_idex] = cur_node
                    openlist.up_key(neighbor_idex,g_cost)
            else:
                costs[neighbor_idex] = g_cost
                parents[neighbor_idex] = cur_node
                openlist.put(neighbor_idex,g_cost)
    
    rospy.loginfo("Dijkstra: Done nodes in openlist")

    if not path_found:
        rospy.loginfo("Dijkstra: No path found")
        return path
    
    if path_found:
        node = goal_index
        path.append(goal_index)
        while node !=  start_index:
            node = parents[node]
            path.append(node)
    
    path = path[::-1]

    return path


def make_plan(req):
    costmap = req.costmap_ros
    width = req.width
    height = req.height
    start_index = req.start
    goal_index = req.goal

    resolution = 0.1

    origin = [-20.0, -20.0 , 0.0]

    start_time = rospy.Time.now()

    path = dijkstra(start_index, goal_index,width,height,costmap,resolution,origin)

    if not len(path):
        rospy.logwarn("No path found")
        path = []
    else:
        end_time = rospy.Time.now() - start_time

        print("\n")
        rospy.loginfo("+++++++++++dijkstra Path found+++++++++++++")
        rospy.loginfo("++++++++Time: %f +++++++++++++", end_time.to_sec())
        rospy.loginfo("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("\n")
        rospy.loginfo("Dijkstra : Path sent to navigation stack")

    resp = PathPlanningPluginResponse()
    resp.plan = path
    rospy.loginfo("Set Dijkstra response")
    return resp
        
                    

def clean_shutdown(): 
    cmd_vel_.publish(Twist())
    rospy.sleep(1)
    
def shutdown_callback():  
     #rospy.loginfo("Shutting down")
     rospy.signal_shutdown("Shutting down") #会停止接受节点信息和关闭节点

if __name__ == '__main__':
    rospy.init_node("dijkstra_path_planning_service_server",log_level=rospy.INFO,anonymous=False)
    make_plan_service_ = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
    cmd_vel_ = rospy.Publisher("cmd_vel",Twist,queue_size=5)

    rospy.on_shutdown(clean_shutdown)

    while not rospy.core.is_shutdown():
        rospy.rostime.wallsleep(0.5)
    rospy.Timer(rospy.Duration(2),shutdown_callback,oneshot=True)
