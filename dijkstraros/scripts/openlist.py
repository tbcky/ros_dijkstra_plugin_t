#! /usr/bin/env python

"""
Open List module
Author: Roberto Zegers R.
Date: Feb 22, 2021
"""

import heapq  #堆队列

class OpenList(object):
    def __init__(self):
        #用于存储待处理的节点。heapq.heappush将节点插入堆中，
        self.heap = [] #初始化了空堆
        #为了快速查找和更新节点的优先级
        #在Dijkstra算法中，每个节点的优先级是从起始节点到该节点的最短路径的长度
        self.heap_dict = dict() #初始化了空字典

    #向OpenList中添加元素的方法。
    #该方法首先将元素及其优先级存储在一个元组中，然后使用 heappush() 函数将元组添加到堆中。
    #传入的参数是元素（item）及其优先级（priority）。元素和其优先级以Tuple的形式被添加到堆中，同时在字典中也添加键（item）和值（priority）的映射。
    def put(self, item, priority):
        #根据元组的第一个元素 priority 进行比较，将元组插入到堆中的适当位置，以确保堆的特性得到维护。堆的特性是，每个节点都比其子节点小（或大，具体取决于堆的类型），即堆中的最小元素（或最大元素）总是位于根节点。
        heapq.heappush(self.heap, (priority, item))#将 (priority, item) 元组插入到堆中的操作
        #这个字典的目的是为了在常数时间内查找和更新项目的优先级。
        self.heap_dict[item] = priority

    #获取并删除具有最高优先级的项目
    def get(self):
        #从堆中删除并返回具有最高优先级的项目。
        _, item = heapq.heappop(self.heap)
        #从字典 `heap_dict` 中删除与项目 `item` 关联的优先级。
        self.heap_dict.pop(item)
        return item
    
    #从 OpenList 中删除一个项目。
    def remove(self, item):
        self.decrease_key(item, (-float('inf'), -float('inf')))
        self.get()
    #查看具有最高优先级的元素，但不从队列中移除它。
    def peek(self):
        return self.heap[0]
    #检查队列是否为空。
    def empty(self):
        return len(self.heap) == 0
    #返回队列中元素的数量。
    def size(self):
        return len(self.heap)
    #检查队列中是否包含指定的元素。
    def contains(self, item):
        return item in self.heap_dict
    #获取指定元素的优先级。
    def get_priority(self, item):
        return self.heap_dict[item]

    #更新某个项目的优先级
    def decrease_key(self, item, new_priority):
        idx = None
        for i, val in enumerate(self.heap):#这里i是堆中的位置，val是元组
            if val[1] == item:   #这里的val[1]是元组的第二个元素
                idx = i
                break

        old_priority = self.heap[idx][0]
        self.heap[idx] = (new_priority, item)
        self.heap_dict[item] = new_priority
        #使用 `heapq._siftdown()` 函数重新调整堆的结构，以确保堆的特性得到维护。
        heapq._siftdown(self.heap, 0, idx)#从heap[idx]向最高优先级访问，重新排序获得节点的四周节点的最小数值排在第一位
        #由于heap是存放节点的四周节点的，需要排序出最短节点
        #这条语句的目的是将某个项目的优先级更新为负无穷大，以便该项目在优先级队列中具有最高优先级，然后从优先级队列中删除该项目。