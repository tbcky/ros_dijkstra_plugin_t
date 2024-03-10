import heapq

class OpenList(object):
    def __init__(self):
        self.heap = []
        self.heap_dict = dict()
    def put(self, value, key): #key优先级，value为元素,索引
        heapq.heappush(self.heap,(key, value))
        self.heap_dict[value] = key
        

    def get(self):
        _,value = heapq.heappop(self.heap)
        self.heap_dict.pop(value)
        return value
    
    def remove(self,value):
        self.up_key(value,-float('inf'))
        self.get()
    
    def size(self):
        return len(self.heap)
    
    def is_empty(self):
        return self.size() == 0
    
    def max_key(self):
        return self.heap[0]
    
    def have_value(self, value):
        return value in self.heap_dict
    
    def get_key(self, value):
        return self.heap_dict[value]
    

    def up_key(self,value,new_key):
        idx = None
        for i,val in enumerate(self.heap):
            if val[1] == value:
                idx = i
                break
        
        old_key = self.heap[idx][0]
        self.heap[idx] = (new_key,value)
        self.heap_dict[value] = new_key
        heapq._siftdown(self.heap,0,idx) #可用heapq.heapify(heap)  # 转换完成后，heap满足最小堆的性质



    
