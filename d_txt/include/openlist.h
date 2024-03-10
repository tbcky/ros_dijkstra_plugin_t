#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include <limits>

class OpenList{
    public:
        OpenList(){}
        void put(int value ,double key){
            heap_.push_back(std::make_pair(key,value));
            heap_dict_[value] = key;
            //对小堆排序，插入的原数会先排最后，后该数索引与其父节点对比大小，小于其父会互换位置
            std::push_heap(heap_.begin(), heap_.end(), std::greater<>()); // 最小堆排序
        }
        int get(){
        //把堆顶放到最后
            std::pop_heap(heap_.begin(), heap_.end(), std::greater<>());
            auto top = heap_.back();//提取最后值
            heap_.pop_back();//删除最后值
            heap_dict_.erase(top.second);
            return top.second;
        }
        void remove(int value){
            this->up_key(value,-std::numeric_limits<double>::max());//输出该类型的可能最大值
            get();
        }
        int size(){ return heap_.size(); }
        bool is_empty(){ return size() == 0; }
        std::pair<double ,int> max_key(){ return heap_[0];}
        // `find()` 方法用于在哈希表中搜索一个键。如果找到与 `value` 相匹配的键，它将返回一个指向该键值对的迭代器；否则，它将返回 `heap_dict_.end()`
        bool have_value(int value){ return heap_dict_.find(value) != heap_dict_.end();}
        double get_key(int value){ return heap_dict_[value];}
        void up_key(int value,double new_key){
            int idx = -1;
            for(int i = 0; i < heap_.size(); i++){
                if(heap_[i].second == value){
                    idx = i;
                    break;
                }
            }

            int old_key = heap_[idx].first;
            heap_[idx] = std::make_pair(new_key, value);
            heap_dict_[value] = new_key;
            //std::greater<>()如果x大于y,则返回true不改变排序，反之则会
            std::make_heap(heap_.begin(), heap_.end(), std::greater<>());//重新比较堆排列

        }


    private:
        std::vector<std::pair<double,int>> heap_ = {};
        std::unordered_map<int,double> heap_dict_ = {};
};
