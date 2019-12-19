import heapq


class AstarPriorityQueue:
    def __init__(self):
        self.elements = []
        self.elements_dict = {} #just for performance (could probably used ordered_dict..)
        
    def IsEmpty(self):
        return len(self.elements) == 0

    def Insert(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
        self.elements_dict[item.config] = item
        
    def Pop(self):
        item = heapq.heappop(self.elements)
        self.elements_dict.pop(item[1].config)
        return item[1]

    def TopKey(self):
        return heapq.nsmallest(1, self.elements)[0][0]

    def Remove(self, config):
        self.elements = [e for e in self.elements if e[1].config != config]
        heapq.heapify(self.elements)
        
        self.elements_dict.pop(config)
        
    def Contains(self, config):
        return config in self.elements_dict
        
    def GetByConfig(self, config):
        return self.elements_dict[config]
        