import py_trees
from std_msgs.msg import Bool 

class WaitForCharge(py_trees.behaviour.Behaviour):
    def __init__(self,name,node):
        super().__init__(name)
        self.node = node
        self.battery_low = False 

    def setup(self, **kwargs):
        self.node.create_subscription(Bool,'/battery_level_low',self.battery_callback,10)

    def update(self):
        if self.battery_low:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS
    def battery_callback(self,msg):
        self.battery_low=msg.data 