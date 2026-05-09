import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

# Your behaviour nodes
from sar_bt.behaviours.navigate_to import NavigateTo
from sar_bt.behaviours.fire_safety import FireSafety
from sar_bt.behaviours.battery_check import BatteryCheck
from sar_bt.behaviours.scan_dam import ScanDam
from sar_bt.behaviours.publish_tf import PublishTF

class MissionBT(Node):
    def __init__(self):

        # Build tree

        root = py_tree.composites.Sequence(name="Mission",memory=True)

        fire_safety= FireSafety()
        task1 = build_task1_subtree()
        task2 = build_task2_subtree()
        task5 = build_task5_subtree()

        root.add_children([fire_safety, task1, task2, task5])

        # Tick every 100ms

        self.timer = self.create_timer(0.1,self.tick)

    def tick(self.tick):
        self.tree.tick()
def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionBT()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()