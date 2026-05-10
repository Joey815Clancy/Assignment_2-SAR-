import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros

from sar_bt.behaviours.navigate_to import NavigateTo
from sar_bt.behaviours.spin_search import SpinSearch
from sar_bt.behaviours.fire_safety import FireSafety
from sar_bt.behaviours.move_from_fire import MoveFromFire
from sar_bt.behaviours.battery_check import BatteryOk
from sar_bt.behaviours.wait_for_charge import WaitForCharge
from sar_bt.behaviours.scan_dam import ScanDam
from sar_bt.behaviours.survivour_tf import SurvivorTf 
from sar_bt.behaviours.wait import Wait


class MissionBT(Node):
    def __init__(self):
        super().__init__('mission_bt')
        self.tree = self.build_tree()
        self.tree.setup(timeout=15)
        self.create_timer(0.1, self.tick)

    def build_tree(self):
        # SpinSearch wrapped in OneShot so it only runs once
        spin = py_trees.decorators.OneShot(
            name='SpinOnce',
            child=SpinSearch('SpinSearch', self),
            policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
        )

        # Fire Safety Fallback
        fire = py_trees.composites.Selector('FireSafety', memory=False)
        fire.add_children([
            FireSafety('FireCheck', self),
            MoveFromFire('MoveFromFire', self)
        ])

        # Battery Fallback
        battery = py_trees.composites.Selector('Battery', memory=False)
        battery.add_children([
            BatteryOk('BatteryOk', self),
            py_trees.composites.Sequence('Charge', memory=True, children=[
                NavigateTo('GoToDock', self, 'docking_station'),
                WaitForCharge('WaitForCharge', self)
            ])
        ])

        # Task 1: go to survivor, wait 1s, broadcast TF, fetch kit, return
        task1 = py_trees.composites.Sequence('Task1', memory=True)
        task1.add_children([
            NavigateTo('GoToSurvivor', self, 'survivor', stopping_distance=1.0),
            Wait('WaitAtSurvivor', duration=1.0),
            SurvivorTf('SurvivorTF', self),
            NavigateTo('GoToMedKit', self, 'medical_kit'),
            NavigateTo('ReturnToSurvivor', self, 'survivor', stopping_distance=1.0)
        ])

        # Task 2: go to dam and scan it
        task2 = py_trees.composites.Sequence('Task2', memory=True)
        task2.add_children([
            NavigateTo('GoToDam', self, 'dam', stopping_distance=0.5),
            ScanDam('ScanDam', self)
        ])

        # Task 5: navigate to exit
        task5 = py_trees.composites.Sequence('Task5', memory=True)
        task5.add_children([
            NavigateTo('GoToExit', self, 'exit')
        ])

        # Root sequence
        root = py_trees.composites.Sequence('Mission', memory=True)
        root.add_children([
            spin,
            fire,
            battery,
            task1,
            task2,
            task5
        ])

        return py_trees_ros.trees.BehaviourTree(root)

    def tick(self):
        self.tree.tick()
        tip = self.tree.root.tip()
        if tip:
            self.get_logger().info(f'BT active: {tip.name} [{tip.status}]')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MissionBT()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
