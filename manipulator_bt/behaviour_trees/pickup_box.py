#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Selector, Sequence
from py_trees.decorators import Repeat
from py_trees.trees import BehaviourTree
from py_trees.display import unicode_tree, render_dot_tree
from py_trees.visitors import SnapshotVisitor
import xml.etree.ElementTree as ET

from BehaviourTreeConstructor import BehaviourTreeConstructor

class DebugPrint(Behaviour):
    def __init__(self, name, msg):
        super(DebugPrint, self).__init__(name)
        self.msg = msg

    def update(self):
        print(self.msg)

        return Status.SUCCESS
    
class BTNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('My Node has started')
        
        task_dict = {
            "DebugPrint": DebugPrint
        }
        func_dict = {}

        BTC = BehaviourTreeConstructor()
        xml_file = os.path.join(get_package_share_directory('manipulator_bt'), 'behaviour_trees', 'pickup_box.xml')
        root = BTC.createNodeFromFile(xml_file, "", task_dict, func_dict)
        
        self.tree = BehaviourTree(root)

    def run(self):
        try:
            while rclpy.ok():
                self.tree.tick()
                rclpy.spin_once(self)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown() 




def main(args=None):




    rclpy.init(args=args)
    node = BTNode()
    node.run()

if __name__ == '__main__':
    main()