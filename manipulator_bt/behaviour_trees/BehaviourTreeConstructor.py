from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Selector, Sequence
from py_trees.decorators import Repeat, EternalGuard, FailureIsSuccess
from py_trees.trees import BehaviourTree
from py_trees.display import unicode_tree, render_dot_tree
import xml.etree.ElementTree as ET

class BehaviourTreeConstructor():
    def __init__(self):
        pass

    def createNodeFromFile(self, folder_path, root_path, tasks_dict, funcs_dict):
        xml_path = folder_path + root_path
        # print("TEST");
        # print(">>", xml_path)
        root = self.parseXMLToTreeElement(xml_path)
        root_node = self.build_node(root, folder_path, tasks_dict, funcs_dict)
        return root_node

    def parseXMLToTreeElement(self, xml_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        return root

    def build_node(self, xml_el, folder_path, tasks_dict, funcs_dict):
        if (xml_el.tag == "SubTree"):
            newNode = self.createNodeFromFile(folder_path, xml_el.attrib["path"], tasks_dict, funcs_dict)
            return newNode

        children = []
        for xml_child in xml_el:
            children.append(self.build_node(xml_child, folder_path, tasks_dict, funcs_dict))

        # Default Name
        if ("name" in xml_el.attrib):
            name = xml_el.attrib["name"]
        else:
            name = xml_el.tag

        if(xml_el.tag == "Selector"):
            return Selector(name=name, memory=True, children=children)
        elif(xml_el.tag == "Sequence"):
            return Sequence(name=name, memory=True, children=children)
        elif(xml_el.tag == "Repeat"):
            if("num_success" in xml_el.attrib):
                return Repeat(name=name, child= children[0], num_success=int(xml_el.attrib["num_success"]))
            return Repeat(name=name, child= children[0], num_success=-1)
        elif(xml_el.tag == "EternalGuard"):
            return EternalGuard(name=name, child=children[0], condition=funcs_dict[xml_el.attrib["cond"]])
        elif(xml_el.tag == "FailureIsSuccess"):
            return FailureIsSuccess(name=name, child=children[0])
        elif(xml_el.tag == "Guard"):
            if("invert" in xml_el.attrib):
                return tasks_dict["Guard"](name="Guard(not"+ xml_el.attrib["cond"] + ")", cond=funcs_dict[xml_el.attrib["cond"]], invert=xml_el.attrib["invert"])
            return tasks_dict["Guard"](name="Guard("+ xml_el.attrib["cond"] + ")", cond=funcs_dict[xml_el.attrib["cond"]])

        # print(xml_el.attrib)
        if not("name" in xml_el.attrib):
            return tasks_dict[xml_el.tag](name=xml_el.tag, **xml_el.attrib)
        return tasks_dict[xml_el.tag](**xml_el.attrib)