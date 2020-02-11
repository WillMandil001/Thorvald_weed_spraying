#!/usr/bin/env python
import csv
import rospy
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import yaml
from strands_navigation_msgs.msg import TopologicalNode

# def add_wp_to_map(name, map, pointset, pose, orientation, ):
def add_wp_to_map(node, name, wp, prev_name, next_name, orientation, yamlfile):
    node[0]["meta"]["node"] = name
    node[0]["node"]["name"] = name

    node[0]["node"]["pose"]["position"]["x"] = wp[0]
    node[0]["node"]["pose"]["position"]["y"] = wp[1]

    node[0]["node"]["pose"]["orientation"]["w"] = orientation[0]
    node[0]["node"]["pose"]["orientation"]["x"] = orientation[1]
    node[0]["node"]["pose"]["orientation"]["y"] = orientation[2]
    node[0]["node"]["pose"]["orientation"]["z"] = orientation[3]

    # edges
    node[0]["node"]["edges"][0]["edge_id"] = name + "_" + prev_name
    node[0]["node"]["edges"][0]["node"] = prev_name

    node[0]["node"]["edges"][1]["edge_id"] = name + "_" + next_name
    node[0]["node"]["edges"][1]["node"] = next_name

    empty_node = node[0]["node"]["edges"][2]
    node[0]["node"]["edges"].pop(2)

    documents = yaml.dump(node, yamlfile)
    node[0]["node"]["edges"].append(empty_node)


def end_node_addwp(node, name, next_name, up_name, down_name, wp, orientation, yamlfile):
    node[0]["meta"]["node"] = name
    node[0]["node"]["name"] = name

    node[0]["node"]["pose"]["position"]["x"] = wp[0]
    node[0]["node"]["pose"]["position"]["y"] = wp[1]

    node[0]["node"]["pose"]["orientation"]["w"] = orientation[0]
    node[0]["node"]["pose"]["orientation"]["x"] = orientation[1]
    node[0]["node"]["pose"]["orientation"]["y"] = orientation[2]
    node[0]["node"]["pose"]["orientation"]["z"] = orientation[3]

    # edges
    node[0]["node"]["edges"][0]["edge_id"] = name + "_" + next_name
    node[0]["node"]["edges"][0]["node"] = next_name
    empty_node = node[0]["node"]["edges"][2]
    
    if up_name != None and down_name != None:
        node[0]["node"]["edges"][1]["edge_id"] = name + "_" + up_name
        node[0]["node"]["edges"][1]["node"] = up_name
        node[0]["node"]["edges"][2]["edge_id"] = name + "_" + down_name
        node[0]["node"]["edges"][2]["node"] = down_name
        documents = yaml.dump(node, yamlfile)
    elif up_name != None:
        node[0]["node"]["edges"][1]["edge_id"] = name + "_" + up_name
        node[0]["node"]["edges"][1]["node"] = up_name
        empty_node = node[0]["node"]["edges"][2]
        node[0]["node"]["edges"].pop(2)
        documents = yaml.dump(node, yamlfile)
        node[0]["node"]["edges"].append(empty_node)
    elif down_name != None:
        node[0]["node"]["edges"][1]["edge_id"] = name + "_" + down_name
        node[0]["node"]["edges"][1]["node"] = down_name
        node[0]["node"]["edges"].pop(2)
        documents = yaml.dump(node, yamlfile)
        node[0]["node"]["edges"].append(empty_node)
    else:
        node[0]["node"]["edges"].pop(1)
        node[0]["node"]["edges"].pop(1)
        documents = yaml.dump(node, yamlfile)
        node[0]["node"]["edges"].append(empty_node)
        node[0]["node"]["edges"].append(empty_node)


if __name__ == '__main__':
    with open('extended_sorted_rows.csv', mode='r') as csv_file:
        reader = csv.reader(csv_file)
        # Print every value of every row. 
        wp_list = []
        for row in reader:
            wp_row = []
            for list_ in row:
                wp = list_.split(",")
                wp[0] = float(wp[0].replace('[', '').replace(']', '').replace(' ', ''))
                wp[1] = float(wp[1].replace('[', '').replace(']', '').replace(' ', ''))
                wp_row.append(wp)
            wp_list.append(wp_row)

    with open('/home/will/thorvald/src/Thorvald_weed_spraying/CMP9767M/uol_cmp9767m_tutorial/maps/test__.yaml','r') as formfile:
        node = yaml.load(formfile)

    print node[0]["node"]["edges"][0]
    node[0]["meta"]["map"] = "test"
    node[0]["meta"]["pointset"] = "test"

    node[0]["node"]["map"] = "test"
    node[0]["node"]["pointset"] = "test"

    with open('/home/will/thorvald/src/Thorvald_weed_spraying/CMP9767M/uol_cmp9767m_tutorial/maps/test.yaml','w') as yamlfile:
        n = 2
        for row_name, row in zip(range(0, len(wp_list)), wp_list):

            orientation = [0,0,1,0]
            if row_name % n == 1:
                row.reverse()
                orientation = [0,0,0,1]

            if row_name == 0:
                row_start_end_middle = 0
            elif row_name == len(wp_list) - 1:
                row_start_end_middle = 2
            else:
                row_start_end_middle = 1

            for wp_name, wp in zip(range(0, len(row)), row):
                name = "row" + str(row_name) + "wp" + str(wp_name)
                if wp_name == 0:  # if at the start node:
                    if row_name == 0: # start node with no node above it:
                        next_name = "row" + str(row_name) + "wp" + str(wp_name + 1)
                        up_name = None
                        down_name = "row" + str(row_name + 1) + "wp" + str(len(wp_list[row_name + 1]))
                        end_node_addwp(node = node, name = name, next_name=next_name, up_name= up_name, down_name=down_name, 
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)
                    elif row_name == len(wp_list) - 1:  # start node with no node below it 
                        next_name = "row" + str(row_name) + "wp" + str(wp_name + 1)
                        up_name = "row" + str(row_name - 1) + "wp" + str(len(wp_list[row_name - 1]))
                        down_name = None
                        end_node_addwp(node = node, name = name, next_name=next_name, up_name= up_name, down_name=down_name, 
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)
                    else:  # start node with both node above and below it
                        next_name = "row" + str(row_name) + "wp" + str(wp_name + 1)
                        up_name = "row" + str(row_name - 1) + "wp" + str(len(wp_list[row_name - 1]))
                        down_name = "row" + str(row_name + 1) + "wp" + str(len(wp_list[row_name + 1]))
                        end_node_addwp(node = node, name = name, next_name=next_name, up_name= up_name, down_name=down_name, 
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)

                elif wp_name == len(row) - 1:  # if at the end node:
                    if row_name == 0: # end node with no node above it:
                        prev_name = "row" + str(row_name) + "wp" + str(wp_name - 1)
                        up_name = None
                        down_name = "row" + str(row_name + 1) + "wp" + str(0)
                        end_node_addwp(node = node, name = name, next_name=prev_name, up_name= up_name, down_name=down_name,
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)
                    elif row_name == len(wp_list) - 1:  # end node with no node below it 
                        prev_name = "row" + str(row_name) + "wp" + str(wp_name - 1)
                        up_name = "row" + str(row_name - 1) + "wp" + str(0)
                        down_name = None
                        end_node_addwp(node = node, name = name, next_name=prev_name, up_name= up_name, down_name=down_name,
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)
                    else:  # end node with both node above and below it
                        prev_name = "row" + str(row_name) + "wp" + str(wp_name - 1)
                        up_name = "row" + str(row_name - 1) + "wp" + str(0)
                        down_name = "row" + str(row_name + 1) + "wp" + str(0)
                        end_node_addwp(node = node, name = name, next_name=prev_name, up_name= up_name, down_name=down_name,
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)

                else:
                    prev_name = "row" + str(row_name) + "wp" + str(wp_name - 1)
                    next_name = "row" + str(row_name) + "wp" + str(wp_name + 1)
                    add_wp_to_map(node = node, name = name , prev_name=prev_name, next_name=next_name, 
                                    wp = wp, orientation = orientation, yamlfile = yamlfile)






    # n = 2
    # for i in range(0,len(wp_list)):
    #     orientation = [0,0,1,0]
    #     if i % n == 1:
    #         wp_list[i].reverse()
    #         orientation = [0,0,0,1]
    #     for waypoint in wp_list[i]:
    #         result = add_wp_to_map(waypoint, orientation)



# - meta:
#     map: test
#     node: WayPoint0
#     pointset: test
#   node:
#     edges:
#     - action: move_base
#       edge_id: WayPoint0_WayPoint3
#       inflation_radius: 0.0
#       map_2d: cropped
#       node: WayPoint3
#       recovery_behaviours_config: ''
#       top_vel: 0.55
#     localise_by_topic: ''
#     map: test
#     name: WayPoint0
#     pointset: test
#     pose:
#       orientation:
#         w: 1.0
#         x: 0.0
#         y: 0.0
#         z: 0.0
#       position:
#         x: 0.0
#         y: 0.0
#         z: 0.0
#     verts:
#     - x: 0.689999997616
#       y: 0.287000000477
#     - x: 0.287000000477
#       y: 0.689999997616
#     - x: -0.287000000477
#       y: 0.689999997616
#     - x: -0.689999997616
#       y: 0.287000000477
#     - x: -0.689999997616
#       y: -0.287000000477
#     - x: -0.287000000477
#       y: -0.689999997616
#     - x: 0.287000000477
#       y: -0.689999997616
#     - x: 0.689999997616
#       y: -0.287000000477
#     xy_goal_tolerance: 0.3
#     yaw_goal_tolerance: 0.2