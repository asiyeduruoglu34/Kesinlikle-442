import numpy as np
import random
import cv2
from skimage import io
import matplotlib.pyplot as plt
from math import ceil, floor


def goalPos(down_img, cur_loc, cur_po, color_codes, info):
    # print(color_codes)
    base_loc = []  # color bases locations
    base_points = []  # color points
    comp_loc = []  # competitors locations
    comp_points = []  # competitors points
    gain_mode = 0

    # find the locations of the colors
    for j in range(len(color_codes)):
        try:
            temp = list(np.where(np.all(down_img == list(color_codes[j][0]), axis=2)))
            # print(temp)
        except:
            temp = [[], []]
            pass
        # print(color_codes[j][0])
        if (len(temp[0]) == 0):
            pass
        elif (len(temp[0]) == 1):
            temp2 = [temp[0][0], temp[1][0]]
            base_loc.append(temp2)
        elif (len(temp[0]) == 2):
            temp2 = [temp[0][0], temp[1][0]]
            base_loc.append(temp2)
            temp2 = [temp[0][1], temp[1][1]]
            base_loc.append(temp2)
        elif (len(temp[0]) == 3):
            temp2 = [temp[0][0], temp[1][0]]
            base_loc.append(temp2)
            temp2 = [temp[0][1], temp[1][1]]
            base_loc.append(temp2)
            temp2 = [temp[0][2], temp[1][2]]
            base_loc.append(temp2)
        for k in range(len(temp[0])):
            base_points.append(color_codes[j][1])

    base_loc_down = []
    ##going too fast on the beginnings
    neighbours = []
    neighbours_po = []
    gn = -100000  # random initial value
    cur_loc_down = [int(cur_loc[0] / 50), int(cur_loc[1] / 50)]
    for j in range(len(base_loc_down)):
        for i in range(2):
            base_loc_down.append(base_loc[j][i])
    ctrl = 1
    if (len(base_points) > 18 and cur_po > 100):
        ctrl = 0
        gain_mode = 1
        if (cur_loc_down[0] % 2 == 0 or cur_loc_down[1] % 2 == 0):
            if (cur_loc_down[0] % 2 == 0 and cur_loc_down[1] % 2 == 1):
                if (cur_loc_down[0] - 1 > 0 or cur_loc_down[0] + 1 < 15):
                    if (cur_loc_down[0] - 1 > 0):
                        try:
                            index1 = base_loc.index([cur_loc_down[0] - 1, cur_loc_down[1]])
                            if (index1 <= 36):
                                neighbours.append([cur_loc_down[0] - 1, cur_loc_down[1]])
                                neighbours_po.append(base_points[index1])
                        except:
                            pass

                    if (cur_loc_down[0] + 1 < 15):
                        try:
                            index2 = base_loc.index([cur_loc_down[0] + 1, cur_loc_down[1]])
                            if (index2 <= 36):
                                neighbours.append([cur_loc_down[0] + 1, cur_loc_down[1]])
                                neighbours_po.append(base_points[index2])
                        except:
                            pass


            elif (cur_loc_down[0] % 2 == 1 and cur_loc_down[1] % 2 == 0):
                if (cur_loc_down[1] - 1 > 0 or cur_loc_down[1] + 1 < 15):
                    if (cur_loc_down[1] - 1 > 0):
                        try:
                            index1 = base_loc.index([cur_loc_down[0], cur_loc_down[1] - 1])
                            if (index1 <= 36):
                                neighbours.append([cur_loc_down[0], cur_loc_down[1] - 1])
                                neighbours_po.append(base_points[index1])
                        except:
                            pass

                    if (cur_loc_down[1] + 1 < 15):
                        try:
                            index2 = base_loc.index([cur_loc_down[0], cur_loc_down[1] + 1])
                            if (index2 <= 36):
                                neighbours.append([cur_loc_down[0], cur_loc_down[1] + 1])
                                neighbours_po.append(base_points[index2])
                        except:
                            pass


            else:

                if (cur_loc_down[0] + 1 < 15 and cur_loc_down[1] + 1 < 15):
                    try:
                        index1 = base_loc.index([cur_loc_down[0] + 1, cur_loc_down[1] + 1])
                        if (index1 <= 36):
                            neighbours.append([cur_loc_down[0] + 1, cur_loc_down[1] + 1])
                            neighbours_po.append(base_points[index1])
                    except:
                        pass
                if (cur_loc_down[0] - 1 > 0 and cur_loc_down[1] + 1 < 15):
                    try:
                        index1 = base_loc.index([cur_loc_down[0] - 1, cur_loc_down[1] + 1])
                        if (index1 <= 36):
                            neighbours.append([cur_loc_down[0] - 1, cur_loc_down[1] + 1])
                            neighbours_po.append(base_points[index1])
                    except:
                        pass
                if (cur_loc_down[0] + 1 < 15 and cur_loc_down[1] - 1 > 0):
                    try:
                        index1 = base_loc.index([cur_loc_down[0] + 1, cur_loc_down[1] - 1])
                        if (index1 <= 36):
                            neighbours.append([cur_loc_down[0] + 1, cur_loc_down[1] - 1])
                            neighbours_po.append(base_points[index1])
                    except:
                        pass
                if (cur_loc_down[0] - 1 > 0 and cur_loc_down[1] - 1 > 0):
                    try:
                        index1 = base_loc.index([cur_loc_down[0] - 1, cur_loc_down[1] - 1])
                        if (index1 <= 36):
                            neighbours.append([cur_loc_down[0] - 1, cur_loc_down[1] - 1])
                            neighbours_po.append(base_points[index1])
                    except:
                        pass

        else:
            if (cur_loc_down[0] + 2 < 15):
                try:
                    index1 = base_loc.index([cur_loc_down[0] + 2, cur_loc_down[1]])
                    if (index1 <= 36):
                        neighbours.append([cur_loc_down[0] + 2, cur_loc_down[1]])
                        neighbours_po.append(base_points[index1])
                except:
                    pass

            if (cur_loc_down[1] - 2 > 0):
                try:
                    index1 = base_loc.index([cur_loc_down[0], cur_loc_down[1] - 2])
                    if (index1 <= 36):
                        neighbours.append([cur_loc_down[0], cur_loc_down[1] - 2])
                        neighbours_po.append(base_points[index1])

                except:
                    pass

            if (cur_loc_down[1] + 2 < 15):
                try:
                    index1 = base_loc.index([cur_loc_down[0], cur_loc_down[1] + 2])
                    if (index1 <= 36):
                        neighbours.append([cur_loc_down[0], cur_loc_down[1] + 2])
                        neighbours_po.append(base_points[index1])
                except:
                    pass

            if (cur_loc_down[0] - 2 > 0):
                try:
                    index1 = base_loc.index([cur_loc_down[0] - 2, cur_loc_down[1]])
                    if (index1 <= 36):
                        neighbours.append([cur_loc_down[0] - 2, cur_loc_down[1]])
                        neighbours_po.append(base_points[index1])
                except:
                    pass

        for j in range(len(base_loc)):
            for i in range(2):
                base_loc[j][i] = int(base_loc[j][i] * 50)
        for j in range(len(neighbours)):
            for i in range(2):
                neighbours[j][i] = 50 * neighbours[j][i]

        # print(neighbours)
        if (len(neighbours) > 0):
            min_loc = neighbours[neighbours_po.index(max(neighbours_po))]
            goal_location = gain(cur_loc, cur_po, min_loc, max(neighbours_po), 0, 0, gain_mode)
            # print(goal_location)
            return [[cur_loc[0], goal_location[1]], [goal_location[0], goal_location[1]]]
        else:
            gain_mode = 0

    if (gain_mode == 0):
        gain_mode = 0
        if (ctrl == 1):
            for j in range(len(base_loc)):
                for i in range(2):
                    base_loc[j][i] = int(base_loc[j][i] * 50)
        if (mode == 1):  # for the initial goal

            try:
                comp_loc.append(list(info['atlas'][0]))
                comp_points.append(info['atlas'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['nebula'][0]))
                comp_points.append(info['nebula'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['mechrix'][0]))
                comp_points.append(info['mechrix'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['tulumba'][0]))
                comp_points.append(info['tulumba'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['ohmygroup'][0]))
                comp_points.append(info['ohmygroup'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['ducati'][0]))
                comp_points.append(info['ducati'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['backspacex'][0]))
                comp_points.append(info['backspacex'][1])
            except:
                pass
            try:
                comp_loc.append(list(info['hepsi1'][0]))
                comp_points.append(info['hepsi1'][1])
            except:
                pass

            gn = -100000  # random initial value
            # sorting algorithm for finding the optimal path
            for i in range(len(base_loc)):
                if (gain(cur_loc, cur_po, base_loc[i], base_points[i], comp_loc, comp_points, gain_mode)[0] > gn):
                    tin = gain(cur_loc, cur_po, base_loc[i], base_points[i], comp_loc, comp_points, gain_mode)
                    gn = tin[0]
                    dummy = tin[1]

            return dummy, base_loc, base_points, tin[2]

        if (mode == 2):  # for the second goal
            # print(base_loc)
            # dummy = [350,350]
            gn = -10000000  # random initial value
            for i in range(len(base_loc)):
                if (abs(int(cur_loc[0] / 50) - int(base_loc[i][0] / 50)) + abs(
                        int(cur_loc[1] / 50) - int(base_loc[i][1] / 50)) < 0.1):
                    # dummy = [350,350]
                    pass
                else:
                    if (gain(cur_loc, cur_po, base_loc[i], base_points[i], comp_loc, comp_points, gain_mode)[0] > gn):
                        tin = gain(cur_loc, cur_po, base_loc[i], base_points[i], comp_loc, comp_points, gain_mode)
                        gn = tin[0]
                        dummy = tin[1]

            return dummy, base_loc, base_points, tin[2]



def gain(current, current_point, goal, goal_point, comp_loc, comp_points, gain_mode):
    if (current_point - goal_point) < 0:
        return [-10000000, 0]
    else:
        p1 = [goal[0] + 4, goal[1] + 4]  # top left
        p2 = [goal[0] + 4, goal[1] + 25]  # top middle
        p3 = [goal[0] + 4, goal[1] + 46]  # top right
        p4 = [goal[0] + 25, goal[1] + 46]  # right middle
        p5 = [goal[0] + 46, goal[1] + 46]  # bottom right
        p6 = [goal[0] + 46, goal[1] + 25]  # bottom middle
        p7 = [goal[0] + 46, goal[1] + 4]  # bottom left
        p8 = [goal[0] + 25, goal[1] + 4]  # left middle
        p = [p1, p2, p3, p4, p5, p6, p7, p8]
        # p = [p1, p3, p5, p7]
        if (gain_mode == 0):
            temp_dist = 1000
            cost = 10000  # initial high cost
            for j in range(len(p)):
                distance = ((current[0] - p[j][0]) ** 2 + (current[1] - p[j][1]) ** 2) ** 0.5
                if (cost > distance):
                    cost = distance
                    goal = p[j]

                for i in range(len(comp_points)):
                    if (goal_point - comp_points[i] > 0):
                        distance2 = ((comp_loc[i][0] - p[j][0]) ** 2 + (comp_loc[i][1] - p[j][1]) ** 2) ** 0.5
                        if (distance2 < temp_dist):
                            temp_dist = distance2


                    else:
                        pass

            # gain = goal_point*2-distance*3.5
            gain = goal_point * 2 - distance * 2.5 + temp_dist * 0.4
            return [gain, goal, goal_point]

        else:
            min_dist = 100000

            for j in range(len(p)):
                distance = ((current[0] - p[j][0]) ** 2 + (current[1] - p[j][1]) ** 2) ** 0.5
                if (distance < min_dist):
                    min_dist = distance
                    tp = p[j]

            return tp



##A* algorithm

def heur(ch_p, en_p):
    h = ((ch_p[0] - en_p[0]) ** 2) + ((ch_p[1] - en_p[1]) ** 2)
    return h


# A* algorithm is also adopted from https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # 4 neighbours

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                    len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                continue

            # node_color = mazeImg[node_position[0], node_position[1], :]
            # Make sure walkable terrain
            # if node_color[0] == 0 and node_color[1] == 0 and node_color[0] == 0:
            # continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)
        # print(node_color)
        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            node_color = maze[node_position[0], node_position[1]]
            # print(node_color)
            # print(maze[start[0], start[1]])

            # Create the f, g, and h values
            # print(current_node.position)
            for j in range(3):
                if ((node_color[j] == maze[start[0], start[1]])[j] or (node_color[j] == maze[end[0], end[1]][j])):
                    child.g = child.g + 1
                else:
                    child.g = child.g + 1000

            child.h = heur(child.position, end_node.position)
            child.f = child.g + child.h
            # print(child.f)

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


def go(current, goal, maxL, cur_po, base_loc, base_points, down_img, img):
    bo = 0

    # down sample the goal and current position
    goal2 = [int(goal[0] / 50), int(goal[1] / 50)]
    current2 = []
    if (current[0] / 50 - int(current[0] / 50) > 0.5):
        current2.append(int(current[0] / 50) + 1)
    else:
        current2.append(int(current[0] / 50))
    if (current[1] / 50 - int(current[1] / 50) > 0.5):
        current2.append(int(current[1] / 50) + 1)
    else:
        current2.append(int(current[1] / 50))
    del_x = goal2[1] - current2[1]
    del_y = goal2[0] - current2[0]

    for j in range(len(base_loc)):
        for i in range(2):
            base_loc[j][i] = int(base_loc[j][i] / 50)
    # print(base_loc)
    # print(del_x+1, del_y+1)
    d_list = []

    for i in range(abs(del_x) + 2):
        for j in range(abs(del_y) + 2):
            try:
                if (del_x >= 0 and del_y >= 0):
                    index = base_loc.index([current2[0] + j, current2[1] + i])
                    d_list.append(base_points[index])
                elif (del_x >= 0 and del_y <= 0):
                    index = base_loc.index([current2[0] - j, current2[1] + i])
                    d_list.append(base_points[index])
                elif (del_x <= 0 and del_y >= 0):
                    index = base_loc.index([current2[0] + j, current2[1] - i])
                    d_list.append(base_points[index])
                elif (del_x <= 0 and del_y <= 0):
                    index = base_loc.index([current2[0] - j, current2[1] - i])
                    d_list.append(base_points[index])
                else:
                    pass
            except:
                pass
    # print(d_list)
    max1, max2 = 0, 0
    try:
        max1 = max(d_list)
        d_list.pop(d_list.index(max1))
        if (len(d_list) != 0):
            max2 = max(d_list)
    except:
        pass

    if (max1 + max2 > cur_po):
        bo = 1
    else:
        bo = 0

    if (bo == 0):
        dist_x = goal[1] - current[1]
        dist_y = goal[0] - current[0]

        nx1 = int(current[1]) + int(dist_x / 2)
        ny1 = int(current[0]) + int(dist_y / 2)
        nx2 = int(nx1) + int(dist_x / 2)
        ny2 = int(ny1) + int(dist_y / 2)

        path = [[int(current[0]), nx1], [ny1, nx1], [ny1, nx2], [ny2, nx2]]
        return path
    else:
        goal2 = [int(goal[0] / 50), int(goal[1] / 50)]
        # current2 = [int(current[0]/50), int(current[1]/50)]
        current2 = []
        if (current[0] / 50 - int(current[0] / 50) > 0.5):
            current2.append(int(current[0] / 50) + 1)
        else:
            current2.append(int(current[0] / 50))
        if (current[1] / 50 - int(current[1] / 50) > 0.5):
            current2.append(int(current[1] / 50) + 1)
        else:
            current2.append(int(current[1] / 50))

        current2 = tuple(current2)
        goal2 = tuple(goal2)

        path = astar(down_img, current2, goal2)

        path2 = []
        for j in range(len(path)):
            path2.append(list(path[j]))

        for j in range(len(path2)):
            for i in range(2):
                path2[j][i] = 50 * path2[j][i]

        current3 = path2[0][0]
        path2.insert(0, [current3, current[1]])

        for j in range(len(path2) - 1):
            for i in range(2):
                if (path2[j][i] == 0 or path2[j][i] == 750):
                    pass
                elif (path2[j][i] % 100 == 0):
                    path2[j][i] = path2[j][i] + 2
                elif (path2[j][i] % 50 == 0):
                    path2[j][i] = path2[j][i] - 2

        return path2


class meturoam:

    def __init__(self, userName, clrDictionary, maxStepSize, maxTime):
        self.name = userName  # your object will be given a user name, i.e. your group name
        self.maxStep = maxStepSize  # maximum length of the returned path from run()
        self.maxTime = maxTime  # run() is supposed to return before maxTime
        global clr
        clr = clrDictionary

    def run(self, img, info):

        ratio = 0.02  # downsample ratio
        down_img = cv2.resize(img, (0, 0), fx=ratio, fy=ratio, interpolation=cv2.INTER_NEAREST)  # downsample the image
        down_img[0], down_img[1], down_img[2] = down_img[2], down_img[1], down_img[0]  # bgr2rgb

        color_codes = []  # create a list then add color informations
        color_codes.append(list(clr['clr100'][:]))
        color_codes.append(list(clr['clr50'][:]))
        color_codes.append(list(clr['clr30'][:]))
        color_codes.append(list(clr['clr20'][:]))
        color_codes.append(list(clr['clr10'][:]))
        color_codes.append(list(clr['clr9'][:]))
        color_codes.append(list(clr['clr8'][:]))
        color_codes.append(list(clr['clr7'][:]))
        color_codes.append(list(clr['clr6'][:]))
        color_codes.append(list(clr['clr5'][:]))
        color_codes.append(list(clr['clr4'][:]))
        color_codes.append(list(clr['clr3'][:]))
        color_codes.append(list(clr['clr2'][:]))
        color_codes.append(list(clr['clr1'][:]))
        # print(color_codes)

        imS = img.shape[0]  # assume square image and get size

        # get current location
        loc, game_point = info[self.name]

        y, x = loc  # get current y,x coordinates
        maxL = self.maxStep  # total travel
        global mode
        myinfo = info[self.name]
        try:

            # print(down_img[int(y/50),int(x/50)])
            mode = 1
            temp = goalPos(down_img, [y, x], game_point, color_codes, info)

            if (len(temp) < 3):
                # print("runmode")
                return temp
            else:
                if (len(temp[2]) > 1):
                    # print("normal")
                    goal = temp[0]
                    path = go([y, x], goal, maxL, game_point, temp[1], temp[2], down_img, img)

                    if (abs(goal[0] - y) + abs(goal[1] - x) < maxL and len(temp[2]) > 1):
                        mode = 2
                        current = goal
                        temp2 = goalPos(down_img, current, game_point, color_codes, info)
                        goal = temp2[0]
                        if (temp[3] + temp2[3] > game_point):
                            pass
                        else:
                            path2 = go(current, goal, maxL, game_point, temp2[1], temp2[2], down_img, img)
                            for j in range(len(path2)):
                                path.append(path2[j])
                            mode = 1
                    # print(path)
                    return path
                else:
                    # print("beyaza ??ektim")
                    if (y % 100 < 50):
                        return [[y, x], [y, x]]
                    else:
                        if (y % 50 < 25):
                            return [[y - 26, x], [y - 26, x]]
                        else:
                            return [[y + 26, x], [y + 26, x]]




        except:
            # print("beyaza ??ektim")
            if (y % 100 < 50):
                return [[y, x], [y, x]]
            else:
                if (y % 50 < 25):
                    return [[y - 26, x], [y - 26, x]]
                else:
                    return [[y + 26, x], [y + 26, x]]




