import numpy as np
import time

class nebula:

    def __init__(self, userName, clrDictionary, maxStepSize, maxTime):
        self.name = userName # your object will be given a user name, i.e. your group name
        self.maxStep = maxStepSize # maximum length of the returned path from run()
        self.maxTime = maxTime # run() is supposed to return before maxTime

    def run(self, img, info):
        colorz = {
        'black':((1,1,1), 0, 13),
        'clr100':((225, 1, 1), 100, 1),
        'clr50':((1, 255, 1), 50, 2),
        'clr30':((1, 1, 255), 30, 2),
        'clr20':((200, 200, 1), 20, 2),
        'clr10':((255, 1, 255), 10, 2),
        'clr9':((1, 255, 255), 9, 3),
        'clr8':((1,1,150), 8, 3),
        'clr7':((120,120,40), 7, 3),
        'clr6':((150,1,150), 6, 3),
        'clr5':((1,150,150), 5, 3),
        'clr4':((222,55,222), 4, 3),
        'clr3':((1, 99, 55), 3, 3),
        'clr2':((200, 100, 10),2, 3),
        'clr1':((100, 10, 200),1, 3)
        }
        myinfo = info[self.name]
        imS = img.shape[0] # assume square image and get size
        loc, game_point = info[self.name]
        y,x = loc # get current y,x coordinates
        yonepix,xonepix = y//50+2,x//50+2
        # a very simple randomizer
        maxL = self.maxStep # total travel
        newMap = np.zeros((19,19),dtype=int)
        for i in range(7):
            for j in range(7):
                if img[75+100*i,75+100*j,0] == colorz['clr100'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr100'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr100'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr100'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr50'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr50'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr50'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr50'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr30'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr30'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr30'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr30'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr20'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr20'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr20'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr20'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr10'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr10'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr10'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr10'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr9'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr9'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr9'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr9'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr8'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr8'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr8'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr8'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr7'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr7'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr7'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr7'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr6'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr6'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr6'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr6'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr5'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr5'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr5'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr5'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr4'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr4'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr4'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr4'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr3'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr3'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr3'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr3'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr2'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr2'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr2'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr2'][1]
                elif img[75+100*i,75+100*j,0] == colorz['clr1'][0][0] and img[75+100*i,75+100*j,1] == colorz['clr1'][0][1] and img[75+100*i,75+100*j,2] == colorz['clr1'][0][2]:
                    newMap[2*i+3,2*j+3] = colorz['clr1'][1]

        if game_point > 100:
            num_neighbor = 2

            left = max(0,yonepix-num_neighbor)
            right = max(0,yonepix+num_neighbor+1)

            bottom = max(0,xonepix-num_neighbor)
            top = max(0,xonepix+num_neighbor+1)

            sample = newMap[left:right,bottom:top]
            sample[0,0], sample[np.shape(sample)[0]-3,np.shape(sample)[1]-3], sample[0,np.shape(sample)[1]-1], sample[np.shape(sample)[0]-1,0], sample[np.shape(sample)[0]-1,np.shape(sample)[1]-1] = 0, 0, 0, 0, 0

            sample = np.where(sample > game_point, -1, sample)
            max_neighbour = np.max(sample)


            index = np.where(sample == max_neighbour)

            dy = (index[0][0]-np.shape(sample)[0]+3)*50
            dx = (index[1][0]-np.shape(sample)[1]+3)*50
            xtarget = x + dx
            ytarget = y + dy

            if (max_neighbour == 0) or ((xtarget <= 0) or (ytarget <= 0) or (xtarget >= 750) or (ytarget >= 750)):
                xtarget = 375
                ytarget = 375
            coordslist = [[y,xtarget],[ytarget,xtarget]]
        else:
            sample = newMap
            sample[0,0], sample[np.shape(sample)[0]-3,np.shape(sample)[1]-3], sample[0,np.shape(sample)[1]-1], sample[np.shape(sample)[0]-1,0], sample[np.shape(sample)[0]-1,np.shape(sample)[1]-1] = 0, 0, 0, 0, 0

            sample = np.where(sample > game_point, -1, sample)
            max_neighbour = np.max(sample)


            index = np.where(sample == max_neighbour)

            dy = (index[0][0]-yonepix)*50
            dx = (index[1][0]-xonepix)*50
            xtarget = x + dx
            ytarget = y + dy
            if ytarget <= 0:
                dy = 50
            if ytarget >= 750:
                dy = -50
            if xtarget <= 0:
                dx = 50
            if xtarget >= 750:
                dx = -50

            xtarget = x + dx
            ytarget = y + dy

            coords = astar(sample,(yonepix,xonepix),(index[0][0],index[1][0]), max_neighbour)
            coordslist = []
            for i in coords:
              new_list = [(j * 50)-75 for j in i ]
              coordslist.append(list(new_list))

        return coordslist


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


def astar(maze, start, end, endvalue):
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
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if (maze[node_position[0]][node_position[1]] != 0) and (maze[node_position[0]][node_position[1]] != endvalue):
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = abs((child.position[0] - end_node.position[0])) + abs((child.position[1] - end_node.position[1]))
            child.f = 0.1*child.g + 0.1*child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)