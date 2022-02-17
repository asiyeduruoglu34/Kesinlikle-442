import numpy as np
import time


class hepsi1:

    def __init__(self, userName, clrDictionary, maxStepSize, maxTime):
        self.name = userName  # your object will be given a user name, i.e. your group name
        self.maxStep = maxStepSize  # maximum length of the returned path from run()
        self.maxTime = maxTime  # run() is supposed to return before maxTime

    def run(self, img, info):
        colorz = {
            'black': ((1, 1, 1), 0, 13),
            'clr100': ((225, 1, 1), 100, 1),
            'clr50': ((1, 255, 1), 50, 2),
            'clr30': ((1, 1, 255), 30, 2),
            'clr20': ((200, 200, 1), 20, 2),
            'clr10': ((255, 1, 255), 10, 2),
            'clr9': ((1, 255, 255), 9, 3),
            'clr8': ((1, 1, 150), 8, 3),
            'clr7': ((120, 120, 40), 7, 3),
            'clr6': ((150, 1, 150), 6, 3),
            'clr5': ((1, 150, 150), 5, 3),
            'clr4': ((222, 55, 222), 4, 3),
            'clr3': ((1, 99, 55), 3, 3),
            'clr2': ((200, 100, 10), 2, 3),
            'clr1': ((100, 10, 200), 1, 3)
        }
        myinfo = info[self.name]
        imS = img.shape[0]  # assume square image and get size
        loc, game_point = info[self.name]
        y, x = loc  # get current y,x coordinates
        yonepix, xonepix = y // 50, x // 50
        newMap = np.zeros((15, 15), dtype=int)
        distanceMap = np.zeros((15, 15), dtype=int)
        neighborMap = np.zeros((15, 15), dtype=int)
        enemy_map = np.zeros((15, 15))
        Target_map = np.zeros((15, 15))
        Target_map2 = np.zeros((15, 15))
        for i in range(7):
            for j in range(7):
                if img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr100'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr100'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr100'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr100'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr50'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr50'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr50'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr50'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr30'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr30'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr30'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr30'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr20'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr20'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr20'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr20'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr10'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr10'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr10'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr10'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr9'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr9'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr9'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr9'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr8'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr8'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr8'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr8'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr7'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr7'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr7'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr7'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr6'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr6'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr6'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr6'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr5'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr5'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr5'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr5'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr4'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr4'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr4'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr4'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr3'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr3'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr3'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr3'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr2'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr2'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr2'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr2'][1]
                elif img[75 + 100 * i, 75 + 100 * j, 0] == colorz['clr1'][0][0] and img[
                    75 + 100 * i, 75 + 100 * j, 1] == colorz['clr1'][0][1] and img[75 + 100 * i, 75 + 100 * j, 2] == \
                        colorz['clr1'][0][2]:
                    newMap[2 * i + 1, 2 * j + 1] = colorz['clr1'][1]

        other_groups = list(info.keys())
        other_groups.remove(self.name)
        sum = 0

        for i in range(7):
            for j in range(7):
                # neighborMap[2*i+1,2*j+1] =  newMap[2*i+1,2*j+1]*(0.75*(newMap[2*i+1-2,2*j+1-2] + newMap[2*i+1+2,2*j+1-2] + newMap[2*i+1-2,2*j+1+2] + newMap[2*i+1+2,2*j+1+2]) + newMap[2*i+1,2*j+1-2] + newMap[2*i+1,2*j+1+2] + newMap[2*i+1-2,2*j+1] + newMap[2*i+1+2,2*j+1])
                distanceMap[2 * i + 1, 2 * j + 1] = (abs(75 + 100 * i - y) + abs(75 + 100 * j - x)) / 50
                Target_map[2 * i + 1, 2 * j + 1] = newMap[2 * i + 1, 2 * j + 1] / distanceMap[2 * i + 1, 2 * j + 1] ** 2
                # Target_map2[2*i+1,2*j+1] = (newMap[2*i+1,2*j+1] + 0.05*neighborMap[2*i+1,2*j+1]) / distanceMap[2*i+1,2*j+1]**2

                for gInd, gName in enumerate(other_groups):
                    sum = sum + abs(info[gName][0][0] - (100 * i + 75)) + abs(info[gName][0][1] - (100 * j + 75))
                enemy_map[2 * i + 1, 2 * j + 1] = sum
                sum = 0

        Target_map = np.nan_to_num(Target_map)
        Target_map = np.where(Target_map > 999999, 0, Target_map)
        guzel_map = np.zeros((7, 7))
        for i in range(7):
            for j in range(7):
                guzel_map[i, j] = Target_map[2 * i + 1, 2 * j + 1]
        np.set_printoptions(precision=2)

        Target_map2 = np.nan_to_num(Target_map2)
        Target_map2 = np.where(Target_map2 > 999999, 0, Target_map2)
        guzel_map2 = np.zeros((7, 7))
        for i in range(7):
            for j in range(7):
                guzel_map2[i, j] = Target_map2[2 * i + 1, 2 * j + 1]
        np.set_printoptions(precision=2)
        '''    
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

            dy = (index[0][0]-np.shape(sample)[0]+1)*50
            dx = (index[1][0]-np.shape(sample)[1]+1)*50
            xtarget = x + dx
            ytarget = y + dy

            if (max_neighbour == 0) or ((xtarget <= 0) or (ytarget <= 0) or (xtarget >= 750) or (ytarget >= 750)):
                xtarget = 375
                ytarget = 375
            coordslist = [[y,xtarget],[ytarget,xtarget]]
        else:
        '''
        sample = Target_map
        # sample[0,0], sample[np.shape(sample)[0]-3,np.shape(sample)[1]-3], sample[0,np.shape(sample)[1]-1], sample[np.shape(sample)[0]-1,0], sample[np.shape(sample)[0]-1,np.shape(sample)[1]-1] = 0, 0, 0, 0, 0

        sample = np.where(newMap > game_point, -1, Target_map)
        max_neighbour = np.max(sample)

        index = np.where(sample == max_neighbour)

        coords = pathfinder(sample, [yonepix, xonepix], [index[0][0], index[1][0]])
        coordslist = []
        for i in coords:
            new_list = [(j * 50) + 25 for j in i]
            coordslist.append(list(new_list))

        return coordslist


def pathfinder(maze, start, end):
    stepcounter = 0
    current = start
    neighbour_index = [[0, 1], [1, 0], [0, -1], [-1, 0]]
    path = [start]
    while current != end:
        # if stepcounter == 2:
        #    break

        neighbours = []
        distances = []
        for i in neighbour_index:
            for j in range(len(i)):
                if j == 0:
                    y = current[j] + i[j]
                elif j == 1:
                    x = current[j] + i[j]
                    if (y < 15) and (y >= 0) and (x < 15) and (x >= 0):
                        if (maze[y][x] != -1):
                            neighbours.append([y, x])
        for i in neighbours:
            for j in range(len(i)):
                if j == 0:
                    ydistance = abs(end[j] - i[j])
                elif j == 1:
                    xdistance = abs(end[j] - i[j])
                    distances.append(ydistance + xdistance)
        # if stepcounter < 2:
        current = neighbours[distances.index(min(distances))]
        path.append(current)

    return path
