"""
Path Planning Sample Code with RRT*

author: AtsushiSakai(@Atsushi_twi)
with edits of Maxim Yastremsky(@MaxMagazin)

"""

import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time

import sys, select, termios, tty

class RRT:
    """
    Class for RRT Planning
    """

    '''
    start - starting coordinates (x, y, radius of cones)
    planDistance -
    obstacleList - coordinates of cones
    expandDis - probably delta?
    
    rttTargets are cones that are a certain distance away from the starting point
    '''
    def __init__(self, start, planDistance, obstacleList, expandDis=0.5, turnAngle=30, maxIter=1000, rrtTargets = None):

        self.start = Node(start[0], start[1], start[2])
        self.startYaw = start[2]

        self.planDistance = planDistance
        self.expandDis = expandDis
        self.turnAngle = math.radians(turnAngle)

        self.maxDepth = int(planDistance / expandDis)
        # print(self.maxDepth)

        self.maxIter = maxIter
        self.obstacleList = obstacleList
        self.rrtTargets = rrtTargets

    def Planning(self, animation=False, interactive=False):
        self.nodeList = [self.start]
        self.leafNodes = []

        for i in range(self.maxIter):
            myRandomPoint = self.get_random_point_from_target_list()

            myNearestListIndex = self.GetNearestListIndex(self.nodeList, myRandomPoint)

            # Create a point
            newNode = self.steerConstrained(myRandomPoint, myNearestListIndex)

            # If newNode does not collide with any obstacles add it to the node list
            if self.__CollisionCheck(newNode, self.obstacleList):
                self.nodeList.append(newNode)

                if newNode.cost >= self.planDistance:
                    self.leafNodes.append(newNode)

            if animation:
                self.DrawSample(myRandomPoint)

            if interactive:
                key = self.getKey()

                if (key == '\x03'): #CTRL+C
                    break

        return self.nodeList, self.leafNodes


    '''
    Create a new node that is steering angle constrained
    '''
    def steerConstrained(self, aRandomPoint, aNearestListIndex):
        # expand tree
        nearestNode = self.nodeList[aNearestListIndex]
        theta = math.atan2(aRandomPoint[1] - nearestNode.y, aRandomPoint[0] - nearestNode.x)

        # dynamic constraints
        angleChange = self.pi_2_pi(theta - nearestNode.yaw)

        angle30degree = math.radians(30)

        if angleChange > angle30degree:
            angleChange = self.turnAngle
        elif angleChange == -angle30degree:
            angleChange = 0
        else:
            angleChange = -self.turnAngle

        newNode = copy.deepcopy(nearestNode)
        newNode.yaw += angleChange
        newNode.x += self.expandDis * math.cos(newNode.yaw)
        newNode.y += self.expandDis * math.sin(newNode.yaw)

        newNode.cost += self.expandDis
        newNode.parent = aNearestListIndex

        return newNode

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2*math.pi) - math.pi

    '''
    If self.rrtTargets is empty, this will generate a random point
    '''
    def get_random_point(self):

        randX = random.uniform(0, self.planDistance)
        randY = random.uniform(-self.planDistance, self.planDistance)
        rnd = [randX, randY]

        car_rot_mat = np.array([[math.cos(self.startYaw), -math.sin(self.startYaw)], [math.sin(self.startYaw), math.cos(self.startYaw)]])
        rotatedRnd = np.dot(car_rot_mat, rnd)

        rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]
        return rotatedRnd

    # Get a random point by producing random values and projecting it on the rrtTargets
    def get_random_point_from_target_list(self):

        maxTargetAroundDist = 3

        if not self.rrtTargets:
            return self.get_random_point()

        targetId = np.random.randint(len(self.rrtTargets))
        x, y, oSize = self.rrtTargets[targetId]

        # circle idea
        randAngle = random.uniform(0, 2 * math.pi)
        randDist = random.uniform(oSize, maxTargetAroundDist)
        finalRnd = [x + randDist * math.cos(randAngle), y + randDist * math.sin(randAngle)]

        return finalRnd


    # Find the distance of all the nodes in relations to the random point
    # return the nearest point to the random point
    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    '''
    Check if aNode collides with cones in aObstacleList
    '''
    def __CollisionCheck(self, aNode, aObstacleList):
        for (ox, oy, size) in aObstacleList:
            dx = ox - aNode.x
            dy = oy - aNode.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision
        return True  # safe

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def DrawSample(self, rnd=None):

        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        # draw obstacles
        axes = plt.gca()
        for (ox, oy, size) in self.obstacleList:
            circle = plt.Circle((ox,oy), radius=size)
            axes.add_patch(circle)

        plt.plot(self.start.x, self.start.y, "xr")

        axes = plt.gca()
        xmin, xmax, ymin, ymax = -5, 25, -20, 20
        axes.set_xlim([xmin,xmax])
        axes.set_ylim([ymin,ymax])

        plt.grid(True)
        plt.pause(0.001)

    def DrawGraph(self):

        # draw obstacles
        ax = plt.gca()
        for (ox, oy, size) in self.obstacleList:
            circle = plt.Circle((ox,oy), radius=size)
            ax.add_patch(circle)

        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                    node.y, self.nodeList[node.parent].y], "-g")

        plt.axis([-5, 45, -20, 20])
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.01)

class Node:
    """
    RRT Node
    """

    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = 0.0
        self.parent = None

    def __str__(self):
        return str(round(self.x, 2)) + "," + str(round(self.y,2)) + "," + str(math.degrees(self.yaw)) + "," + str(self.cost)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.yaw == other.yaw and self.cost == other.cost

    def __repr__(self):
        return str(self)


def main():
    print("Start rrt planning!")

    # ====Search Path with RRT====
    radius = 1
    obstacleList = [
        (1, -3, radius),
        (1, 3, radius),
        # (3, -3, radius),
        # (3, 3, radius),
        (6, -3, radius),
        (6, 3, radius),
        # (9, -2.8, radius),
        # (9, 3.2, radius),
        (12.5, -2.5, radius),
        (12, 4, radius),
        # (16, -2, radius),
        # (15, 5, radius),
        (20, -1, radius),
        (18.5, 6.5, radius),
        (24, 1, radius),
        (22, 8, radius)
    ]  # [x,y,size(radius)]

    start = [0.0, 0.0, math.radians(0.0)]
    planDistance = 10
    iterationNumber = 500
    rrtConeTargets = []

    for o in obstacleList:
        # Add cone coordinate into rttConeTargets if they are certain distance away
        coneDist = math.sqrt((start[0] - o[0]) ** 2 + (start[1] - o[1]) ** 2)

        if coneDist > 10 and coneDist < 15:
            rrtConeTargets.append((o[0], o[1], radius))

    startTime = time.time()

    # Set Initial parameters
    rrt = RRT(start, planDistance, obstacleList=obstacleList, expandDis=1, maxIter=iterationNumber, rrtTargets = rrtConeTargets)
    rrt.Planning(True, True)
    # rrt.Planning()

    print "rrt.Planning(): {0} ms".format((time.time() - startTime) * 1000);
    print "nodesNumber/iteration: {0}/{1}".format(len(rrt.nodeList), iterationNumber)

    show_animation = True

    if show_animation:
        rrt.DrawGraph()
        # plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    main()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
