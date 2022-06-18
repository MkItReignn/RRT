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

indexCheck = False

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
    def __init__(self, start, planDistance, obstacleList, expandDis=0.5, turnAngle=30,
                 maxIter=3100, rrtTargets = None, aUniformRandomGeneratorSeed = None, aRandomNumberGeneratorSeed = None):

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

        # Randomiser
        self.theRandomUniformNumberGenerator = np.random.RandomState(aUniformRandomGeneratorSeed)
        self.theRandomNumberGenerator = np.random.RandomState(aRandomNumberGeneratorSeed)

        self.i = 0

    def Planning(self, animation=False, interactive=False):
        self.nodeList = [self.start]
        self.leafNodes = []

        for i in range(self.maxIter):
            self.i = i
            myRandomPoint = self.get_random_point_from_target_list()
            # print(f"{myRandomPoint[0]:.16f} {myRandomPoint[1]:.16f}")

            myNearestListIndex = self.GetNearestListIndex(self.nodeList, myRandomPoint)
            # print(myNearestListIndex)

            if i == 310 and indexCheck:
                print(f"{myRandomPoint[0]:.16f} {myRandomPoint[1]:.16f}")
                print(f"{myNearestListIndex}")
                print()
                for node in self.nodeList:
                    print(f"{node.x:.16f} {node.y:.16f}")

            # Create a point
            newNode = self.steerConstrained(myRandomPoint, myNearestListIndex)
            # print(f"{newNode.x:.16f} {newNode.y:.16f}") # when i = 310
            if i == 310 and indexCheck:
                print()
                print(f"{newNode.x:.16f} {newNode.y:.16f}")
            # print(newNode)

            # If newNode does not collide with any obstacles add it to the node list
            # check node lives outside the obstacles
            if self.__CollisionCheck(newNode, self.obstacleList):
                self.nodeList.append(newNode)

                if newNode.cost >= self.planDistance:
                    self.leafNodes.append(newNode)

            if len(self.nodeList) == 276:
                # print(i)
                # print(f"{newNode.x:.16f} {newNode.y:.16f}")
                None

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
        # print("%.16f" % theta)

        # dynamic constraints
        # print("%.16f" % (theta - nearestNode.yaw))
        angleChange = self.pi_2_pi(theta - nearestNode.yaw)
        # print("%.16f" % angleChange)

        angle30degree = math.radians(30)

        if self.i == 310 and indexCheck:
            print()
            print(f"{theta:.16f} {angleChange:.16f}")

        if angleChange > angle30degree:
            angleChange = self.turnAngle
        elif angleChange >= -angle30degree:
            angleChange = 0
        else:
            angleChange = -self.turnAngle

        # print("%.16f" % angleChange)

        newNode = copy.deepcopy(nearestNode)
        newNode.yaw += angleChange
        newNode.x += self.expandDis * math.cos(newNode.yaw)
        newNode.y += self.expandDis * math.sin(newNode.yaw)
        if self.i == 310 and indexCheck:
            print(f"newNode.yaw = {newNode.yaw}")
            print(f"math.sin(newNode.yaw) = {math.sin(newNode.yaw):.16f}")
            None

        newNode.cost += self.expandDis
        newNode.parent = aNearestListIndex

        return newNode

    def pi_2_pi(self, angle):
        # return (angle + math.pi) % (2*math.pi) - math.pi
        return math.fmod((angle + math.pi), (2*math.pi)) - math.pi

    '''
    If self.rrtTargets is empty, this will generate a random point
    '''
    def get_random_point(self):

        # Replacing random.uniform with np.random.uniform()
        # randX = random.uniform(0, self.planDistance)
        # randY = random.uniform(-self.planDistance, self.planDistance)

        # randX = np.random.uniform(0, self.planDistance)
        # randY = np.random.uniform(-self.planDistance, self.planDistance)
        randX = self.theRandomUniformNumberGenerator.uniform(0, self.planDistance)
        randY = self.theRandomUniformNumberGenerator.uniform(-self.planDistance, self.planDistance)
        rnd = [randX, randY]

        car_rot_mat = np.array([
            [math.cos(self.startYaw), -math.sin(self.startYaw)],
            [math.sin(self.startYaw),  math.cos(self.startYaw)]
        ])
        rotatedRnd = np.dot(car_rot_mat, rnd)

        rotatedRnd = [rotatedRnd[0] + self.start.x, rotatedRnd[1] + self.start.y]
        return rotatedRnd

    # Get a random point by producing random values and projecting it on the rrtTargets
    def get_random_point_from_target_list(self):

        maxTargetAroundDist = 3

        if not self.rrtTargets:
            return self.get_random_point()

        # targetId = np.random.randint(len(self.rrtTargets))
        targetId = self.theRandomNumberGenerator.randint(len(self.rrtTargets)) # len == 2?
        # print(f"{targetId} {len(self.rrtTargets)}")
        # print(targetId)
        x, y, oSize = self.rrtTargets[targetId]

        # circle idea
        # Replacing random.uniform with np.random.uniform()
        # randAngle = random.uniform(0, 2 * math.pi)
        # randDist = random.uniform(oSize, maxTargetAroundDist)

        # randAngle = np.random.uniform(0, 2 * math.pi)
        # randDist = np.random.uniform(oSize, maxTargetAroundDist)
        myRandomAngle = self.theRandomUniformNumberGenerator.uniform(0, 2 * math.pi)
        myRandomDistance = self.theRandomUniformNumberGenerator.uniform(oSize, maxTargetAroundDist)
        finalRnd = [x + myRandomDistance * math.cos(myRandomAngle), y + myRandomDistance * math.sin(myRandomAngle)]
        # print("%.16f " % math.cos(myRandomAngle), end='')
        # print("%.16f" % math.sin(myRandomAngle))
        # print("%.16f " % finalRnd[0], end='')
        # print("%.16f" % finalRnd[1])
        return finalRnd


    # Find the distance of all the nodes in relations to the random point
    # return the nearest point to the random point
    def GetNearestListIndex(self, nodeList, rnd):
        dlist = []
        for node in nodeList:
            myDistance = (node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2
            dlist.append(myDistance)
            if self.i == 310 and indexCheck:
                print(f"{node.x:.16f} {rnd[0]:.16f}")
                print(f"{node.y:.16f} {rnd[1]:.16f}")
                print("%.16f" % myDistance)
                None
        minind = dlist.index(min(dlist))
        # self.i += 1
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN)
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
        # return f"{self.x:.16f} {self.y:.16f} {math.degrees(self.yaw):.16f} {self.cost:.16f}"
        return f"{self.x:.16f} {self.y:.16f} {self.yaw:.16f} {self.cost:.16f}"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.yaw == other.yaw and self.cost == other.cost

    def __repr__(self):
        return str(self)

def printNodes(aNodeList, printIndex = False):
    i = 0
    for myNode in aNodeList:
        if printIndex:
            print(f"{i}: {myNode}")
        else:
            print(f"{myNode}")

        i += 1

if __name__ == '__main__':
    # print("Start rrt planning!")

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
        (20, 310, radius),
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

        if 10 < coneDist < 15:
            rrtConeTargets.append((o[0], o[1], radius))
            # print(f"{o[0]} {o[1]}")

    startTime = time.time()

    # Set Initial parameters
    rrt = RRT(start, planDistance, obstacleList=obstacleList, expandDis=1, maxIter=iterationNumber,
              rrtTargets = rrtConeTargets, aUniformRandomGeneratorSeed=100, aRandomNumberGeneratorSeed=200)
    myReturnVal = rrt.Planning(False, False)
    # rrt.Planning()
    # print(myReturnVal)
    # print(f"Node List:")
    printNodes(myReturnVal[1])

    # print(f"Leaf Nodes:")
    # print(myReturnVal[1])
    # printNodes(myReturnVal[1])

    # print("rrt.Planning(): {0} ms".format((time.time() - startTime) * 3100))
    # print("nodesNumber/iteration: {0}/{1}".format(len(rrt.nodeList), iterationNumber))

    show_animation = False
    # show_animation = False

    if show_animation:
        rrt.DrawGraph()
        # plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)  # Need for Mac
        plt.show()


# if __name__ == '__main__':
    # settings = termios.tcgetattr(sys.stdin)

    # main()

    # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
