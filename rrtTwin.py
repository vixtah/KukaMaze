

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math
import copy
from collections import namedtuple
Point = namedtuple('Point', 'x y')

show_animation = True

class Pose2D():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Node():
    def __init__(self, pose):
        self.pose = pose
        self.parent = None

class Frame():
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

class Obstacle():

    def __init__(self, frame):
        self.frame = frame
        self.points = [
            Point(frame.x, frame.y),
            Point(frame.x+frame.width, frame.y),
            Point(frame.x+frame.width, frame.y+frame.height),
            Point(frame.x, frame.y+frame.height)
        ]

    @staticmethod
    def ccw(A,B,C):
        return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

    # Return true if line segments AB and CD intersect
    def intersects(self, pose1, pose2):
        C = pose1
        D = pose2
        for i, point in enumerate(self.points):
            A = point
            B = self.points[(i+1)%len(self.points)]
            if Obstacle.ccw(A,C,D) != Obstacle.ccw(B,C,D) and Obstacle.ccw(A,B,C) != Obstacle.ccw(A,B,D):
                return True
        return False



class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, expandDis=.1, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(Pose2D(start[0], start[1], 0))
        self.end = Node(Pose2D(goal[0], goal[1], 0))
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.obstacleList = obstacleList

    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.startNodes = [self.start]
        self.endNodes = [self.end]


        selectSwitch = True
        while True:
            nodeList = self.startNodes if selectSwitch else self.endNodes
            otherList = self.startNodes if not selectSwitch else self.endNodes
            selectSwitch = not selectSwitch

            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand), 0] #add random pose
            else:
                rnd = [self.end.pose.x, self.end.pose.y, self.end.pose.theta]

            # Find nearest node
            nind = self.GetNearestListIndex(nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.pose.y, rnd[0] - nearestNode.pose.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.pose.x += self.expandDis * math.cos(theta)
            newNode.pose.y += self.expandDis * math.sin(theta)
            newNode.pose.theta = 0 #set theta
            newNode.parent = nind

            if not self.__CollisionCheck(newNode.pose, nearestNode.pose, self.obstacleList):
                continue

            nodeList.append(newNode)
            print("nNodelist:", len(nodeList))

            # check goal
            nearestOtherNodeInd = self.GetNearestNode(otherList, newNode)
            nearestOtherNode = otherList[nearestOtherNodeInd]
            if self.__CollisionCheck(newNode.pose, nearestOtherNode.pose, self.obstacleList):
                print("Goal!!")
                break

            # dx = newNode.pose.x - self.end.pose.x
            # dy = newNode.pose.y - self.end.pose.y
            # d = math.sqrt(dx * dx + dy * dy)
            # if d <= self.expandDis:
            #     print("Goal!!")
            #     break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.pose.x, self.end.pose.y, self.end.pose.theta]]
        lastIndex = len(nodeList) - 1
        while nodeList[lastIndex].parent is not None:
            node = nodeList[lastIndex]
            path.append([node.pose.x, node.pose.y, node.pose.theta])
            lastIndex = node.parent
        path.append([self.start.pose.x, self.start.pose.y, self.start.pose.theta])

        return path

    def DrawGraph(self, rnd=None):
        """
        Draw Graph
        """
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.startNodes:
            if node.parent is not None:
                plt.plot([node.pose.x, self.startNodes[node.parent].pose.x], [
                         node.pose.y, self.startNodes[node.parent].pose.y], "-g")

        for node in self.endNodes:
            if node.parent is not None:
                plt.plot([node.pose.x, self.endNodes[node.parent].pose.x], [
                         node.pose.y, self.endNodes[node.parent].pose.y], "-r")

        # Create figure and axes

        # fig,ax = plt.subplots(1)
        ax = plt.gca()
        # currentAxis.add_patch(Rectangle((someX - .1, someY - .1), 0.2, 0.2,alpha=1))

        for obstacle in self.obstacleList:
            # Create a Rectangle patch
            frame = obstacle.frame
            rect = patches.Rectangle((frame.x,frame.y),frame.width,frame.height,linewidth=1,edgecolor='r',facecolor='r')

            # Add the patch to the Axes
            ax.add_patch(rect)
            # plt.plot(ox, oy, "ok", ms=30 * size)

        plt.plot(self.start.pose.x, self.start.pose.y, "xr")
        plt.plot(self.end.pose.x, self.end.pose.y, "xr")
        plt.axis([0, 1.2, 0, 1.6])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.pose.x - rnd[0]) ** 2 + (node.pose.y - rnd[1])
                 ** 2 for node in nodeList]

        count = 0
        minind = dlist.index(min(dlist))
        return minind

    def GetNearestNode(self, nodeList, newNode):
        coords = [newNode.pose.x, newNode.pose.y]
        return self.GetNearestListIndex(nodeList, coords)


    def __CollisionCheck(self, node1, node2, obstacleList):

        for obstacle in obstacleList:
            if obstacle.intersects(node1, node2):
                return False

        return True  # safe

def main():
    print("start simple RRT path planning")

    # ====Search Path with RRT====
    obstacleList = [
        Obstacle(Frame(0,.2,.4,.05)),
        Obstacle(Frame(.6,.2,.7,.05)),
        Obstacle(Frame(.8,.2,.05,1.2)),
        Obstacle(Frame(.2,1.4,.6,.05))

    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[0, 0], goal=[1.0, 1.2], obstacleList=obstacleList,
              randArea=[0, 1.6])
    path = rrt.Planning(animation=show_animation)

    # Draw final path
    if show_animation:
        rrt.DrawGraph()
        plt.plot([x for (x, y, theta) in path], [y for (x, y, theta) in path], '-r')
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()