#! /usr/bin/env python

import rospy
import actionlib
from rll_planning_project.srv import *
from rll_planning_project.msg import *
from geometry_msgs.msg import Pose2D
from heapq import heappush, heappop # for priority queue
import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math
import copy
from collections import namedtuple
Point = namedtuple('Point', 'x y')

show_animation = True

# class Pose2D():
#     def __init__(self, x, y, theta):
#         self.x = x
#         self.y = y
#         self.theta = theta

class Node():
    def __init__(self, pose):
        self.pose = pose
        self.parent = None

# class Frame():
#     def __init__(self, x, y, width, height):
#         self.x = x
#         self.y = y
#         self.width = width
#         self.height = height

# class Obstacle():

#     def __init__(self, frame):
#         self.frame = frame
#         self.points = [
#             Point(frame.x, frame.y),
#             Point(frame.x+frame.width, frame.y),
#             Point(frame.x+frame.width, frame.y+frame.height),
#             Point(frame.x, frame.y+frame.height)
#         ]

#     @staticmethod
#     def ccw(A,B,C):
#         return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

#     # Return true if line segments AB and CD intersect
#     def intersects(self, pose1, pose2):
#         C = pose1
#         D = pose2
#         for i, point in enumerate(self.points):
#             A = point
#             B = self.points[(i+1)%len(self.points)]
#             if Obstacle.ccw(A,C,D) != Obstacle.ccw(B,C,D) and Obstacle.ccw(A,B,C) != Obstacle.ccw(A,B,D):
#                 return True
#         return False



class RRT():
    """
    Class for RRT Planning
    """

    def __init__(self, start, goal, checkCollision,
                 xBounds, yBounds, expandDis=.03, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(Pose2D(start.x, start.y, 0))
        self.end = Node(Pose2D(goal.x, goal.y, 0))
        self.minrandx = xBounds[0]
        self.maxrandx = xBounds[1]
        self.minrandy = yBounds[0]
        self.maxrandy = yBounds[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.checkCollision = checkCollision



    def Planning(self, animation=True):
        """
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrandx, self.maxrandx), random.uniform(
                    self.minrandy, self.maxrandy), random.randint(0,1)*math.pi/2] #add random pose
            else:
                rnd = [self.end.pose.x, self.end.pose.y, self.end.pose.theta]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.pose.y, rnd[0] - nearestNode.pose.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.pose.x += self.expandDis * math.cos(theta)
            newNode.pose.y += self.expandDis * math.sin(theta)
            newNode.pose.theta = rnd[2] #set theta 
            newNode.parent = nind

#             if not self.__CollisionCheck(newNode.pose, nearestNode.pose, self.obstacleList):
#                 continue
            if not self.checkCollision(newNode.pose, nearestNode.pose).valid:
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.pose.x - self.end.pose.x
            dy = newNode.pose.y - self.end.pose.y
            
            d = math.sqrt(dx * dx + dy * dy)
            
            if d <= self.expandDis and (self.end.theta - newNode.theta) < .01:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

        path = [[self.end.pose.x, self.end.pose.y, self.end.pose.theta]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
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
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.pose.x, self.nodeList[node.parent].pose.x], [
                         node.pose.y, self.nodeList[node.parent].pose.y], "-g")

        # # Create figure and axes
        # ax = plt.gca()

        # for obstacle in self.obstacleList:
        #     frame = obstacle.frame
        #     rect = patches.Rectangle((frame.x,frame.y),frame.width,frame.height,linewidth=1,edgecolor='r',facecolor='r')
        #     ax.add_patch(rect)

        plt.plot(self.start.pose.x, self.start.pose.y, "xr")
        plt.plot(self.end.pose.x, self.end.pose.y, "xr")
        plt.axis([-.6, .6, -.8, .8])
        plt.grid(True)
        plt.pause(0.01)

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.pose.x - rnd[0]) ** 2 + (node.pose.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    # def __CollisionCheck(self, node1, node2, obstacleList):

    #     for obstacle in obstacleList:
    #         if obstacle.intersects(node1, node2):
    #             return False

    #     return True  # safe

def plan_to_goal(req):
    """ Plan a path from Start to Goal """
    pose_start = Pose2D()
    pose_goal = Pose2D()
    pose_check_start = Pose2D()
    pose_check_goal = Pose2D()
    pose_move = Pose2D()

    rospy.loginfo("Got a planning request")

    pose_start = req.start
    pose_goal = req.goal

    move_srv = rospy.ServiceProxy('move', Move)
    check_srv = rospy.ServiceProxy('check_path', CheckPath, persistent=True)

    ###############################################
    # Implement your path planning algorithm here #
    ###############################################

    # Input: map dimensions, start pose, and goal pose
    # retrieving input values  
    map_width = rospy.get_param('~map_width')
    map_length = rospy.get_param('~map_length')
    xStart, yStart, tStart = pose_start.x, pose_start.y, pose_start.theta
    xGoal, yGoal, tGoal = pose_goal.x, pose_goal.y, pose_goal.theta
    # printing input values
    rospy.loginfo("map dimensions: width=%1.2fm, length=%1.2fm", map_width, map_length)
    rospy.loginfo("start pose: x %f, y %f, theta %f", xStart, yStart, tStart)
    rospy.loginfo("goal pose: x %f, y %f, theta %f", xGoal, yGoal, tGoal)

    # Output: movement commands

    rrt = RRT(start=pose_start, goal=pose_goal, checkCollision=check_srv,
              xBounds=[-.6, .6], yBounds=[-.8,.8])
    path = rrt.Planning(animation=show_animation)
    path.reverse()
    print(path)
    
#     pose_check_start.x, pose_check_start.y, pose_check_start.theta= xStart, yStart, tStart
#     pose_check_goal.x, pose_check_goal.y, pose_check_goal.theta= xStart, yStart, tStart
    for i, pose in enumerate(path):
        if i == 0:
            continue
        pose_move.x, pose_move.y, pose_move.theta = pose[0], pose[1], pose[2]

        resp = move_srv(pose_move)

#     pose_check_start.x, pose_check_start.y, pose_check_start.theta= xStart, yStart, tStart
#     pose_check_goal.x, pose_check_goal.y, pose_check_goal.theta= xGoal, yGoal, tGoal
#     resp = check_srv(pose_check_start, pose_check_goal) # checking if the arm can move to the goal pose
#     if resp.valid:
#         rospy.loginfo("Valid pose")
#         pose_move.x, pose_move.y, pose_move.theta = xGoal, yGoal, tGoal 
#         # executing a move command towards the goal pose
#         resp = move_srv(pose_move)
#     else:
#         rospy.loginfo("Invalid pose")
        
    ###############################################
    # End of Algorithm #
    ###############################################


class PathPlanner:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("plan_to_goal", PlanToGoalAction, self.execute, False)
        self.server.start()

    def execute(self, req):
        plan_to_goal(req)
        self.server.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('path_planner')

    server = PathPlanner()

    rospy.spin()
