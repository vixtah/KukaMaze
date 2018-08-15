"""

A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import math
from collections import namedtuple


show_animation = True

Point = namedtuple('Point', 'x y')

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

    def contains(self, x, y):
        if x >= self.frame.x and y >= self.frame.y and x <= self.frame.x+self.frame.width and y <= self.frame.y+self.frame.height:
            return True
        return False


class Pose2D():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Node:
    def __init__(self, x, y, theta, cost, pind):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

def calc_final_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry



def collisionCheck(self, x, y):
    for obstacle in self.obstacleList:
        if obstacle.contains(x, y):
            return False

    return True  # safe


class AStar:

    def __init__(self, start, goal, obstacleList,
                 reso, xbound, ybound):
        self.start = Node(round(start[0] / reso), round(start[1] / reso), 0, 0, -1)
        self.end = Node(round(goal[0] / reso), round(goal[1] / reso), 0, 0, -1)
        self.reso = reso
        self.minx = xbound[0]
        self.maxx = xbound[1]
        self.miny = ybound[0]
        self.maxy = ybound[1]
        self.obstacleList = obstacleList

        self.motion = [[1, 0, 0, 1],#add theta
                       [0, 1, 0, 1],
                       [-1, 0, 0, 1],
                       [0, -1, 0, 1]]

    def calc_index(self, node):
        thetaVal = 0 if node.theta == 0 else 1
        thetaVal = 0
        xwidth = self.maxx - self.minx
        ywidth = self.maxy - self.miny
        print(node.x, self.minx, node.x - self.minx)
        return thetaVal * xwidth * ywidth + (node.y - self.miny) * xwidth + (node.x - self.minx)

    def calc_heuristic(n1, n2):
        w = 1  # weight of heuristic
        d = w * math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
        return d

    def collisionCheck(self, x, y):
        for obstacle in self.obstacleList:
            if obstacle.contains(x, y):
                return False
        return True  # safe

    def draw(self, node):
        plt.plot(node.x * self.reso, node.y * self.reso, "xc")
        ax = plt.gca()
        # currentAxis.add_patch(Rectangle((someX - .1, someY - .1), 0.2, 0.2,alpha=1))

        for obstacle in self.obstacleList:
            # Create a Rectangle patch
            frame = obstacle.frame
            rect = patches.Rectangle((frame.x,frame.y),frame.width,frame.height,linewidth=1,edgecolor='r',facecolor='r')

            # Add the patch to the Axes
            ax.add_patch(rect)
        if len(self.closedset.keys()) % 10 == 0:
            plt.pause(0.001)


    def a_star_planning(self):
        openset, closedset = dict(), dict()
        self.closedset = closedset
        startIndex = self.calc_index(self.start)
        openset[startIndex] = self.start

        while 1:
            c_id = min(
                openset, key=lambda o: openset[o].cost + AStar.calc_heuristic(self.end, openset[o]))
            current = openset[c_id]

            # show graph
            if show_animation:
                self.draw(current)

            if current.x == self.end.x and current.y == self.end.y:
                print("Find goal")
                self.end.pind = current.pind
                self.end.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = current

            # expand search grid based on motion model
            for i in range(len(self.motion)):
                newx = round(current.x + (self.motion[i][0]))
                newy = round(current.y + (self.motion[i][1]))
                if not (self.minx <= newx*self.reso and newx*self.reso <= self.maxx and self.miny <= newy*self.reso and newy*self.reso <= self.maxy):
                    continue

                if show_animation:
                    plt.plot(newx * self.reso, newy * self.reso, "xc")

                node = Node(newx,
                            newy,
                            current.theta + self.motion[i][2],
                            current.cost + self.motion[i][3],
                            c_id)
                n_id = self.calc_index(node)

                if n_id in closedset:
                    continue

                if not self.collisionCheck(node.x*self.reso, node.y*self.reso):
                    continue

                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node

                tcost = current.cost + AStar.calc_heuristic(current, node)

                if tcost >= node.cost:
                    continue  # this is not a better path

                node.cost = tcost
                openset[n_id] = node  # This path is the best unitl now. record it!

        rx, ry = calc_final_path(self.end, closedset, self.reso)

        return rx, ry


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = -0.2  # [m]
    sy = 0  # [m]
    gx = 3.6  # [m]
    gy = 1.2  # [m]

    if show_animation:
        # plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "xr")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    obstacleList = [
        Obstacle(Frame(0,.2,.4,.05)),
        Obstacle(Frame(.6,.2,.7,.05)),
        Obstacle(Frame(.8,.2,.05,1.2)),
        Obstacle(Frame(.2,1.4,.6,.05))
    ]

    aStar = AStar([sx, sy], [gx, gy], obstacleList,
        .2, [-5, 5], [-5, 5])

    rx, ry = aStar.a_star_planning()

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()