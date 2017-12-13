import sys
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import numpy as np
from math import *

'''
Set up matplotlib to create a plot with an empty square
'''
def setupPlot():
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    plt.autoscale(False)
    plt.axis('off')
    ax = fig.add_subplot(1,1,1)
    ax.set_axis_off()
    ax.add_patch(patches.Rectangle(
        (0,0),   # (x,y)
        1,          # width
        1,          # height
        fill=False
        ))
    return fig, ax

'''
Make a patch for a single pology
'''
def createPolygonPatch(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0]/10., xy[1]/10.))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch

def createPolygonPatchFull(polygon, color):
    verts = []
    codes= []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch

def createPathPatch(path, points, color):
    verts = [points[path[0]]]
    codes= [Path.MOVETO]

    for i in range(len(path)):
        key = path[i] 
        point = points[key]
        verts.append(point)
        codes.append(Path.LINETO)
        # verts.append(point)
        # codes.append(Path.MOVETO)

    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='None', edgecolor=color, lw=2)

    return patch

def createRRTPatch(branch, points, startPoint, color):
    verts = [startPoint]
    codes = [Path.MOVETO]

    for i in range(len(branch)):
        key = branch[i]
        point = points[key]
        verts.append(point)
        codes.append(Path.LINETO)
        verts.append(startPoint)
        codes.append(Path.MOVETO)

    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor='None', edgecolor=color, lw=1)

    return patch


'''
Render the problem
'''
def drawProblem(robotStart, robotGoal, polygons):
    fig, ax = setupPlot()
    patch = createPolygonPatch(robotStart, 'green')
    ax.add_patch(patch)
    patch = createPolygonPatch(robotGoal, 'red')
    ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()


'''
Builds an adjacencyList based on points
'''
# is given the nearest point found, and the point to connect
def updateALM(point1, point2, newPoints, adjListMap):
    #finding the existing adjacent vertices
    for i in range(len(newPoints)):
        if point1 == newPoints[i+1]:
            p1 = i+1
        if point2 == newPoints[i+1]:
            p2 = i+1

    temp1 = adjListMap.get(p1, None)
    temp2 = adjListMap.get(p2, None)

    if temp1 == None:
        temp1 = []
    if temp2 == None:
        temp2 = []

    temp1.append(p2)
    temp2.append(p1)

    # temp1.sort()
    # temp2.sort()

    adjListMap[p1] = temp1
    adjListMap[p2] = temp2

    # print "AdjListMap " + str(point1) + " " + str(point2)

    return adjListMap

def updateALMcut(point1, point2, point3, newPoints, adjListMap):
    #finding the existing adjacent vertices
    for i in range(len(newPoints)):
        if point1 == newPoints[i+1]:
            p1 = i+1
        if point2 == newPoints[i+1]:
            p2 = i+1
        if point3 == newPoints[i+1]:
            p3 = i+1

    temp1 = adjListMap.get(p1, None)
    temp2 = adjListMap.get(p2, None)
    temp3 = adjListMap.get(p3, None)

    if temp1 == None:
        temp1 = []
    if temp2 == None:
        temp2 = []
    if temp3 == None:
        temp3 = []

    #linking p1 to p2
    temp1.append(p2)
    temp2.append(p1)
    #linking p2 to p3
    temp2.append(p3)
    temp3.append(p2)

    #removing old ink
    temp1.remove(p3)
    temp3.remove(p1)

    # temp1.sort()
    # temp2.sort()
    # temp3.sort()

    adjListMap[p1] = temp1
    adjListMap[p2] = temp2
    adjListMap[p3] = temp3

    # print "AdjListMap " + str(point1) + " " + str(point2)

    return adjListMap

'''Used for RRT - finds distance between two points'''
def distance(point1, point2):
        dist = sqrt(abs((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2))
        return round(dist,8)

'''Used for finding perp distance to line'''
#returns distance and point perpendicular
#p1 and p2 form line, p3 is point we are tring to find perpendicular to line
def distanceToLine(p1, p2, p3):
    #perpendicular code here
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = p3[0]
    y3 = p3[1]

    x1 = float(x1)
    y1 = float(y1)
    x2 = float(x2)
    y2 = float(y2)
    x3 = float(x3)
    y3 = float(y3)
    k = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / ((y2-y1)**2 + (x2-x1)**2)
    x4 = x3 - k * (y2-y1)
    y4 = y3 + k * (x2-x1)

    #calculating line distance
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    d = np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1)

    return d, (x4,y4)

'''
Grow a simple RRT
'''
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()

    testLinePoints = []
    
    #putting in the start node
    newPoints[1] = points[1]

    for i in range(2,len(points)+1):
        minDist = 10*sqrt(2)
        minIndex = -1
        minIndexLine = -1
        withLine = False

        #checking for closest point in the Tree newPoints
        for j in range(1, len(newPoints)+1):
            pointDist = distance(points[i], newPoints[j])
            #if the distance is now closer update to new nearest point
            if pointDist < minDist:
                minDist = pointDist
                minIndex = j
        #checking against lines formed by the adjacency list, with the random Point
        # #won't enter if there's only one other point in the Tree
        '''
        This is line intersecting checking - need to get this working
        '''
        for j in range(1, len(adjListMap)+1):
            #the adjacent vertecies
            branch = adjListMap[j]
            # #checking the lines formed by a branch
            for k in range(len(branch)):
                start = newPoints[j]
                key = branch[k]
                if start != newPoints[key]:
                    lineDist, point = distanceToLine(start, newPoints[key], points[i])
                    buff = .01

                    #if the lineDist is smaller than the buffer
                    if minDist - lineDist > buff and lineDist != 0:
                        #checking to make sure its on the relevant parts of the line
                        left = min(start[0], newPoints[key][0])
                        right = max(start[0], newPoints[key][0])

                        if left < point[0] and point[0] < right:
                            withLine = True
                            minDist = lineDist

                            linePoint = point
                            minIndex = j
                            minIndexLine = key


        #closest point found is on a line formed by the tree
        if withLine:
            #adding in the point on the line
            newPoints[len(newPoints)+1] = linePoint
            #updateAlM special for removing those two from the adjListMap??
            #connecting it to both ends of the line
            adjListMap = updateALMcut(newPoints[minIndex], linePoint, newPoints[minIndexLine], newPoints, adjListMap)

            pointToAdd = points[i]
            newPoints[len(newPoints)+1] = pointToAdd
            adjListMap = updateALM(linePoint, pointToAdd, newPoints, adjListMap)

            #for visualization
            testLinePoints.append(linePoint)

        else:
            pointToAdd = points[i]
            newPoints[len(newPoints)+1] = pointToAdd
            adjListMap = updateALM(newPoints[minIndex], pointToAdd, newPoints, adjListMap)

        #reaching end here will have closest possible point

    # print newPoints
    # print "adjListmap is: "
    # print adjListMap

    #testLinePoints is not one of the given ones
    return newPoints, adjListMap

'''
Perform basic search
'''
def basicSearch(tree, start, goal):
    path = []
    queue = [[start]]
    visited = dict()

    while len(queue)>0:
        path = queue.pop(0)
        # print path

        vertex = path[len(path)-1]

        if vertex == goal:
            # print "\nthis is path from " + str(start) + " to " + str(goal) + ": " + str(path) + "\n"
            return path

        elif visited.get(vertex, None) == None:
            # print vertex
            # print tree[vertex][0]
            for i in range(len(tree[vertex])):
                new_path = list(path)
                new_path.append(tree[vertex][i])
                # print new_path
                queue.append(new_path)
                # print queue

            visited[vertex] = True


    return path

'''
Display the RRT and Path
'''
def displayRRTandPath(points, tree, path, robotStart = None, robotGoal = None, polygons = None):

    # Your code goes here
    # You could start by copying code from the function
    # drawProblem and modify it to do what you need.
    # You should draw the problem when applicable.

    fig = plt.figure()
    ax = fig.add_subplot(111)

    if robotStart != None and robotGoal != None and obstacles != None:
        patch = createPolygonPatchFull(robotStart, 'green')
        ax.add_patch(patch)
        patch = createPolygonPatchFull(robotGoal, 'red')
        ax.add_patch(patch)
        for p in range(0, len(polygons)):
            patch = createPolygonPatchFull(polygons[p], 'gray')
            ax.add_patch(patch)

        #finding the new values
        points, tree, path = RRT(robot, obstacles, robotStart[0], robotGoal[0])
    # else:
        # ''' used for seeing original poitns, remove later'''
        # x1 = []
        # y1 = []
        # for i in range(1,len(og_points)):
        #     x1.append(og_points[i+1][0])
        #     y1.append(og_points[i+1][1])

        # plt.scatter(x1,y1, edgecolors='yellow', facecolor='green', s=20)
        # # count = 2
        # # for xy in zip(x1, y1):                                       
        # #     ax.annotate(count, xy=xy, xytext=(-2,-2), textcoords='offset points')
        # #     count += 1

        # x2 = []
        # y2 = []
        # # print testLinePoints
        # for i in range(len(testLinePoints)):
        #     x2.append(testLinePoints[i][0])
        #     y2.append(testLinePoints[i][1])

        # plt.scatter(x2,y2, edgecolors='red', s=25)
        # '''remove til here'''

    for i in range(1, len(tree)+1):
        branch = tree[i]
        startPoint = points[i]
        patch = createRRTPatch(branch, points, startPoint, 'black')
        ax.add_patch(patch)

    ax.set_xlim(0,10)
    ax.set_ylim(0,10)
    x0,x1 = ax.get_xlim()
    y0,y1 = ax.get_ylim()
    ax.set_aspect(abs(x1-x0)/abs(y1-y0))


    #shows the points in newPoints
    x = []
    y = []
    for i in range(len(points)):
        x.append(points[i+1][0])
        y.append(points[i+1][1])

    plt.scatter(x,y, edgecolors='black', facecolor='black', s=10)
    # count = 1
    # for xy in zip(x, y):                                       
    #     ax.annotate(count, xy=xy, xytext=(3,3), textcoords='offset points')
    #     count += 1


    #shows the path
    patch = createPathPatch(path, points, 'orange')
    ax.add_patch(patch)


    
    plt.show()
    return

#line intersection used for collision detection
# def intersecting(point1, point2, point3, point4):
#     xdiff = (point1[0] - point2[0], point3[0] - point4[0])
#     ydiff = (point1[1] - point2[1], point3[1] - point4[1])

#     def det(a, b):
#         return a[0] * b[1] - a[1] * b[0]

#     div = det(xdiff, ydiff)
#     if div == 0:
#         return False

#     line1 = [point1, point2]
#     line2 = [point3, point4]        
#     d = (det(*line1), det(*line2))
#     x = det(d, xdiff) / div
#     y = det(d, ydiff) / div
    
#     # left1 = min(point1[0],point2[0])
#     # right1 = max(point1[0],point2[0])
#     left = min(point3[0],point4[0])
#     right = max(point3[0],point4[0])
#     top = max(point3[1],point4[1])
#     bottom = min(point3[1],point4[1])

#     if left<x and x<right and bottom<y and y<top:
#         return True
#     else:
#         return False


# def line_intersection(p1, p2, p3, p4):
#     line1 = [p1,p2]
#     line2 = [p3,p4]
#     xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
#     ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

#     def det(a, b):
#         return a[0] * b[1] - a[1] * b[0]

#     div = det(xdiff, ydiff)
#     if div == 0:
#        raise Exception('lines do not intersect')

#     d = (det(*line1), det(*line2))
#     x = det(d, xdiff) / div
#     y = det(d, ydiff) / div

#     left = min(p3[0],p4[0])
#     right = max(p3[0],p4[0])
#     bottom = min(p3[1],p4[1])
#     top = max(p3[1],p4[1])

#     if left<x and x<right:
#         if bottm<y and y<top:
#             return True

#     return False

# from numpy import *
def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def line_intersection(a1,a2, b1,b2) :
    a1 = np.array(a1)
    a2 = np.array(a2)
    b1 = np.array(b1)
    b2 = np.array(b2)
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    point = (num / denom.astype(float))*db + b1

    p1 = a1
    p2 = a2
    p3 = b1
    p4 = b2
    x = point[0]
    y = point[1]
    left = min(p3[0],p4[0])
    right = max(p3[0],p4[0])
    bottom = min(p3[1],p4[1])
    top = max(p3[1],p4[1])

    if left<x and x<right:
        if bottom<y and y<top:
            print "intersection found", p1, p2, p3, p4, (x,y)
            return True

    return False

#     return False
'''
Collision checking
'''
# def isCollisionFree(robot, poitn, obstacles):
    # return True

def isCollisionFree(robot, point, obstacles):

    # Your code goes here

    #pseudo code
    #1 - transalte robot to this new point
    #2 - for each polygon
    #3 -    each line in polygon
    #4 -    for each side of the robot:
    #5          check if it collides 
    #               specified by intersection and left bound right bound of robot, and of point found
    #6 -        if collision in bounds - return false

    # movedRobot= []
    # for i in range(len(robot)):
    #     x = robot[i][0] + point[0]
    #     y = robot[i][1] + point[1]
    #     movedRobot.append((round(x,4),round(y,4)))

    # #navigating through the points of the robot
    # for i in range(len(movedRobot)):
    #     if i == len(movedRobot)-1:
    #         # print "testing last full loop" + str(i)
    #         p1 = movedRobot[i]
    #         p2 = movedRobot[0]
    #     else:
    #         p1 = movedRobot[i]
    #         p2 = movedRobot[i+1]

    #     for j in range(len(obstacles)):
    #         polygon = obstacles[j]

    #         for k in range(len(polygon)):
    #             if k == len(polygon)-1:
    #                 p3 = polygon[k]
    #                 p4 = polygon[0]
    #             else:
    #                 p3 = polygon[k]
    #                 p4 = polygon[k+1]

    #             x,y = intersecting(p1, p2, p3, p4)

    #             left = min(p1[0],p2[0])
    #             right = max(p1[0],p2[0])
    #             bottom = min(p1[0],p2[0])
    #             top = max(p1[0],p2[0])

    #             if left < x and x < right:
    #                 if bottom < y and y < top:
    #                     #collision detected?""
    #                     print "collision"
    #                     return False

    #translating the robot to the point given
    # print robot[0][0], point[0]
    robotPoints = []
    for i in range(len(robot)):
        x = robot[i][0] + point[0]
        y = robot[i][1] + point[1]
        robotPoints.append((round(x,4),round(y,4)))
        #robot is out of bounds
        if robotPoints[i][0] < 0 or robotPoints[i][0] > 10:
            # print "robot out of bounds"
            return False
        if robotPoints[i][1] < 0 or robotPoints[i][1] > 10:
            return False

    # # # print robotPoints

    # # #running for each of the obstacles
    # for i in range(len(obstacles)):
    #     polygon = obstacles[i]

    #     for j in range(len(polygon)-1):
    #         p1 = polygon[j]
    #         p2 = polygon[j+1]

    #         for k in range(len(robotPoints)-1):
    #             p3 = robotPoints[k]
    #             p4 = robotPoints[k+1]
    #             # intersect = intersecting(p1,p2,p3,p4)
    #             intersect = line_intersection(p1,p2,p3,p4)
    #             if intersect == True:
    #                 return False
    #         p3 = robotPoints[len(robotPoints)-1]
    #         p4 = robotPoints[0]
    #         # intersect = intersecting(p1,p2,p3,p4)
    #         intersect = line_intersection(p1,p2,p3,p4)
    #         if intersect == True:
    #             return False
    #     p1 = polygon[len(polygon)-1]
    #     p2 = polygon[0]
    #     # intersect = intersecting(p1,p2,p3,p4)
    #     intersect =line_intersection(p1,p2,p3,p4)
    #     if intersect == True:
    #         return False

    # for i in range(len(obstacles)):
    #     polygon = obstacles[i]
    #     for j in range(len(polygon)):
    #         if j == len(polygon)-1:
    #             p1 = polygon[j]
    #             p2 = polygon[0]
    #         else: 
    #             p1 = polygon[j]
    #             p2 = polygon[j+1]

    #         for k in range(len(robotPoints)):
    #             if k == len(robotPoints)-1:
    #                 p3 = robotPoints[k]
    #                 p4 = robotPoints[0]
    #             else:
    #                 p3 = robotPoints[k]
    #                 p4 = robotPoints[k+1]
    #             intersect = line_intersection(p1,p2,p3,p4)
    #             if intersect == True:
    #                 # print "intersection found" + str(p1) 
    #                 return False


    # for i in range(0,len(robot)):
    #     robot[i][0] = robot[i][0] + point[0]
    #     robot[i][1] = robot[i][1] + point[1]
    #     if(robot[i][0] >= 10 or robot[i][1] p310):
    #         return False

    # for i in range(0,len(robotPoints)):
    #     if(i != len(robotPoints)-1):
    #         if(robotPoints[i+1][1] == robotPoints[i][1]):
    #             xDist = robotPoints[i+1][0]- robotPoints[i][0]
    #             for j in range(0,100):
    #                 increment = xDist/100
    #                 xPoint = robotPoints[i][0] + (j*increment)
    #                 yPoint = robotPoints[i][1]
    #                 for k in range(0,len(obstacles)):
    #                     for l in range(0,len(obstacles[k])):
    #                         if(abs(xPoint-obstacles[k][l][0]) < 0.2):
    #                             if(abs(yPoint-obstacles[k][l][1]) < 0.2):
    #                                 return False
    #         elif(robotPoints[i+1][0] == robotPoints[i][0]):
    #             yDist = robotPoints[i+1][1]- robotPoints[i][1]
    #             for j in range(0,100):
    #                 increment = yDist/100
    #                 yPoint = robotPoints[i][1] + (j*increment)
    #                 xPoint = robotPoints[i][0]
    #                 for k in range(0,len(obstacles)):
    #                     for l in range(0,len(obstacles[k])):
    #                         if(abs(xPoint-obstacles[k][l][0]) < 0.2):
    #                             if(abs(yPoint-obstacles[k][l][1]) < 0.2):
    #                                 return False
    #         else:
    #             slope =(robotPoints[i+1][1]-robotPoints[i][1])/(robotPoints[i+1][0]-robotPoints[i][0])
    #             b = robotPoints[i][1] - (robotPoints[i][0]*slope)
    #             xDist = robotPoints[i+1][0] - robotPoints[i][0]
    #             for j in range(0,100):
    #                 increment = xDist/100
    #                 xPoint = robotPoints[i][0] + (j*increment)
    #                 yPoint = (xPoint*slope) + b
    #                 for k in range(0,len(obstacles)):
    #                     for l in range(0,len(obstacles[k])):
    #                         if(abs(xPoint-obstacles[k][l][0]) < 0.2):
    #                             if(abs(yPoint-obstacles[k][l][1]) < 0.2):
    #                                 return False
    #     if(robotPoints[i][1] == robotPoints[0][1]):
    #         xDist = robotPoints[i][0]- robotPoints[0][0]
    #         for j in range(0,100):
    #             increment = xDist/100
    #             xPoint = robotPoints[0][0] + (j*increment)
    #             yPoint = robotPoints[0][1]
    #             for k in range(0,len(obstacles)):
    #                 for l in range(0,len(obstacles[k])):
    #                     if(abs(xPoint-obstacles[k][l][0]) < 0.2):
    #                         if(abs(yPoint-obstacles[k][l][1]) < 0.2):
    #                             return False
    #     elif(robotPoints[i][0] == robotPoints[0][0]):
    #         yDist = robotPoints[i][1]- robotPoints[0][1]
    #         for j in range(0,100):
    #             increment = yDist/100
    #             yPoint = robotPoints[0][1] + (j*increment)
    #             xPoint = robotPoints[0][0]
    #             for k in range(0,len(obstacles)):
    #                 for l in range(0,len(obstacles[k])):
    #                     if(abs(xPoint-obstacles[k][l][0]) < 0.2):
    #                         if(abs(yPoint-obstacles[k][l][1]) < 0.2):
    #                             return False
    #     else:
    #         slope = (robotPoints[0][1] - robotPoints[i][1])/(robotPoints[0][0]-robotPoints[i][0])
    #         b = robotPoints[i][1] - (robotPoints[i][0]*slope)
    #         xDist = robotPoints[i][0] - robotPoints[i][0]
    #         for j in range(0,100):
    #             increment = xDist/100
    #             xPoint = robotPoints[i][0] + (j*increment)
    #             yPoint = (xPoint*slope) + b
    #             for k in range(0,len(obstacles)):
    #                 for l in range(0,len(obstacles[k])):
    #                     if(abs(xPoint-obstacles[k][l][0]) < 0.2):
    #                         if(abs(yPoint-obstacles[k][l][1]) < 0.2):
    #                             return False
    return True


'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    # print "running RRT"
    points = dict()
    tree = dict()
    path = []

    # Your code goes here.
    #psuedo code - 
    #1 - start first point
    #2 - RRT stuff
    #3 - find closest point (including linepoint)
    #4 - check if collision
    #5 if no collision, add, otherwise new sample
    ##6 - after ten or so runs check goal 

    #inserting starting point
    points[1] = startPoint
    goalFound = False
    checkEvery = 10
    sigFig = 4
    count = 1
    goalIndex = 1

    #RRT stuff starts here
    while not goalFound:
        # length = len(points)
        # print "running " + str(count)
        minDist = 10*sqrt(2)
        minIndex = -1
        minIndexLine = -1
        withLine = False
        goalCheck = False

        #determining new point to check - 
        if count%checkEvery == 0:
            randPoint = goalPoint
            goalCheck = True
        else:
            x = np.random.uniform(0,10)
            y = np.random.uniform(0,10)
            randPoint = (round(x, sigFig), round(y, sigFig))

        #checking for closest point in the Tree newPoints
        for j in range(1, len(points)+1):
            pointDist = distance(randPoint, points[j])
            #if the distance is now closer update to new nearest point
            if pointDist < minDist:
                minDist = pointDist
                minIndex = j

        #checking against lines formed by the adjacency list, with the random Point
        # #won't enter if there's only one other point in the Tree
        '''
        This is line intersecting checking - need to get this working
        '''
        for j in range(1, len(tree)+1):
            #the adjacent vertecies
            branch = tree[j]
            # #checking the lines formed by a branch
            for k in range(len(branch)):
                start = points[j]
                key = branch[k]
                if start != points[key]:
                    lineDist, point = distanceToLine(start, points[key], randPoint)
                    buff = .01

                    #if the lineDist is smaller than the buffer
                    if minDist - lineDist > buff and lineDist != 0:
                        #checking to make sure its on the relevant parts of the line
                        left = min(start[0], points[key][0])
                        right = max(start[0], points[key][0])

                        if left < randPoint[0] and randPoint[0] < right:
                            withLine = True
                            minDist = lineDist

                            linePoint = point
                            minIndex = j
                            minIndexLine = key


        #closest point found is on a line formed by the tree
        '''Need to check if collision'''
        collisionFree = True
        if withLine:
            #number of steps along path to check
            numSteps = 20
            for i in range(1,numSteps):
                #incrementing along line to check collisions
                stepSize = round(minDist/numSteps, sigFig)*i
                xChange = (stepSize/minDist)*(randPoint[0]-linePoint[0])
                yChange = (stepSize/minDist)*(randPoint[1]-linePoint[1])
                newX = linePoint[0] + xChange
                newY = linePoint[1] + yChange
                pointCheck = (round(newX, sigFig),round(newY, sigFig))

                #checking if collision free
                collisionFree = isCollisionFree(robot, pointCheck, obstacles)

                #if a collision is found
                if not collisionFree:
                    # print "we are breaking here??"
                    # print "collision detected" + str(count)
                    break
                # if not collisionFree:
                    # break

            if collisionFree:
                # print "adding in " + str(count)
                #adding in the point on the line
                points[len(points)+1] = linePoint
                #updateAlM special for removing those two from the adjListMap??
                #connecting it to both ends of the line
                tree = updateALMcut(points[minIndex], linePoint, points[minIndexLine], points, tree)

                pointToAdd = randPoint
                points[len(points)+1] = pointToAdd
                tree = updateALM(linePoint, pointToAdd, points, tree)

                #for visualization
                # testLinePoints.append(linePoint)

                if goalCheck:
                    goalFound = True
                    break

        else:
            #number of steps along path to check
            numSteps = 20
            for i in range(1, numSteps):
                #incrementing along the line to check collisions
                stepSize = round(minDist/numSteps, sigFig)*i
                xChange = (stepSize/minDist)*(randPoint[0]-points[minIndex][0])
                yChange = (stepSize/minDist)*(randPoint[1]-points[minIndex][1])
                newX = points[minIndex][0] + xChange
                newY = points[minIndex][1] + yChange
                pointCheck = (round(newX, sigFig),round(newY, sigFig))

                #checking if collision free
                collisionFree = isCollisionFree(robot, pointCheck, obstacles)

                #if a collision is found
                if not collisionFree:
                    # print "BE:LHSD:LFKJSD"
                    # print "COllision detected" + str(count)
                    break
            
            if collisionFree:
                # print "adding in " + str(count)
                pointToAdd = randPoint
                points[len(points)+1] = pointToAdd
                tree = updateALM(points[minIndex], pointToAdd, points, tree)

                if goalCheck:
                    goalFound = True
                    break

        count +=1

    # print tree, count
    #find path
    path = basicSearch(tree, 1, len(points))
    # print "path is", path

    return points, tree, path
    # print points
    # print tree
    # print path



    points 
if __name__ == "__main__":

    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    robot = []
    obstacles = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            xy = xys[p].split(',')
            polygon.append((float(xy[0]), float(xy[1])))
        if line == 0 :
            robot = polygon
        else:
            obstacles.append(polygon)

    # Print out the data
    print "Robot:"
    print str(robot)
    print "Pologonal obstacles:"
    for p in range(0, len(obstacles)):
        print str(obstacles[p])
    print ""

    # Visualize
    robotStart = []
    robotGoal = []

    def start((x,y)):
        return (x+x1, y+y1)
    def goal((x,y)):
        return (x+x2, y+y2)
    robotStart = map(start, robot)
    robotGoal = map(goal, robot)
    drawProblem(robotStart, robotGoal, obstacles)

    # Example points for calling growSimpleRRT
    # You should expect many mroe points, e.g., 200-500
    points = dict()
    points[1] = (5, 5)
    points[2] = (7, 8.2)
    points[3] = (6.5, 5.2)
    points[4] = (0.3, 4)
    points[5] = (6, 3.7)
    points[6] = (9.7, 6.4)
    points[7] = (4.4, 2.8)
    points[8] = (9.1, 3.1)
    points[9] = (8.1, 6.5)
    points[10] = (0.7, 5.4)
    points[11] = (5.1, 3.9)
    points[12] = (2, 6)
    points[13] = (0.5, 6.7)
    points[14] = (8.3, 2.1)
    points[15] = (7.7, 6.3)
    points[16] = (7.9, 5)
    points[17] = (4.8, 6.1)
    points[18] = (3.2, 9.3)
    points[19] = (7.3, 5.8)
    points[20] = (9, 0.6)

    # Printing the points
    print ""
    print "The input points are:"
    print str(points)
    print ""

    # # I ADDED THIS IN
    # og_points = dict()
    # og_points = points
    # # print "og points"
    # # print og_points
    # ########

    # points, adjListMap, testLinePoints = growSimpleRRT(points)

    # # Search for a solution
    # path = basicSearch(adjListMap, 1, 20)

    # # Your visualization code
    # displayRRTandPath(points, adjListMap, path, og_points, testLinePoints)
    # displayRRTandPath()

    # # Solve a real RRT problem
    # # RRT(robot, obstacles, (x1, y1), (x2, y2))

    # randPoints, tree, path = RRT(robot, obstacles, (x1, y1), (x2, y2))

    # # Your visualization code
    # # displayRRTandPath(points, adjListMap, path, og_points, testLinePoints, robotStart, robotGoal, obstacles)
    # # displayRRTandPath(randPoints, tree, path, og_points, testLinePoints, robotStart, robotGoal, obstacles)
    
    points, adjListMap = growSimpleRRT(points)

    # Search for a solution  
    path = basicSearch(adjListMap, 1, 20)    

    # Your visualization code 
    displayRRTandPath(points, adjListMap, path) 

    # Solve a real RRT problem
    RRT(robot, obstacles, (x1, y1), (x2, y2))
    
    # Your visualization code 
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles) 