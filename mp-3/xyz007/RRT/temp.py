import sys
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import ConnectionPatch
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
Make a patch for a single pologyn
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
    print "polygon patch"
    # print verts
    # print codes
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    # print path
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
def updateALM(point1, point2, vertexMap, adjacencyListMap):

    # def distance(point1, point2):
    #     dist = sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
    #     return round(dist,3)

    #finding indexes for the two points
    for i in range(len(vertexMap)):
        if point1 == vertexMap.get(i+1):
            p1 = i+1

    p2 = len(vertexMap)
    temp1 = adjacencyListMap.get(p1, None)
    temp2 = adjacencyListMap.get(p2, None)

    if temp1 == None:
        temp1 = []
    if temp2 == None:
        temp2 = []

    #updating lists for ALM
    add = True
    for i in range(len(temp1)):
        if temp1[i] == [p2]:
            add = False
            break
    if add:
        temp1.append(p2)
        temp2.append(p1)
    #updating ALM
    adjacencyListMap[p1] = temp1
    adjacencyListMap[p2] = temp2
    return adjacencyListMap

'''
Grow a simple RRT
'''
def growSimpleRRT(points):
    newPoints = dict()
    adjListMap = dict()
    #make list and insert into dictionary at end
    newPointsList = []

    #p1 and p2 form line, p3 is point we are tring to find perpendicular to line
    def distanceToLine(p1, p2, p3):
        x_diff = p2[0] - p1[0]
        y_diff = p2[1] - p1[1]
        x_diff = round(x_diff, 8)
        y_diff = round(y_diff, 8)
        num = abs(y_diff*p3[0] - x_diff*p3[1] + p2[0]*p1[1] - p2[1]*p1[0])
        den = sqrt(y_diff**2 + x_diff**2)
        if den == 0:
            return 0

        return num/den

    # print "grow simple"
    newPointsList.append(points[1])
    newPoints[1] = points[1]
    # Your code goes here
    indexMin = 0
    withLine = False

    for i in range(2, len(points)):
        minDist = 10*sqrt(2)
        linePoint = (0,0)
        for j in range(0, len(newPointsList)):
            pointDist = hypot(points[i][0] - newPointsList[j][0], points[i][1] - newPointsList[j][1])
            #if find a closer point
            if pointDist < minDist:
                minDist = pointDist
                withLine = False
                indexMin = j
            #check if able to create line from points
            if len(newPointsList) > 1:
                #checking to make sure not at last point
                if j != len(newPointsList)-1:
                    lineDist = distanceToLine(newPointsList[j], newPointsList[j+1], points[i])
                    if lineDist < minDist and lineDist != 0:
                        withLine = True
                        indexMin = j
                        #calculate the point
                        x1 = points[i][0]
                        y1 = points[i][1]
                        x2 = newPointsList[j][0]
                        y2 = newPointsList[j][1]
                        x3 = newPointsList[j+1][0]
                        y3 = newPointsList[j+1][1] 
                        k = ((y2-y1) * (x3-x1) - (x2-x1) * (y3-y1)) / ((y2-y1)**2 + (x2-x1)**2)
                        x4 = x3 - k * (y2-y1)
                        y4 = y3 + k * (x2-x1)
                        linePoint = (round(x4,4),round(y4,4))

        #if no line can be formed
        if len(newPointsList) > 2:
            if withLine:
                #connect to both end points
                newPointsList.append(linePoint)
                newPoints[len(newPoints)+1] = linePoint
                #call update twice, for each end point
                adjListMap = updateALM(newPointsList[indexMin], linePoint, newPoints, adjListMap)
                adjListMap = updateALM(newPointsList[indexMin+1], linePoint, newPoints, adjListMap)

                #connecting line point to new point
                newPointsList.append(points[i])
                newPoints[len(newPoints)+1] = points[i]
                #call update once to add
                adjListMap = updateALM(linePoint, points[i], newPoints, adjListMap)
                withLine = False
            else:
                newPointsList.append(points[i])
                newPoints[len(newPoints)+1] = points[i]
                #call update once to add
                adjListMap = updateALM(newPointsList[indexMin], points[i], newPoints, adjListMap)
        else:
            #if there is only one other point present
            newPointsList.append(points[i])
            newPoints[len(newPoints)+1] = points[i]
            #call update once to add
            adjListMap = updateALM(newPointsList[indexMin], points[i], newPoints, adjListMap)

    # print newPoints
    # print adjListMap

    return newPoints, adjListMap

'''
Perform basic search
'''
def basicSearch(tree, start, goal):
    path = []

    # temp = tree.get(start)
    # print "This is temp" 
    # print temp
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.

    return path

def createRRTPatch(branch, points, startPoint):
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
    patch = patches.PathPatch(path, facecolor='black', lw=1)
    return patch

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
    for i in range(1, len(tree)+1):
        branch = tree[i]
        startPoint = points[i]
        patch = createRRTPatch(branch, points, startPoint)
        ax.add_patch(patch)

    ax.set_xlim(0,10)
    ax.set_ylim(0,10)
    plt.show()
    return

'''
Collision checking
'''
def isCollisionFree(robot, point, obstacles):

    # Your code goes here.

    for i in range(0,len(robot)):
        robot[i][0] = robot[i][0] + point[0]
        robot[i][1] = robot[i][1] + point[1]
    if(robot[i][0] >= 10 or robot[i][1] >= 10):
        return False
    for i in range(0,len(robot)):
        if(i != len(robot)-1):
            slope =(robot[i+1][1]-robot[i][1])/(robot[i+1][0]-robot[i][0])
            b = robot[i][1] - (robot[i][0]*slope)
            xDist = robot[i+1][0] - robot[i][0]
            for j in range(0,100):
                xPoint = robot[i][0] + (xDist/j)
                yPoint = (xPoint*slope) + b
                for k in range(0,len(obstacles)):
                    if(abs(xPoint-obstacles[k][0]) < 0.2):
                        if(abs(yPoint-obstacles[k][1]) < 0.2):
                            return False
        slope = (robot[0][1] - robot[i+1][1])/(robot[0][0]-robot[i+1][0])
        b = robot[i+1][1] - (robot[i+1][0]*slope)
        xDist = robot[i+1][0] - robot[i][0]
        for j in range(0,10):
            xPoint = robot[i][0] + (xDist/j)
            yPoint = (xPoint*slope) + b
            for k in range(0,len(obstacles)):
                if(abs(xPoint-obstacles[k][0]) < 0.2):
                    if(abs(yPoint-obstacles[k][1]) < 0.2):
                        return False
    return True

'''
The full RRT algorithm
'''
def RRT(robot, obstacles, startPoint, goalPoint):

    points = dict()
    tree = dict()
    path = []
    # Your code goes here.

    return points, tree, path

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

    points, adjListMap = growSimpleRRT(points)

    # Search for a solution
    path = basicSearch(adjListMap, 1, 20)

    # Your visualization code
    displayRRTandPath(points, adjListMap, path)

    # Solve a real RRT problem
    RRT(robot, obstacles, (x1, y1), (x2, y2))

    # Your visualization code
    displayRRTandPath(points, adjListMap, path, robotStart, robotGoal, obstacles)
