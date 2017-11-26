import sys
import math

'''
Report reflexive vertices
'''

## Helper Methods

def findAngle(x1, y1, x2, y2):
    #find slope
    m = (y2-y1)/(x2-x1)
    #angle of inclanation for each line (atan of slope)
    angle = math.degrees(math.atan(m))
    return angle

def polygonAngles(vertices):
    #iterate through all the points in the polygon
    temp = []
    for i in range(len(vertices) - 1):
        temp.append(findAngle(vertices[i][0], vertices[i][1], vertices[i+1][0], vertices[i+1][1]))

    #finding angle for the last vertex
    length = len(vertices)-1
    temp.append(findAngle(vertices[length][0], vertices[length][1], vertices[0][0], vertices[0][1]))

    #taking care of negative numbers
    for i in range(len(temp)):
        if temp[i] <= 0:
            temp[i] = -temp[i]

    #finding angles in the polygon
    angles = []
    angles.append(temp[len(temp)-1]+temp[0])
    for i in range(len(temp)-1):
        angles.append(temp[i+1]+temp[i])

    return angles

#calculates distance between two points
def distance(point1, point2):
    dist = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
    return round(dist,3)

def slope(point1, point2):
    m = (point2[0]-point1[0])/(point2[1]-point1[1])
    return m

# updates the adjacency linked map dictionary
def updateALM(point1, point2, vertexMap, adjacencyListMap):
    #finding indexes for the two points
    for i in range(len(vertexMap)):
        if point1 == vertexMap.get(i+1):
            p1 = i+1
        if point2 == vertexMap.get(i+1):
            p2 = i+1

    temp1 = adjacencyListMap.get(p1, None)
    temp2 = adjacencyListMap.get(p2, None)

    if temp1 == None:
        temp1 = []
    if temp2 == None:
        temp2 = []
    dist = distance(point1, point2)
    #updating lists for ALM
    temp1.append([p2,dist])
    temp2.append([p1,dist])
    #updating ALM
    adjacencyListMap[p1] = temp1
    adjacencyListMap[p2] = temp2
    return adjacencyListMap

#---------------------------------------------------------------#

def findReflexiveVertices(polygons):
    vertices=[]

    #know for sure first 2 vertices are correct
    # loop to iterate through all the polygons
    for i in range(len(polygons)):
        answer = polygonAngles(polygons[i])
        for j in range(len(answer)):
            if answer[j] > 90:
                vertices.append(polygons[i][j])

    return vertices

'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = dict()

    # build vertexMap dictionary
    for i in range(len(reflexVertices)):
        vertexMap[i+1] = reflexVertices[i]

    #checking if its on the same polygon
    count = 0
    for i in range(len(polygons)):
        #iterating through the various vertices of a polygon
        tempPoly = polygons[i]
        #next poly for comparison
        nextPoly = polygons[i+1]
        first = count
        #if on same poly
        for j in range(len(tempPoly)):
            if tempPoly[j] == reflexVertices[count]:
                #makes sure its not the last vertex of that polygon
                if j == len(tempPoly)-1:
                #checks if its consecutive with end
                    if tempPoly[0] == reflexVertices[first]:
                        adjacencyListMap = updateALM(tempPoly[len(tempPoly)-1], tempPoly[0], vertexMap, adjacencyListMap)
                else:
                    if tempPoly[j+1] == reflexVertices[count+1]:
                        adjacencyListMap = updateALM(tempPoly[j], tempPoly[j+1], vertexMap, adjacencyListMap)
                count+=1
        #not same poly
        for a in range(len(tempPoly)):
            for k in range(len(nextPoly)):
                for l in range(len(polygons)):
                    temp2 = polygons[l]
                    for m in range(len(polygons[l])):
                        #X1,X2,Y1,Y2 for the "line". extended by 100 because must be line segment
                        X1 = 100*tempPoly[j][0]
                        Y1 = 100*tempPoly[j][1]
                        X2 = 100*nextPoly[k][0]
                        Y2 = 100*nextPoly[k][1]
                        #X3,Y3,X4,Y4 are all the edges, iterated through in a loop
                        X3 = temp2[m][0]
                        Y3 = temp2[m][1]
                        X4 = temp2[m+1][0]
                        Y4 = temp2[m+1][1]

                        #define intersection interval
                        left = max(min(X1,X2),min(X3,X4))
                        right = min(max(X1,X2),max(X3,X4))
                        bottom = max(min(Y1,Y2),min(Y3,Y4))
                        top = min(max(Y1,Y2),max(Y2,Y3))

                        if(max(X1,X2) < min(X3,temp2[0][0])):
                            adjacencyListMap = updateALM(tempPoly[j], nextPoly[k], vertexMap, adjacencyListMap)
                        A1 = (Y1-Y2)/(X1-nextPoly[k][[0]])
                        A2 = (Y3-Y4)

                        if(A1==A2):
                            #parallel lines, do nothing
                        if((left == right) && (top==bottom)):
                            #intersection
                        else:
                            adjacencyListMap = updateALM(tempPoly[j], nextPoly[k], vertexMap, adjacencyListMap)

    return vertexMap, adjacencyListMap

'''
Perform uniform cost search
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0

    reached = False
    path.append(start)

    while(not reached)
    # Your code goes here. As the result, the function should
    # return a list of vertex labels, e.g.
    #
    # path = [23, 15, 9, ..., 37]
    #
    # in which 23 would be the label for the start and 37 the
    # label for the goal.

    return path, pathLength

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    # Your code goes here. Note that for convenience, we
    # let start and goal have vertex labels 0 and -1,
    # respectively. Make sure you use these as your labels
    # for the start and goal vertices in the shortest path
    # roadmap. Note that what you do here is similar to
    # when you construct the roadmap.

    return startLabel, goalLabel, updatedALMap

if __name__ == "__main__":

    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print ("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()

    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append(map(float, xys[p].split(',')))
        polygons.append(polygon)

    # Print out the data
    print("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print (str(polygons[p]))
    print ("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print ("Reflexive vertices:")
    print (str(reflexVertices))
    print ("")

    # Compute the roadmap
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print ("Vertex map:")
    print (str(vertexMap))
    print ("")
    print ("Base roadmap:")
    print (str(adjListMap))
    print ("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print ("Updated roadmap:")
    print (str(updatedALMap))
    print ("")

    # Search for a solution
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print ("Final path:")
    print (str(path))
    print ("Final path length:") + (str(length))


    # Extra visualization elements goes here
