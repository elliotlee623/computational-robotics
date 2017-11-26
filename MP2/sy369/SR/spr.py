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

#adds points to the graph in order based on length from increasing to decreasing
def addToGraph(vectDistances, possibleVects, path):
    if possibleVects == None:
        return vectDistances
    #for all adjacent vectors of a specified vector
    for i in range(len(possibleVects)):
        inserted = False
        #checks to see if its been visited
        for j in range(len(path)):
            if possibleVects[i][0] == path[j]:
                inserted = True
                break
        #checks to see if its already in the list
        for j in range(len(vectDistances)):
            if possibleVects[i][0] == vectDistances[j][0]:
                #replacing it if better value found
                if possibleVects[i][1] < vectDistances[j][1]:
                    vectDistances[j][1] = possibleVects[i][1]
                    inserted = True
                    break
        #adds it in if not already in
        for j in range(len(vectDistances)):
            if possibleVects[i][1] < vectDistances[j][1]:
                vectDistances.insert(j-1, possibleVects[i])
                inserted = True
                break
        #if it is the largest value possible, adds it in at the end
        if not inserted:
            vectDistances.append(possibleVects[i])
    return vectDistances    

#find if two lines are intersecting, line formed by point1 and point2, compared with point3 and point4
#returns false if not intersecting, returns true if intersecting
# def intersecting(point1, point2, point3, point4):
#     xdiff = (point1[0] - point2[0], point3[0] - point4[0])
#     ydiff = (point1[1] - point2[1], point3[1] - point4[1])

#     def det(a, b):
#         return a[0] * b[1] - a[1] * b[0]

#     div = det(xdiff, ydiff)
#     if div == 0:
#         return False

#     return True

# def bitangents(polygons, vertexMap, adjListMap):
#     print len(vertexMap)
#     vMap = []
#     for i in range(len(vertexMap)):
#         vMap.append(vertexMap.get(i+1))

#     while(len(vMap) > 1):
#         p1 = vMap.pop(0)
#         tempMap = vMap
#         #comparing for each posible vertex
#         for i in range(len(tempMap)):
#             intersect = False
#             p2 = tempMap.pop(0)
#             #comparing against each posible polygon
#             for j in range(len(polygons)):
#                 points = polygons[j]
#                 #each of the lines in the polygon
#                 for k in range(len(points)-1):
#                     p3 = points[k]
#                     p4 = points[k+1]
#                     print p1, p2, p3, p4
#                     intersect = intersecting(p1, p2, p3, p4)
#                     if intersect:
#                         break
#                 #completing loop of polygon
#                 p3 = points[len(points)-1]
#                 p4 = points[0]
#                 intersect = intersecting(p1, p2, p3, p4)
#                 if intersect:
#                     break
#                 #make sure to loop back
#             # no intersection found, can be added to ALM
#             if not intersect:
#                 adjListMap = updateALM(p1, p2, vertexMap, adjListMap)

#     return adjListMap


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
        # nextPoly = polygons[i+1]
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

    # adjacencyListMap = bitangents(polygons, vertexMap, adjacencyListMap)
                
    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    vectDistances = []
    start = 3
    path.append(start)
    vectDistances = addToGraph(vectDistances, adjListMap.get(start, None), path)
    key = 0
    goal = 7
    while(key != goal):
        vertex = vectDistances.pop(0)
        key = vertex[0]
        path.append(key)
        pathLength += vertex[1]
        vectDistances = addToGraph(vectDistances, adjListMap.get(key, None), path)
        if len(vectDistances) == 0 and key != goal:
            pathLength = -1
            print "No path found"
            break
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

    updatedALMap = adjListMap
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
        print "Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]"
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
    print "Pologonal obstacles:"
    for p in range(0, len(polygons)):
        print str(polygons[p])
    print ""

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print "Reflexive vertices:"
    print str(reflexVertices)
    print ""

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print "Vertex map:"
    print str(vertexMap)
    print ""
    print "Base roadmap:"
    print str(adjListMap)
    print ""

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print "Updated roadmap:"
    print str(updatedALMap)
    print ""

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start, goal)
    print "Final path:"
    print str(path)
    print "Final path length:" + str(length)
    

    # Extra visualization elements goes here
