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
        #if on same poly
        for j in range(len(tempPoly)):
            if tempPoly[j] == reflexVertices[count]:
                #makes sure its not the last vertex of that polygon
                if j == len(tempPoly-1):
                #checks if its consecutive with end
                    if tempPoly[0] = reflexVertices[count-j]:
                        #add to ALM
                #check to see if consecutive reflex vertex
                else:
                    if tempPoly[j+1] == reflexVertices[counts+1]:
                        #add to ALM


    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    path = []
    pathLength = 0
    
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
