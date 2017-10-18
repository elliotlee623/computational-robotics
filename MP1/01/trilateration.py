import sys
import numpy as np
from math import *
# Takes in a list of lists, each list is [x,y,z,d]
def trilaterate3D(distances):
    vect1 = np.array(distances[0])
    vect2 = np.array(distances[1])
    vect3 = np.array(distances[2])
    vect4 = np.array(distances[3])

    # #creating the translation vector to set the first vector at the origin
    # ox = vect1[0]*-1
    # oy = vect1[1]*-1
    # oz = vect1[2]*-2
    # translate = np.array([ox, oy, oz, 0])

    # #translating all vectors in relation, treating the first vector as origin
    # vect1 = vect1 + translate
    # vect2 = vect2 + translate
    # vect3 = vect3 + translate
    # vect4 = vect4 + translate
    dist = [vect1[3], vect2[3], vect3[3]]

    vect1 = vect1[0:3]
    vect2 = vect2[0:3]
    vect3 = vect3[0:3]

    #transform circle 1 to 0
    ox = (vect2-vect1)/np.linalg.norm(vect2-vect1)
    i = np.dot(ox, vect3-vect1)
    oy = (vect3-vect1-i*ox)/(np.linalg.norm(vect3-vect1-i*ox))
    oz = np.cross(ox,oy)
    d = np.linalg.norm(vect2 - vect1)
    j = np.dot(oy, vect3 - vect1)
    #compute trilaterization

    x = (pow(dist[0],2) - pow(dist[1],2) + pow(d,2))/(2*d)
    y = ((pow(dist[0],2) - pow(dist[2],2) + pow(i,2) + pow(j,2))/(2*j)) - ((i/j)*x)
    z = sqrt(pow(dist[0],2) - pow(x,2) - pow(y,2))

    print x,y,z
    #returns local point
    return [x,y,z]

if __name__ == "__main__":
    
    # Retrive file name for input data
    if(len(sys.argv) == 1):
        print "Please enter data file name."
        exit()
    
    filename = sys.argv[1]

    # Read data
    lines = [line.rstrip('\n') for line in open(filename)]
    distances = []
    for line in range(0, len(lines)):
        distances.append(map(float, lines[line].split(' ')))

    # Print out the data
    print "The input four points and distances, in the format of [x, y, z, d], are:"
    for p in range(0, len(distances)):
        print distances[p] 

    # Call the function and compute the location 
    location = trilaterate3D(distances)
    print 
    print "The location of the point is: " + str(location)
