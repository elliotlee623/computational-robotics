import sys
import numpy
import matplotlib

# Takes in a list of lists, each list is [x,y,z,d]
def trilaterate3D(distances):
    #navigate through list
    #compute trilaterization
    #returns local point
    return [0.,0.,0.,0.]

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
