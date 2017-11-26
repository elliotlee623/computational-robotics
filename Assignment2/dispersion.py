import sys
import random
import numpy as np
import matplotlib.pyplot as plt]

LT = x,y
RT = x + diffx, y
RB = x + diffx, y - diffy
LB = x, y - diffy

#returns a 4 coordinate array, with the values
#of a square, from LT, RT, RB, LB
def makesquare(x,y,diffx,diffy):
	#LT of square
	square = []
	temp = [x,y]
	square.append(temp)
	#RT of square
	temp = [x + diffx, y]
	square.append(temp)
	#RB of square
	temp = [x + diffx, y - diffx]
	square.append(temp)
	#LB of square
	temp = [x, y - diffx]
	square.append(temp)
	return square



if __name__ == "__main__":

	#k = number of random samples
	k = 5
	tol = .0001
	samplesx = []
	samplesy = []

	for i in range(k):
		x = random.uniform(0.,1.)
		y = random.uniform(0.,1.)
		samplesx.append(x)
		samplesy.append(y)
	squareSize = []
	for i in range(k):
		diffx = 1-samplesx[i]
		diffy = abs(0-samplesy[i])
		squareSize.append([diffx,diffy])
		#testing each point
		for j in range(k-1):
			square = makesquare(samplesx[i],samplesy[i],diffx,diffy)
			LT = square[0]
			RT = square[1]
			RB = square[2]
			LB = square[3]

			#checking if its in the x bounds
			if LT[0] < samplesx[j+1] && samplesx[j+1] < RT[0]:
				#checking if its in the y bounds
				if LT[1] > samplesy[j+1] && samplesy[j+1] > LB[1]:
					diffx = samplesx[j+1]-LT[0]
					diffy = LT[1] - samplesy[j+1]

			#updating size of largest square
			squareSize[i] = [diffx,diffy]

	point = 0
	biggest = 0
	for i in range(len(squareSize)):

		#finding smallest side of square
		if squareSize[i][0] < squareSize[i][1]:
			limit = squareSize[i][0]
		else:
			limit = squareSize[i][1]

		#updating biggest square
		if limit > biggest:
			biggest = limit
			point = i

Samplesx[point]
Samplesy[point]
Biggest #diffx/diffy







	plt.plot(samplesx, samplesy, '-ro')
	plt.show()
