#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 23:58:51 2022

@author: ezoboli
"""

import numpy as np
import cv2
import math
import sys

global image
nodeCount = 0
OpenList = []
ClosedList = []
ClosedListPoints = []
solution = []
allNodes = []
allCosts = []
nodesData = []
tempList = []
tempList2 = []
tempID = 0

# create list with all points on the map
for x in range(0, 401):
    for y in range(0, 251):
        allNodes.append((x,y))
        nodesData.append(list(((x,y), 0)))
        allCosts.append(math.inf)


# function that checks if a point is inside the obstacles or not.
def checkDistance(point):
    x = point[0]
    y = point[1]
    
    #Polygon
    L1 = -0.316*x+71.148-y
    L2 = -0.714*x+128.29-y
    L3 = 3.263*x-213.15-y
    L4 = 1.182*x+30.176-y
    L5 = 0.0766*x+60.405-y
    
    #Hexagon
    Lh1 = -0.577*x+219.28-y
    Lh2 = 0.577*x-11.658-y
    Lh3 = 240-x
    Lh4 = -0.577*x+311.66-y
    Lh5 = 0.577*x+80.718-y
    Lh6 = 160-x
    
    #Circle
    Lc = (x-300)**2 + (y-65)**2 - 45**2
    
    #check for upper polygon
    if L1<=0 and L2>=0 and L5>=0:
        return 1
    #check for lower polygon
    if L3<=0 and L4>=0 and L5<=0:
        return 1
    #check for hexagon
    elif Lh1<=0 and Lh2<=0 and Lh3>=0 and Lh4>=0 and Lh5>=0 and Lh6<=0:
        return 1
    #check for circle
    elif Lc<0:
        return 1
    #check for border
    elif x <= 4 or x>= 398 or y <= 5 or y >= 245:
        return 1
    else:
        return 0

#Create the plot with nothing going on
def plotWorkspace():
    global image
    image = 255*np.ones((251, 401, 3), np.uint8)
    poly_points = np.array([[27, 63], [144, 26], [86, 67], [117, 168]],dtype=np.int32)
    hex_points = np.array([[160,127], [200,103], [240,127], [240,173], [200,196], [160,173]],dtype=np.int32)
    cv2.fillConvexPoly(image,poly_points, 0)
    cv2.fillConvexPoly(image,hex_points, 0)
    cv2.circle(image, (300, 65), 45, (0, 0, 0),-1)
    #resized = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    return image

#function to take user input
def userInput():
    try:
        start_x = int(input('Enter starting x position: '))
        if start_x < 0 :
            print("Invalid starting x position, setting x position to 0")
            start_x = 0
        elif start_x >400:
            print("Invalid starting x position, setting x position to 400")
            start_x = 400
    
        start_y = 250 - int(input('Enter starting y position: '))
        if start_y< 0 :
            print("Invalid starting y position, setting y position to 0")
            start_y = 0
        elif start_y >250:
            print("Invalid starting y position, setting y position to 250")
            start_y = 250
    
        goal_x = int(input('Enter goal x position: '))
        if goal_x < 0 :
            print("Invalid goal x position, setting x position to 0")
            goal_x = 0
        elif goal_x >400:
            print("Invalid goal x position, setting x position to 400")
            goal_x = 400
    
        goal_y = 250 - int(input('Enter goal y position: '))
        if goal_y < 0 :
            print("Invalid goal y position, setting y position to 0")
            goal_y = 0
        elif goal_y >250:
            print("Invalid goal y position, setting y position to 250")
            goal_y = 250
    except:
        print("Invalid Input, closing program")
        sys.exit()
        
    if checkDistance((start_x, start_y)) == 1:
        print("start is inside an obstacle, please try again")
        sys.exit()
            
    if checkDistance((goal_x, goal_y)) == 1:
        print("goal is inside an obstacle, please try again")
        sys.exit()
    
    return (start_x, start_y), (goal_x, goal_y)

#function to find element in 2d list
def find(lis, m):
    for x, new_val in enumerate(lis):
        try:
            y = new_val.index(m)
        except ValueError:
            continue
        yield x, y
        
#General moving function, the inputs are the current node, change in x, change in y, and cost
def move(currentNode, dx, dy, cost):
    x = nodesData[currentNode][0][0] + dx
    y = nodesData[currentNode][0][1] + dy
    if x < 0 or y < 0:
        x = 0
        y = 0
    parentID = currentNode
    ownID = allNodes.index((x,y))
    cost = allCosts[currentNode] + cost
    
    if checkDistance((x,y)) == 0:
        if ownID not in ClosedList:
            image[y, x] = [255,255,0]
            resized = cv2.resize(image, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
            cv2.imshow("Nodes",resized)
            cv2.waitKey(1)
            if cost < allCosts[ownID]:
                    allCosts[ownID] = cost
                    nodesData[ownID][1] = parentID
            if ownID not in OpenList:
                OpenList.append(ownID)

#================================================================================================================================

start, goal = userInput()
#start, goal = (15, 15), (40,40)
# Point, parentID, ownID, cost
startID = allNodes.index(start)
endID = allNodes.index(goal)
allCosts[startID] = 0

plotWorkspace()

#create first set of movements
#up
move(startID, 0, -1, 1)
#up right
move(startID, 1, -1, 1.4)
#right
move(startID, 1, 0, 1)
#down right
move(startID, 1, 1, 1.4)
#down
move(startID, 0, 1, 1)
#down left
move(startID, -1, 1, 1.4)
#left
move(startID, -1, 0, 1)


while 1:
    
    # for every element in the OpenList, find the corresponding cost
    for i in OpenList:
        icost = allCosts[i]
        # use the two temporary lists to store cost and ID
        if i not in tempList2:
            tempList.append(icost)
            tempList2.append(i)
            
    #set the new node to be the node with lowestcost
    currentNode = tempList2[tempList.index(min(tempList))]
    OpenList.remove(currentNode)
    
    #empty the temporary lists
    tempList = []
    tempList2 = []
    
    # if we reached the end node then start backtracking
    if currentNode == endID:
        backTrack = currentNode
        while backTrack != startID:
            (x,y), parentID = nodesData[backTrack]
            solution.append((x,y))
            backTrack = parentID
        
        solution.reverse()
        
        #graph the solution
        for i in solution:
            (x,y) = i
            image[y, x] = [255,0,255]
            resized = cv2.resize(image, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
            cv2.imshow("Nodes",resized)
            cv2.waitKey(50)
            
        break
    
    # if we haven't reached the solution then generate new set of points
    else:
        #up
        move(currentNode, 0, -1, 1)
        #up right
        move(currentNode, 1, -1, 1.4)
        #right
        move(currentNode, 1, 0, 1)
        #down right
        move(currentNode, 1, 1, 1.4)
        #down
        move(currentNode, 0, 1, 1)
        #down left
        move(currentNode, -1, 1, 1.4)
        #left
        move(currentNode, -1, 0, 1)
        #up Left
        move(currentNode, -1, -1, 1.4)
        
        # put the currentNode in the closed list so that it won't be analyzed anymore
        if currentNode not in ClosedList:
            ClosedList.append(currentNode)
        























