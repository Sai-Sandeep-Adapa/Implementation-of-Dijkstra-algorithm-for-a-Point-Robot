"""
Author : Sai Sandeep Adapa (sadapa@umd.edu)

Brief : Implementation of Dijkstra algorithm for a Point Robot

""""
# Importing required Libraries
import sys
import cv2
import numpy as np
import heapq as hq
import time
import imutils


height_map = 250    
width_map = 400
clr_robot = 5    
obstacle_color = [0,0,255]    


def generateMap(height, width):

    map = np.empty((height, width, 3), dtype='uint8')

    for x in range(width):
        for y in range(height):
            
            if ((x-300)**2 + (y-65)**2 <= ((40+clr_robot)**2)):
                map[y][x] = obstacle_color
       
            if (y+(0.57*x)) >= 218.53 and (y-(0.57*x))>= -10.04 and (x<=240) and (y+(0.57*x))<=310.04 and (y-(0.57*x)) <= 81.465 and (x >= 160):
                map[y][x] = obstacle_color

            if ((y+(0.316*x) >= 71.1483) and (y+(0.857*x) <= 145.156) and (y-(0.114*x) <= 60.909)) or ((y-(1.23*x)) <= 28.576 and (y-(3.2*x)) >= -202.763 and (y-(0.114*x)) >= 60.909):
                map[y][x] = obstacle_color

            if (x<=clr_robot) or (x>=(400-clr_robot)) or (y<=clr_robot) or (y>=(250-clr_robot)):
                map[y][x] = obstacle_color  
    
    return map
 

def checkInputFeasibility(x_start, y_start, x_goal, y_goal, map):
 
    input_flag = True

    if isObstacle(map, y_start, x_start):
        print("!! Start Position is in an Obstacle/Wall, try again!")
        input_flag = False
    if isObstacle(map, y_goal, x_goal):
        print("!! Goal Position is in an Obstacle/Wall!, try again")
        input_flag = False
    
    return input_flag


def isObstacle(map, i, j):
 
    if (map[i][j][2] < obstacle_color[2]):# and map[i][j][1] is not obstacle_color[1]):
        return False
    else:
        return True


def isGoalNode(CurrentNode, goalNode):

    if list(CurrentNode) == goalNode:
        return True
    else:
        return False


def actionMoveTop(CurrentNode,map):

    NextNode = CurrentNode.copy()
    if(NextNode[1]-1 > 0) and (not isObstacle(map, NextNode[1]-1, NextNode[0])):
        Status = True
        NextNode[1] = NextNode[1] - 1 
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveTopRight(CurrentNode,map):

    NextNode = CurrentNode.copy()
    
    if(NextNode[1]-1 > 0) and (NextNode[0]+1 <map.shape[1]) and (not isObstacle(map, NextNode[1]-1, NextNode[0])):
        Status = True
        NextNode[0] = NextNode[0] + 1 
        NextNode[1] = NextNode[1] - 1
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveRight(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[0]+1 <map.shape[1]) and (not isObstacle(map, NextNode[1], NextNode[0]+1)):
        Status = True
        NextNode[0] = NextNode[0] + 1 
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveBottomRight(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]+1 < map.shape[0]) and (NextNode[0]+1 <map.shape[1]) and (not isObstacle(map, NextNode[1]+1, NextNode[0]+1)):
        Status = True
        NextNode[0] = NextNode[0] + 1 
        NextNode[1] = NextNode[1] + 1
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveBottom(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]+1 < map.shape[0]) and (not isObstacle(map, NextNode[1]+1, NextNode[0])):
        Status = True 
        NextNode[1] = NextNode[1] + 1
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveBottomLeft(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]+1 < map.shape[0]) and (NextNode[0]-1 >0) and (not isObstacle(map, NextNode[1]+1, NextNode[0]-1)):
        Status = True 
        NextNode[0] = NextNode[0] - 1
        NextNode[1] = NextNode[1] + 1
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveLeft(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[0]-1 > 0) and (not isObstacle(map, NextNode[1], NextNode[0]-1)):  
        Status = True 
        NextNode[0] = NextNode[0] - 1
    else:
        Status = False   

    return (Status, NextNode)


def actionMoveTopLeft(CurrentNode,map):
    
    NextNode = CurrentNode.copy()
    if(NextNode[1]-1 > 0) and (NextNode[0]-1 > 0) and (not isObstacle(map, NextNode[1]-1, NextNode[0]-1)):
        Status = True 
        NextNode[0] = NextNode[0] - 1
        NextNode[1] = NextNode[1] - 1
    else:
        Status = False   

    return (Status, NextNode)


def findDijkstraPath(startNode, goalNode, map):
    
    closed_list = {}    
    opened_list = []    
    

    hq.heapify(opened_list)
    hq.heappush(opened_list, [0, startNode, startNode])
    
    start_time = time.time()
    while True:
        
        if (len(opened_list) > 0):
            
            explored_node = hq.heappop(opened_list)
            cost_to_come, present_node, parent_node = explored_node[0], explored_node[1], explored_node[2]
           
            closed_list[(present_node[0],present_node[1])] = parent_node
            
            if isGoalNode(present_node, goalNode):
                print("\n Goal reached! ***")
                end_time = time.time()
                print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                backTracking(goalNode,startNode,closed_list,map)
                return True

            else:

                # TOP 
                flag, child_node = actionMoveTop(present_node,map)    
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):     
                        cost = cost_to_come + 1
                        child_node = list(child_node)
                        closelist_flag = False    

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True        
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        
                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal Preached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        backTracking(goalNode,startNode,closed_list,map)
                        return True

                
                # TOP RIGHT
                flag, child_node = actionMoveTopRight(present_node,map)    
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False   
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):   
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        backTracking(goalNode,startNode,closed_list,map)
                        return True

                # RIGHT      
                flag, child_node = actionMoveRight(present_node,map)    
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False    
                        cost = cost_to_come+1
                        child_node = list(child_node)
                        
                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):  
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        backTracking(goalNode,startNode,closed_list,map)
                        return True

                
                # BOTTOM RIGHT
                flag, child_node = actionMoveBottomRight(present_node,map) 
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False    
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):  
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):   
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        backTracking(goalNode,startNode,closed_list,map)
                        return True
                

                # BOTTOM
                flag,child_node = actionMoveBottom(present_node,map)   
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False 
                        cost = cost_to_come + 1
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        if(not closelist_flag):  
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        backTracking(goalNode,startNode,closed_list,map)
                        return True
                

                # BOTTOM LEFT
                flag, child_node = actionMoveBottomLeft(present_node,map)  
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False  
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True        
                                if(node[0] > cost):   
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        closed_list[child_node] = present_node
                        backTracking(goalNode,startNode,closed_list,map)
                        return True
                

                # LEFT
                flag,child_node = actionMoveLeft(present_node,map)  
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False 
                        cost = cost_to_come + 1
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                if(node[0] > cost):    
                                    node[0] = cost
                                    node[2] = present_node
                                break
                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        backTracking(goalNode,startNode,closed_list,map)
                        return True                


                # TOP LEFT
                flag,child_node = actionMoveTopLeft(present_node,map)   
                child_node = tuple(child_node)

                if(flag is True and child_node not in closed_list):
                    if not isGoalNode(child_node, goalNode):
                        closelist_flag = False    
                        cost = cost_to_come + 1.4
                        child_node = list(child_node)

                        for node in opened_list:
                            if(node[1] == child_node):
                                closelist_flag = True
                                cost = cost_to_come + 1.4
                                if(node[0] > cost):  
                                    node[0] = cost
                                    node[2] = present_node
                                break

                        if(not closelist_flag):    
                            hq.heappush(opened_list,[cost, child_node, present_node])
                    else:
                        print("\nGoal reached!")
                        end_time = time.time()
                        print("\nTime: "+str(round((end_time-start_time),4)) + " [secs]")
                        backTracking(goalNode,startNode,closed_list,map)
                        return True    
                        
        else:
            print("\n No path found between the start and goal explored_nodes") 
            return False


def backTracking(goalNode, startNode, closed_list, map):
    video_writer = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('proj02_case2.avi',video_writer,1000,(width_map,height_map))

    final_parent = closed_list.get(tuple(goalNode))   
    cv2.line(map, tuple(goalNode), tuple(final_parent), (255,0,0), 1)

    parent_node_keys = closed_list.keys()
    for key in parent_node_keys:
        if key is not tuple(startNode):   
            map[key[1]][key[0]] = [255,255,255]
            cv2.circle(map,tuple(startNode),5,(0,255,0),-1)
            out.write(map)
        cv2.circle(map,tuple(goalNode),5,(0,255,0),-1)
        
        cv2.imshow("Path Generation",map)
        cv2.waitKey(1)

    while True:
        key = closed_list.get(tuple(final_parent))    
        
        cv2.line(map, tuple(key), tuple(final_parent), (255,0,0), 1)
        out.write(map)

        final_parent = key
        
        if key is startNode:
            break

    cv2.imshow("Path Generation", map)
    cv2.waitKey(0)


if __name__ == '__main__':

    map = generateMap(height_map, width_map)

    if(len(sys.argv) == 5):
        x_start = int(sys.argv[1])
        y_start =  height_map - int(sys.argv[2])   
        x_goal = int(sys.argv[3])
        y_goal = height_map - int(sys.argv[4])    
        if (checkInputFeasibility(x_start, y_start, x_goal, y_goal, map)):
            startNode = [x_start, y_start]
            goalNode = [x_goal, y_goal]
            
            res = findDijkstraPath(startNode, goalNode, map)

            cv2.destroyAllWindows()
    
    else: 
        print("\nInvalid number of arguments, try again!")
        exit()