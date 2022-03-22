
import numpy as np
import cv2
from math import *
import sys

#Clearance
c = 15
#Total clearance including robot dmensions
# Here we have defined clearance between obstacles and
#Rigid robot
##################################
#section 1 :
############ Define obstacles in map
#Define governing eqations for points inside the obstacle
#Color the obstacles as blue


def obstacle_circle(map):
  crow = 300+c
  ccol = 185+c
  rad = 40
  for row in range(400):
    for col in range(250):
      if (row - crow)**2+(col-ccol)**2 - rad**2 < 0:
        map[row][col] = (255, 0, 0)
  return map


def polygon(map):
   x1, y1 = 36+c, 185+c
   x2, y2 = 115+c, 210+c
   x3, y3 = 80+c, 180+c
   x4, y4 = 105+c, 100+c
   for row in range(400):
     for col in range(250):
      r1 = (col - y1) * (x2 - x1) - (row - x1) * (y2 - y1)
      r2 = (col - y2) * (x3 - x2) - (row - x2) * (y3 - y2)
      r3 = (col - y3) * (x4 - x3) - (row - x3) * (y4 - y3)
      r4 = (col - y4) * (x1 - x4) - (row - x4) * (y1 - y4)
      r5 = (col - y1) * (x3 - x1) - (row - x1) * (y3 - y1)
      if (r1 <= 0 and r2 <= 0 and r5 >= 0) or (r3 <= 0 and r4 <= 0 and r5 <= 0):
        map[row][col] = (255, 0, 0)
   return map


def hexagon(map):
   s = 40.4125
   x1, y1 = 200+c, 100+s+c
   x2, y2 = 235+c, 100+s/2+c
   x3, y3 = 235+c, 100-s/2+c
   x4, y4 = 200+c, 100-s+c
   x5, y5 = 165+c, 100-s/2+c
   x6, y6 = 165+c, 100+s/2+c
   for row in range(400):
     for col in range(250):
      r1 = (col - y1) * (x2 - x1) - (row - x1) * (y2 - y1)
      r2 = (col - y2) * (x3 - x2) - (row - x2) * (y3 - y2)
      r3 = (col - y3) * (x4 - x3) - (row - x3) * (y4 - y3)
      r4 = (col - y4) * (x5 - x4) - (row - x4) * (y5 - y4)
      r5 = (col - y5) * (x6 - x5) - (row - x5) * (y6 - y5)
      r6 = (col - y6) * (x1 - x6) - (row - x6) * (y1 - y6)
      if r1 <= 0 and r2 <= 0 and r3 <= 0 and r4 <= 0 and r5 <= 0 and r6 <= 0:
        map[row][col] = (255, 0, 0)
   return map

#checks if next node is safe


def check_safe(map, x, y):
    if x <= 385 and y <= 235 and x >= 15 and y >= 15 and (np.int0(map[x][y]) == (255, 255, 255)).all():
        return True
    else:
      return False
#check if goal is reached
def check_goal(map,current_node,goal_node,g_c):
  if eucledian(current_node, goal_node)<=g_c:# and current_node[2]==goal_node[2]: If we want desired orientation it 
  
    return True

def find_neighbors(dis,world, current_node):
    x = current_node[0]
    y = current_node[1]
    theta_absoulute=current_node[2]
    neighbor = []
    #c1 = dis
    
    for k in range(-2,3):
        #print(current_node)
        theta1=round((degrees(pi*(k/6))))
        theta2=round((theta1+theta_absoulute)%360)
        theta3=radians(theta2)
        

        x2 = round(x + dis*cos(theta3))
        y2 = round(y + dis*sin(theta3))
        
        
        
        if check_safe(world, x2, y2):

             neighbor.append([(x2, y2,theta2), dis])
    return neighbor #next neighbor
    
    


def resize_function(mat, angle):

    height, width = mat.shape[:2]
    image_center = (width/2, height/2)

    rotation_mat = cv2.getRotationMatrix2D(image_center, angle, 1.)

    abs_cos = abs(rotation_mat[0, 0])
    abs_sin = abs(rotation_mat[0, 1])

    # find the new width and height bounds
    bound_w = int(height * abs_sin + width * abs_cos)
    bound_h = int(height * abs_cos + width * abs_sin)

    # subtract old image center (bringing image back to origo) and adding the new image center coordinates
    rotation_mat[0, 2] += bound_w/2 - image_center[0]
    rotation_mat[1, 2] += bound_h/2 - image_center[1]

    # rotate image with the new bounds and translated rotation matrix
    rotated_mat = cv2.warpAffine(mat, rotation_mat, (bound_w, bound_h))
    return rotated_mat


def eucledian(current_node, goal_node):
    x_c = current_node[0]
    y_c = current_node[1]
    x_g = goal_node[0]
    y_g = goal_node[1]
    dx = (x_g-x_c)**2
    dy = (y_g-y_c)**2
    return sqrt(dx+dy)

#Main function which gives visited nodes and path
def Astar(world, start, goal,mapc,st):
   
    
    
    open_list = []  # list of open nodes
    
    closed_list = set()
    #closed_list=[]
    parents = dict()  # dictionary for storing parent nodes
    cost_to_come = dict()  # dictionary
    cost_to_come[start] = 0  # coordinates are the key. Coordinates are tuple
    cost = dict()

    cost_to_go = eucledian(start, goal)
    open_list.append([(start), cost_to_go])
    shortest_path = []
    path_found = False  # Flag for path
    cost[start] = cost_to_go
    while open_list:
        open_list.sort(key=lambda x: x[1])  # sort the list
        current_node = open_list.pop(0)[0]  # pop the first node
        #closed_list.append(current_node)
        closed_list.add(current_node)
        
        if check_goal(world, current_node, goal,st):
            goal=current_node
            path_found = True
            break
        neighbors = find_neighbors(st,world, current_node)
    
        

        for neighbor, stepcost in neighbors:
            
              
              if neighbor in closed_list:
                      continue
              mapc=cv2.line(mapc, ((current_node[0]),(250-current_node[1])),((neighbor[0]),(250-neighbor[1])), (255,255,0),1)      
              temp_c2c = cost_to_come.get(current_node)+stepcost
              temp_c2g = temp_c2c + eucledian(neighbor, goal)
              in_open_list = False
              for idx, element in enumerate(open_list):
                  if element[0] == neighbor:
                      in_open_list = True
                      break
              if in_open_list:
                   if temp_c2g < cost.get(neighbor):
                  
                        cost[neighbor] = temp_c2g
                        cost_to_come[neighbor] = temp_c2c
                        parents[neighbor] = current_node
                        open_list[idx] = [neighbor, temp_c2g ]
              else:
               
                        cost_to_come[neighbor] = temp_c2c
                        cost[neighbor] = temp_c2g
                        parents[neighbor] = current_node
                        open_list.append([neighbor, temp_c2g])
                     
              cv2.imshow("Visited", mapc)
              cv2.waitKey(1)
           
                
                     

    if not path_found:
        return shortest_path

    if path_found:
      node = goal
      shortest_path.append(goal)
      while node != start:
          shortest_path.append(node)

          node = parents[node]
    shortest_path = shortest_path[::-1]
    return shortest_path, closed_list,mapc


map = 255*np.ones((400, 250, 3))

map = obstacle_circle(map)
map = polygon(map)
map = hexagon(map)
map_ = map.copy()
#inputs
x_s = int(input("Enter x start coordinate greater than 15 and less 385 : "))
y_s = int(input("Enter y start coordinate  greater than 15 and less 235:"))
theta_in=int(input("Enter intial theta angle:"))

start = (x_s, y_s,theta_in)


if not check_safe(map, start[0], start[1]):
    sys.exit("start node inside object or out of limit. Please enter again")

x_g = int(input("Enter x goal coordinate greater than 15 and less 385 : "))
y_g = int(input("Enter y goal coordinate greater than 15 and less 235:"))
theta_goal=int(input("Enter goal theta angle:"))

goal = (x_g, y_g,theta_goal)


if not check_safe(map, goal[0], goal[1]):
    sys.exit("Goal node inside object or out of limit")
s_t = int(input("enter step cost from 0 to 10 : "))
mapc=resize_function(map, 90)
path, closed_list,am = (Astar(map_, start, goal,mapc,s_t))
print("Path found:", path)
print("-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-")
print("Exploring using set for storing closed nodes. ")
img = am.copy()
print("--------------------------")

print("Plotting the path")

for p in range(len(path)):
    
    if p > 0 :
      x1,y1,t1=path[p]
      x2,y2,t2=path[p-1]
      cv2.line(img,(x1,250-y1),(x2,250-y2),(0,0,0),1)
  
    cv2.imshow('map', img)
    cv2.waitKey(150)
cv2.imwrite("result.jpg", img)      
cv2.destroyAllWindows()