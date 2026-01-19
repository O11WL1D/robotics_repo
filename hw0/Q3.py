


from enum import Enum
import math
import time


class TURNDIRECTION(Enum):
        LEFT = 1
        RIGHT = 2
        FORWARD = 3
        BACK = 4
        
class DIRECTION(Enum):
        LEFT = 1
        RIGHT = 2
        TOP = 3
        DOWN = 4

class STATE(Enum):
        SEARCHERROR = 1
        MOVING=2



def shortest_path_grid(grid, start, goal):
      '''
      Function that returns the length of t
      he shortest path in a grid
      that HAS obstacles represented by 1s. The length is simply the number
      of cells on the path including the 'start' and the 'goal'
      :param grid: list of lists (represents a square grid where 0 represents free space and 1s obstacles)
      :param start: tuple of start index
      :param goal: tuple of goal index
      :return: length of path
      '''


      currentx=start[1]
      currenty=start[0]


      #print("printing head")
      head = node()


      head.self=[currentx,currenty]
      print(head.self)
      currentnode=head

      movementcounter=0
      blockedcounter=0
      #the blocked counter will
      #count the number of obstacle 
      #detection errors there are. 
      #if two obstacles are detected, 
      #the function will return -1. 
      while(True):
            print("---------------------------------------NEW CYCLE!")
            movementcounter+=1
     
            if(movementcounter==20):
                  print("!!!!!!!!!!!!!!!ROBOT STUCK!")
                  return -1
            if(currentnode.self[0]==goal[1] and currentnode.self[1]==goal[0]):
                  print("!!!!!!!!!!!!!!!MAZE COMPLETE! RETURNING!")
                  print("SHORTEST PATH LENGTH: " + str(movementcounter))
                  return movementcounter



            
            currentnode.ch1=findbottom(currentnode.self)
            currentnode.ch2=findright(currentnode.self)
            currentnode.ch3=findtop(currentnode.self)
            currentnode.ch4=findleft(currentnode.self)
         
            currentnode.ch1=check_boundary(currentnode.ch1)
            currentnode.ch2=check_boundary(currentnode.ch2)
            currentnode.ch3=check_boundary(currentnode.ch3)
            currentnode.ch4=check_boundary(currentnode.ch4)
            print("NODES!")
            print(currentnode.self)
            print(currentnode.ch1)
            print(currentnode.ch2)
            print(currentnode.ch3)
            print(currentnode.ch4)

            # ch1
            if (currentnode.ch1[0] >= 0) and currentnode.ch1[0] < (xmax + 1):
                  if (currentnode.ch1[1] >= 0) and currentnode.ch1[1] < (ymax + 1):
                        if grid[currentnode.ch1[1]][currentnode.ch1[0]] == 1:
                              currentnode.ch1 = [-1, -1]
                        else:
                              grid[currentnode.ch1[1]][currentnode.ch1[0]] = 2
        
            # ch2
            if (currentnode.ch2[0] >= 0) and currentnode.ch2[0] < (xmax + 1):
                  if (currentnode.ch2[1] >= 0) and currentnode.ch2[1] < (ymax + 1):
                        if grid[currentnode.ch2[1]][currentnode.ch2[0]] == 1:
                              currentnode.ch2 = [-1, -1]
                        else:
                              grid[currentnode.ch2[1]][currentnode.ch2[0]] = 2
   
            # ch3
            if (currentnode.ch3[0] >= 0) and currentnode.ch3[0] < (xmax + 1):
                  if (currentnode.ch3[1] >= 0) and currentnode.ch3[1] < (ymax + 1):
                        if grid[currentnode.ch3[1]][currentnode.ch3[0]] == 1:
                              currentnode.ch3 = [-1, -1]
                        else:
                              grid[currentnode.ch3[1]][currentnode.ch3[0]] = 2

            # ch4
            if (currentnode.ch4[0] >= 0) and currentnode.ch4[0] < (xmax + 1):
                  if (currentnode.ch4[1] >= 0) and currentnode.ch4[1] < (ymax + 1):
                        if grid[currentnode.ch4[1]][currentnode.ch4[0]] == 1:
                              currentnode.ch4 = [-1, -1]
                        else:
                              grid[currentnode.ch4[1]][currentnode.ch4[0]] = 2
            # self
            if (currentnode.self[0] >= 0) and currentnode.self[0] < (xmax + 1):
                  if (currentnode.self[1] >= 0) and currentnode.self[1] < (ymax + 1):
                        if grid[currentnode.self[1]][currentnode.self[0]] == 1:
                              currentnode.self = [-1, -1]
                        else:
                              grid[currentnode.self[1]][currentnode.self[0]] = 2
      

            for x in grid:
                  print(x)
            
          #  newnode = node()
                  
            
            
            if not (currentnode.ch4 == [-1, -1]):
                  currentnode.self = currentnode.ch4
            
            
            if not (currentnode.ch3 == [-1, -1]):
                  currentnode.self = currentnode.ch3
            if not (currentnode.ch2 == [-1, -1]):
                   currentnode.self = currentnode.ch2



            if(not(currentnode.ch1==[-1,-1])):
                  currentnode.self=currentnode.ch1
            


            print("NODES!")
            print(currentnode.self)
            print(currentnode.ch1)
            print(currentnode.ch2)
            print(currentnode.ch3)
            print(currentnode.ch4)

xmax=3
ymax=3
    



def check_boundary(ch1):
      print("CHECKING BOUNDARY")
      if (ch1[0] >= 0) and ch1[0] < (xmax + 1):
            if (ch1[1] >= 0) and ch1[1] < (ymax + 1):
                  return ch1
            else:
                  return [-1,-1]
      else:
            return [-1,-1]



class node:
  self=[0,0]
  ch1=[0,0]
  ch2=[0,0]
  ch3=[0,0]
  ch4=[0,0]
  

def findleft(coord):
      px=coord[0]
      py=coord[1]
      return([px-1,py])
def findright(coord):
      px=coord[0]
      py=coord[1]
      return([px+1,py])

def findtop(coord):
      px=coord[0]
      py=coord[1]
      return([px,py-1])

def findbottom(coord):
      px=coord[0]
      py=coord[1]
      return([px,py+1])




def findangle(x,y):
      return math.degrees(math.atan2(x,y))




def turndirection(curturndirection, curdir):
    #option specifies what direction to turn. 
    #as detailed in the enum below. 
    #the resulting direction to turn is 
    #returned by this function.
    newdirx=curdir[0]
    newdiry=curdir[1]

    if(curturndirection==TURNDIRECTION.FORWARD):
          return curdir
    
    if(curturndirection==TURNDIRECTION.RIGHT):
          
          curangle=findangle(newdirx,newdiry)
          #print("CURRENT ANGLE: "+str(curangle))
          newx=round(math.sin(math.radians(curangle-90)),0)
          newy=round(math.cos(math.radians(curangle-90)),0)
          #print("NEW X: "+ str(newx) + "NEW y: "+ str(newy))
          return [newx,newy]
    if(curturndirection==TURNDIRECTION.LEFT):
          
          curangle=findangle(newdirx,newdiry)
          #print("CURRENT ANGLE: "+str(curangle))
          newx=round(math.sin(math.radians(curangle+90)),0)
          newy=round(math.cos(math.radians(curangle+90)),0)
          #print("NEW X: "+ str(newx) + "NEW y: "+ str(newy))
          return [newx,newy]
          
          
 
    
            

        





if __name__ == "__main__":
    grid = [[0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]]
    start, goal = (0, 0), (3, 2)
    print(shortest_path_grid(grid, start, goal))
    ##print("test")
    assert shortest_path_grid(grid, start, goal) == 6
    start, goal = (1, 1), (3, 2)
    assert shortest_path_grid(grid, start, goal) == 4
    
    start, goal = (1, 2), (2, 2)
    assert shortest_path_grid(grid, start, goal) == 2
    


