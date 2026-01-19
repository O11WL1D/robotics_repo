


from enum import Enum
import math
import time
from collections import deque


def makenode():
      newnode=Node()
      return newnode




class Node:
    def __init__(self):
        
        self.self = [0, 0] 
        self.ch1 = [0, 0]
        self.ch2 = [0, 0]
        self.ch3 = [0, 0]
        self.ch4 = [0, 0]
        self.c1canmove=True
        self.c2canmove=True
        self.c3canmove=True
        self.c4canmove=True
     
        self.ch1node = None
        self.ch2node = None
        self.ch3node = None
        self.ch4node = None
        self.parent= None

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



    

def check_boundary(ch1,xmax,ymax):
      #print("CHECKING BOUNDARY")
      if (ch1[0] >= 0) and ch1[0] < (xmax + 1):
            if (ch1[1] >= 0) and ch1[1] < (ymax + 1):
                  return True
            else:
                  return False
      else:
            return False






def shortest_path_grid(grid, start, goal):


      #xmax=3
      #ymax=3
  
      ymax = len(grid) - 1          # last valid row index
      xmax = len(grid[0]) - 1       # last valid column index



      nodestack=[]

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
      head = Node()

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
     
    
      PARENTNODE=Node()
      subtractionamt=0
      while(True):


                                  
            blockedcounter+=1
            if(blockedcounter==50):
                  print("!!!!!!!!!!!!ROBOT IS BLOCKED!!")
                  return -1

           
            #time.sleep(1)
            print("---------------------------------------NEW CYCLE!")
            #grid[currentnode.self[1]][currentnode.self[0]]=2
            print("TEST")
            for x in grid:
                  print(x)


            #find neighboring nodes.
            currentnode.ch1=findtop(currentnode.self)
            currentnode.ch2=findleft(currentnode.self)
            currentnode.ch3=findbottom(currentnode.self)
            currentnode.ch4=findright(currentnode.self)

            #check if neighboring nodes are blocked by 
            #boundary violations.
            currentnode.c1canmove=check_boundary(currentnode.ch1,xmax,ymax)
            currentnode.c2canmove=check_boundary(currentnode.ch2,xmax,ymax)
            currentnode.c3canmove=check_boundary(currentnode.ch3,xmax,ymax)
            currentnode.c4canmove=check_boundary(currentnode.ch4,xmax,ymax)



            #prevent creation of nodes if blocked by 
            #obstacles, or if node belongs to parent node. 
                        # ch1
            if 0 <= currentnode.ch1[0] <= xmax and 0 <= currentnode.ch1[1] <= ymax:
                  if grid[currentnode.ch1[1]][currentnode.ch1[0]] in (1, 2):
                        currentnode.c1canmove=False
                        
            # ch2
            if 0 <= currentnode.ch2[0] <= xmax and 0 <= currentnode.ch2[1] <= ymax:
                  if grid[currentnode.ch2[1]][currentnode.ch2[0]] in (1, 2):
                        currentnode.c2canmove=False
            # ch3
            if 0 <= currentnode.ch3[0] <= xmax and 0 <= currentnode.ch3[1] <= ymax:
                  if grid[currentnode.ch3[1]][currentnode.ch3[0]] in (1, 2):
                        currentnode.c3canmove=False
            # ch4
            if 0 <= currentnode.ch4[0] <= xmax and 0 <= currentnode.ch4[1] <= ymax:
                  if grid[currentnode.ch4[1]][currentnode.ch4[0]] in (1, 2):
                        currentnode.c4canmove=False
 

            print("CURRENT COORDS")
            print(currentnode.self)

            #print("NODES!")
            #print(currentnode.ch1)
            #print(currentnode.ch2)
            #print(currentnode.ch3)
            #print(currentnode.ch4)


            #end condition which terminates search and counts path length. 
            if(currentnode.self[0]==goal[1] and currentnode.self[1]==goal[0]):
                  print("!!!!!!!!!!!!!!!MAZE COMPLETE! RETURNING!")
                  print("NOW TRACING BACK PATH!")

                  movementcounter=0
                  while(True):

                        if(currentnode==None):
                              break
                        print("------------------NEW TRACEBACK CYCLE!")
                        print("CURRENT COORDS")
                        grid[currentnode.self[1]][currentnode.self[0]]=3
                        movementcounter+=1
                        for x in grid:
                              print(x)
                        
                        print(currentnode.self)
                        currentnode=currentnode.parent

                  print("SHORTEST PATH LENGTH!"+str(movementcounter))
                  return movementcounter


                                    





            print("NOW FINDING NEW NODE TO EVALUATE")


            #add nodes to que if eligible. 
            if(not(currentnode.c1canmove==False)):
                   currentnode.ch1node=makenode()
                   currentnode.ch1node.self=currentnode.ch1
                   nodestack.append(currentnode.ch1node)
                   grid[currentnode.ch1[1]][currentnode.ch1[0]]=2
                   currentnode.ch1node.parent=currentnode
                   
    

            if not(currentnode.c2canmove==False):
                  currentnode.ch2node=makenode()
                  currentnode.ch2node.self=currentnode.ch2
                  nodestack.append(currentnode.ch2node)
                  grid[currentnode.ch2[1]][currentnode.ch2[0]]=2
                  currentnode.ch2node.parent=currentnode


            if not(currentnode.c3canmove==False):
                  currentnode.ch3node=makenode()
                  currentnode.ch3node.self=currentnode.ch3
                  nodestack.append(currentnode.ch3node)
                  grid[currentnode.ch3[1]][currentnode.ch3[0]]=2
                  currentnode.ch3node.parent=currentnode
                  
            if not(currentnode.c4canmove==False):
                  
                  currentnode.ch4node=makenode()
                  currentnode.ch4node.self=currentnode.ch4
                  nodestack.append(currentnode.ch4node)
                  grid[currentnode.ch4[1]][currentnode.ch4[0]]=2
                  currentnode.ch4node.parent=currentnode



            #output node stack
            # print("############CURRENT NODE STACK!")
            
            #for i in nodestack:
            #      print(i.self)
            #print("OKAY")
            

            #pop last item and assign to currentnode:
            if(not (len(nodestack) == 0)):
                 currentnode=nodestack.pop(0)
            






        





if __name__ == "__main__":
  

    grid = [[0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]]
    


    start, goal = (0, 0), (3, 2)
    assert shortest_path_grid(grid, start, goal) == 6

    grid = [[0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0]]

    

    start, goal = (1, 1), (3, 2)
    assert shortest_path_grid(grid, start, goal) == 4
    

    grid = [[0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0]]

    

    start, goal = (1, 2), (2, 2)
    assert shortest_path_grid(grid, start, goal) == 2
    

    grid = [
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
]
    start, goal = (4, 4), (0, 0)
    assert shortest_path_grid(grid, start, goal) == 9



