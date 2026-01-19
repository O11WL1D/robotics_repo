
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


def makenode():
      newnode=Node()
      return newnode


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



            print("---------------------------------------NEW CYCLE!")




            movementcounter+=1
            grid[currentnode.self[1]][currentnode.self[0]]=2

            
            for x in grid:
                  print(x)
            

           # time.sleep(1)

            
            #time.sleep(0.5) 

            
            currentnode.ch1=findtop(currentnode.self)
            currentnode.ch2=findleft(currentnode.self)
            currentnode.ch3=findbottom(currentnode.self)
            currentnode.ch4=findright(currentnode.self)
         
            #currentnode.ch1=check_boundary(currentnode.ch1)
            #currentnode.ch2=check_boundary(currentnode.ch2)
            #currentnode.ch3=check_boundary(currentnode.ch3)
            #currentnode.ch4=check_boundary(currentnode.ch4)

            currentnode.c1canmove=check_boundary(currentnode.ch1)
            currentnode.c2canmove=check_boundary(currentnode.ch2)
            currentnode.c3canmove=check_boundary(currentnode.ch3)
            currentnode.c4canmove=check_boundary(currentnode.ch4)
           
            print("NODES!")
            print(currentnode.ch1)
            print(currentnode.ch2)
            print(currentnode.ch3)
            print(currentnode.ch4)

   



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

 









            currentnode.parent=currentnode
            

            print("PARENT COORDS")
            print(currentnode.parent.self)

            print("PARENT")
            print(currentnode.parent)
            print("current")
            print(currentnode)


            if(not(currentnode.c1canmove==False)):
                #  currentnode.self=currentnode.ch1
            
                   currentnode.ch1node=makenode()
                   currentnode.ch1node.self=currentnode.ch1
                   
                   PARENTNODE=currentnode
                   currentnode=currentnode.ch1node
                   currentnode.parent=PARENTNODE

            else:
               
                  if not(currentnode.c2canmove==False):
                  #  currentnode.self = currentnode.ch2
                        currentnode.ch2node=makenode()
                        currentnode.ch2node.self=currentnode.ch2
                       
                       
                        PARENTNODE=currentnode
                        currentnode=currentnode.ch2node
                        currentnode.parent=PARENTNODE
                  else:


                        if not(currentnode.c3canmove==False):
                        # currentnode.self = currentnode.ch3
                              currentnode.ch3node=makenode()
                              currentnode.ch3node.self=currentnode.ch3
                            

                              PARENTNODE=currentnode
                            
                              currentnode=currentnode.ch3node
                              currentnode.parent=PARENTNODE


                        else:
                                 

                                 if not(currentnode.c4canmove==False):
                                          
                                          currentnode.ch4node=makenode()
                                          currentnode.ch4node.self=currentnode.ch4
                                          PARENTNODE=currentnode
                                          
                                          currentnode=currentnode.ch4node
                                          currentnode.parent=PARENTNODE

                                          #currentnode.self = currentnode.ch4
 



            print("CURRENT COORDS")
            print(currentnode.self)


            if(movementcounter==20):
                  print("!!!!!!!!!!!!!!!ROBOT STUCK!")
                  return -1

            if(currentnode.self[0]==goal[1] and currentnode.self[1]==goal[0]):
                  print("!!!!!!!!!!!!!!!MAZE COMPLETE! RETURNING!")
                  print("SHORTEST PATH LENGTH: " + str(movementcounter+1))
                  return movementcounter+1-subtractionamt






            currentnode.ch1=findtop(currentnode.self)
            currentnode.ch2=findleft(currentnode.self)
            currentnode.ch3=findbottom(currentnode.self)
            currentnode.ch4=findright(currentnode.self)

            #currentnode.ch1=check_boundary(currentnode.ch1)
            #currentnode.ch2=check_boundary(currentnode.ch2)
            #currentnode.ch3=check_boundary(currentnode.ch3)
            #currentnode.ch4=check_boundary(currentnode.ch4)

            currentnode.c1canmove=check_boundary(currentnode.ch1)
            currentnode.c2canmove=check_boundary(currentnode.ch2)
            currentnode.c3canmove=check_boundary(currentnode.ch3)
            currentnode.c4canmove=check_boundary(currentnode.ch4)



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




            if((not currentnode.c1canmove) and (not currentnode.c2canmove) and (not currentnode.c3canmove) and (not currentnode.c4canmove) ):
                  

               
                  print("!!!!!!!!!!!!!!!!!!!!!!STUCK!")
                  grid[currentnode.self[1]][currentnode.self[0]]=2
                  currentnode=currentnode.parent
                  subtractionamt+=1


              







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
                  return True
            else:
                  return False
      else:
            return False





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
        self.parentxy =[0,0]




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
    

    
    grid = [[0, 0, 0, 1],
            [1, 0, 0, 0],
            [1, 0, 1, 0],
            [1, 1, 0, 0]]
    start, goal = (3, 2), (2, 1)
    #print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 9



    grid = [[0, 0, 0, 0],
            [0, 1, 1, 0],
            [1, 1, 0, 0],
            [1, 0, 0, 1]]
    start, goal = (3, 1), (1, 0)
    #print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 10


    grid = [[0, 0, 0, 0],
            [1, 0, 1, 0],
            [1, 0, 1, 0],
            [1, 0, 1, 0]]
    
    start, goal = (3, 3), (0, 0)
    #print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 7
    
    grid = [[0, 0, 0, 1],
            [1, 0, 0, 0],
            [1, 0, 1, 0],
            [1, 1, 0, 0]]
    start, goal = (3, 2), (0, 0)
    #print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 8



    grid = [[0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]]
    start, goal = (1, 1), (0, 0)
    #print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 3




 


    grid = [[0, 1],
            [1, 0]]
    start, goal = (0, 0), (1, 1)
    #print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == -1



