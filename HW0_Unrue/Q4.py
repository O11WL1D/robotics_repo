
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






    n = len(grid)
    # YOUR CODE


    xmax=3
    ymax=3
    #this is for boundary checking. 

    

    currentx=start[1]
    currenty=start[0]

    #this is the currently searched position.

    targetx=start[0]
    targety=start[1]

    #this is the checked grid position 
    #before robot moves into grid. 

    #positions that have already been explored will be marked with a 2
    #grid[currentx][currenty]=2



    tgridcontents=0



    #while(True):
     #   print("test")
    
    curturndirection=TURNDIRECTION.FORWARD
    robot_t_state=1

    currentdir=[1,0]


    #currentdir=turndirection(curturndirection,currentdir)

    robotstate=STATE.MOVING
    robot_t_state=1

    #currentdir=turndirection(curturndirection,currentdir)



    blockedcounter=0
    #the blocked counter will
    #count the number of obstacle 
    #detection errors there are. 
    #if two obstacles are detected, 
    #the function will return -1. 

    movementcounter=0
    #this is the movement counter
    #which increments every time 

    while(True):
      

      if(currentx==goal[0] and currenty==goal[1]):
            print("!!!!!!!!!!!!!!!MAZE COMPLETE! RETURNING!")
            print("SHORTEST PATH LENGTH: " + str(movementcounter+1))
            return movementcounter+1


      #time.sleep(1)
      print("---------------------------------------NEW CYCLE!")
      
      turnedxy=turndirection(curturndirection,currentdir)
      currentdir=turnedxy

      targetx=(int)(currentx+turnedxy[0])
      targety=(int)(currenty+turnedxy[1])

      
      print("CURRENT DIRECTION: "+ str(currentdir))

 
      print("CURRENT X "+ str(currentx)+" CURRENT Y "+ str(currenty))

      #print("CURRENT CELL CONTENTS : " + str(grid[currenty][currentx]) )

      print("TARGET X "+ str(targetx)+" TARGET Y "+ str(targety))



      #maze bounds error
      if((targetx>xmax) or (targety>ymax) or (targetx<0) or (targety<0) ):
           print("BOUNDS ERROR")
           robotstate=STATE.SEARCHERROR
           robot_t_state=robot_t_state+1
      else:
            robotstate=STATE.MOVING
            tgridcontents=grid[targety][targetx]
            #print("TARGET GRID CONTENTS: " + str(tgridcontents))
            


      #space inaccessable error
      if(tgridcontents==1):
            print("------------------------OBSTACLE FOUND IN TARGET SPACE!")
            robot_t_state=3
            robotstate=STATE.SEARCHERROR
            blockedcounter+=1
      else:
            blockedcounter=0




      if(blockedcounter==2):
            print("!!!!!!!!!!!!!!!!!!!robot is blocked")
            return -1

  



      if(robot_t_state==1):
            curturndirection=TURNDIRECTION.FORWARD
            print("GOING FORWARD!")
            robot_t_state=1



      
      if(robot_t_state==2):
            curturndirection=TURNDIRECTION.LEFT
            print("TURNING LEFT!")
            robot_t_state=1






     
        #special condition, obstacle avoidance. 
      if(robot_t_state==4):
                curturndirection=TURNDIRECTION.RIGHT
                print("OBSTACLE AVOID, TURNING RIGHT!")
                robot_t_state=1
                

   

 #special condition, obstacle avoidance. 
      if(robot_t_state==3):
            curturndirection=TURNDIRECTION.LEFT
            print("OBSTACLE AVOID, TURNING LEFT!")
            robot_t_state=4
            

    




      if(robotstate==STATE.MOVING):
            
            robot_t_state=1
            
            currentx=targetx
            currenty=targety
            movementcounter+=1
            #print("MOVED TO X: " + str(currentx)+"Y: " +str(currenty))
            #print("CURRENT CELL CONTENTS: " + str(grid[currenty,currentx]))
            




      


  #robot will first attempt to go right, then forwards, 
  #then left. 
   




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

    start, goal = (3, 2), (0, 0)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 8



    grid = [[0, 1],
            [1, 0]]
    start, goal = (0, 0), (1, 1)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == -1

    grid = [[0, 0, 0, 0],
            [1, 1, 1, 0],
            [1, 1, 0, 0],
            [1, 0, 0, 1]]

    start, goal = (3, 1), (0, 0)
    print(shortest_path_grid(grid, start, goal))
    assert shortest_path_grid(grid, start, goal) == 9


