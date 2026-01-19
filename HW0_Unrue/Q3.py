

def shortest_path_grid(grid, start, goal):
    '''
    Function that returns the length of the shortest path in a grid
    that has no obstacles. The length is simply the number of cells
    on the path including the 'start' and the 'goal'

    :param grid: list of lists (represents a square grid)
    :param start: tuple of start index
    :param goal: tuple of goal index
    :return: length of path
    '''
    n = len(grid)
    # YOUR CODE

    #print(str(goal[0]))
    #print((goal[1]))

    result = abs(start[0]-goal[0])+abs(start[1]-goal[1])+1

    print("now returning result:")
    print(result)
    return result

    






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
    


