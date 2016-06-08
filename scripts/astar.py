import Queue

def calcMHDist(grid,height,width,goal):
    for x in range(0,height):
        for y in range(0,width):
            grid[x][y].MHDist = abs(x-goal[0]) + abs(y-goal[1])
            grid[x][y].steps = 0
            grid[x][y].visited = -1
            grid[x][y].prev = 0


def AStar(grid,height,width,start,numOfWalls,moves):
    startSqre = grid[start[0]][start[1]]
    startSqre.visited = 1
    visited = 1
    PQ = Queue.PriorityQueue(height * width - numOfWalls)
    PQ.put((startSqre.MHDist,startSqre))
    done = 0
    while(visited < height * width - numOfWalls and done == 0):
        sqre = PQ.get()[1]
        # check all possible moves
        for i in range(0,len(moves)):
            nextX = moves[i][0] + sqre.x
            nextY = moves[i][1] + sqre.y
            # within map range
            if nextX >= 0 and nextX < height and nextY >= 0 and nextY < width:
                nextSqre = grid[nextX][nextY]
                # check for goal
                if nextSqre.goal == 1:   
                    done = 1
                    # create path list (list of lists) to return
                    pathList = []
                    pathLen = sqre.steps + 2
                    for i in range(0,pathLen):
                        pathList.append(0)
                    # fill path list
                    pathList[pathLen-1] = [nextSqre.x,nextSqre.y]
                    pathList[pathLen-2] = [sqre.x,sqre.y]
                    pIndex = 3
                    while sqre.prev != 0 and pIndex <= pathLen:
                        sqre = sqre.prev
                        pathList[pathLen-pIndex] = [sqre.x,sqre.y]
                        pIndex += 1
                    return pathList
                elif nextSqre.wall != 1 and nextSqre.visited != 1:
                    nextSqre.visited = 1
                    nextSqre.prev = sqre
                    nextSqre.steps = sqre.steps + 1
                    priority = nextSqre.steps + nextSqre.MHDist
                    # give pits really low priority
                    if nextSqre.pit == 1:
                        priority = height*width*4
                    PQ.put((priority, nextSqre))
                    visited += 1
