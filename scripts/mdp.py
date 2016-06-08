
def MDP(grid,height,width,config):
    # k = 0, initialize
    for x in range(0,height):
        for y in range(0,width):
            sqre = grid[x][y]
            if sqre.goal == 1:
                sqre.reward = config['reward_for_reaching_goal']
            elif sqre.wall == 1:
                sqre.reward = config['reward_for_hitting_wall']
            elif sqre.pit == 1:
                sqre.reward = config['reward_for_falling_in_pit']
            else:
                sqre.reward = 0

    # new grid to keep track of next iteration rewards
    newGrid = []
    for i in range(0,height):
        newGrid.append([]) 
    for x in range(0,height):
        for y in range(0,width):
            newGrid[x].append(0)
            
    # iterate
    k = 1
    moves = config['move_list']
    probUp = 0.0
    probDown = 0.0
    probLeft = 0.0
    probRight = 0.0
    rewardUp = 0.0
    rewardDown = 0.0
    rewardLeft = 0.0
    rewardRight = 0.0
    while k < config['max_iterations']:
        for x in range(0,height):
            for y in range(0,width):
                
                # calculate result of action for every direction 
                rewardUp = "Wall"
                if x-1 >= 0:
                    if grid[x-1][y].goal == 1:
                        rewardUp = "Goal"
                    elif grid[x-1][y].pit == 1:
                        rewardUp = "Pit"
                    elif grid[x-1][y].wall != 1:
                        rewardUp = "Spot"

                rewardDown = "Wall"
                if x+1 < height:
                    if grid[x+1][y].goal == 1:
                        rewardDown = "Goal"
                    elif grid[x+1][y].pit == 1:
                        rewardDown = "Pit"
                    elif grid[x+1][y].wall != 1:
                        rewardDown = "Spot"

                rewardLeft = "Wall"
                if y-1 >= 0:
                    if grid[x][y-1].goal == 1:
                        rewardLeft = "Goal"
                    elif grid[x][y-1].pit == 1:
                        rewardLeft = "Pit"
                    elif grid[x][y-1].wall != 1:
                        rewardLeft = "Spot"
                
                rewardRight = "Wall"
                if y+1 < width:
                    if grid[x][y+1].goal == 1:
                        rewardRight = "Goal"
                    elif grid[x][y+1].pit == 1:
                        rewardRight = "Pit"
                    elif grid[x][y+1].wall != 1:
                        rewardRight = "Spot"            
                
                maxReward = -1000
                firstCalc = 1
                # compare all directions
                for i in range(0,len(moves)):
                    moveX = moves[i][0]
                    moveY = moves[i][1]
                    # North
                    if moveX == -1 and moveY == 0:
                        probUp = config['prob_move_forward']
                        probDown = config['prob_move_backward']
                        probLeft = config['prob_move_left']
                        probRight = config['prob_move_right']
                    # South
                    elif moveX == 1 and moveY == 0:
                        probUp = config['prob_move_backward']
                        probDown = config['prob_move_forward']
                        probLeft = config['prob_move_right']
                        probRight = config['prob_move_left']
                    # West
                    elif moveX == 0 and moveY == -1:
                        probUp = config['prob_move_right']
                        probDown = config['prob_move_left']
                        probLeft = config['prob_move_forward']
                        probRight = config['prob_move_backward']
                    # East
                    else:
                        probUp = config['prob_move_left']
                        probDown = config['prob_move_right']
                        probLeft = config['prob_move_backward']
                        probRight = config['prob_move_forward']

                    rewardTot = 0.0
                    upReward = 0.0
                    downReward = 0.0
                    leftReward = 0.0
                    rightReward = 0.0

                    if grid[x][y].wall == 1:
                        maxReward = config['reward_for_hitting_wall']
                    elif grid[x][y].pit == 1:
                        maxReward = config['reward_for_falling_in_pit']
                    elif grid[x][y].goal == 1:
                        maxReward = config['reward_for_reaching_goal']
                    else:
                        # calculate reward for that direction
                        if rewardUp == "Wall":
                            upReward = config['reward_for_hitting_wall'] + config['discount_factor'] * grid[x][y].reward
                        elif rewardUp == "Pit":
                            upReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_falling_in_pit']
                        elif rewardUp == "Goal":
                            upReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_reaching_goal']
                        else:
                            upReward = config['reward_for_each_step'] + config['discount_factor'] * grid[x-1][y].reward

                        if rewardDown == "Wall":
                            downReward = config['reward_for_hitting_wall'] + config['discount_factor'] * grid[x][y].reward
                        elif rewardDown == "Pit":
                            downReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_falling_in_pit']
                        elif rewardDown == "Goal":
                            downReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_reaching_goal']
                        else:
                            downReward = config['reward_for_each_step'] + config['discount_factor'] * grid[x+1][y].reward

                        if rewardLeft == "Wall":
                            leftReward = config['reward_for_hitting_wall'] + config['discount_factor'] * grid[x][y].reward
                        elif rewardLeft == "Pit":
                            leftReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_falling_in_pit']
                        elif rewardLeft == "Goal":
                            leftReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_reaching_goal']
                        else:
                            leftReward = config['reward_for_each_step'] + config['discount_factor'] * grid[x][y-1].reward

                        if rewardRight == "Wall":
                            rightReward = config['reward_for_hitting_wall'] + config['discount_factor'] * grid[x][y].reward
                        elif rewardRight == "Pit":
                            rightReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_falling_in_pit']
                        elif rewardRight == "Goal":
                            rightReward = config['reward_for_each_step'] + config['discount_factor'] * config['reward_for_reaching_goal']
                        else:
                            rightReward = config['reward_for_each_step'] + config['discount_factor'] * grid[x][y+1].reward

                        rewardTot = probUp * upReward + probDown * downReward + probLeft * leftReward + probRight * rightReward
                        if i == 0:
                            maxReward = rewardTot
                            if moveX == -1 and moveY == 0:
                                grid[x][y].policy = "N"
                            elif moveX == 1 and moveY == 0:
                                grid[x][y].policy = "S"
                            elif moveX == 0 and moveY == -1:
                                grid[x][y].policy = "W"
                            else:
                                grid[x][y].policy = "E"
                        elif rewardTot > maxReward:
                            maxReward = rewardTot
                            if moveX == -1 and moveY == 0:
                                grid[x][y].policy = "N"
                            elif moveX == 1 and moveY == 0:
                                grid[x][y].policy = "S"
                            elif moveX == 0 and moveY == -1:
                                grid[x][y].policy = "W"
                            else:
                                grid[x][y].policy = "E"

                newGrid[x][y] = maxReward
        
        for x in range(0,height):
            for y in range(0,width):
                grid[x][y].reward = newGrid[x][y]
        
        k += 1
    
    policyList = []
    for x in range(0,height):
        for y in range(0,width):
            policyList.append(grid[x][y].policy)

    return policyList



