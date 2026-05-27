import numpy as np


def get_neighbors(pos, grid_map, rows, cols):
    '''
    Check if neighbors are obstacles
    '''
    dirs = [(-1, 0), (1, 0), (0, -1), (0, 1), 
            (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    neighbors = []
    distances = []

    for d in dirs:
        nr = pos[0] + d[0]
        nc = pos[1] + d[1]

        # if the cell is in range and free
        if 0 <= nr < rows and 0 <= nc < cols:
            if grid_map[nr, nc] == 0:
                neighbors.append((nr, nc))
                
                # Diagonal movement costs more (sqrt(1^2 + 1^2) ≈ 1.414)
                if d[0] != 0 and d[1] != 0:
                    distances.append(1.414)
                else:
                    distances.append(1.0)
                
    return neighbors, distances


def heuristic(p_start, p_end):
    '''Compute Manhattan distance'''
    return abs(p_start[0] - p_end[0]) + abs(p_start[1] - p_end[1])


def reconstruct_path(came_from, current, start, rows):
    '''Invert path'''
    
    path = [[current[0], current[1]]]
    
    while current != start:
        current = came_from[current]
        path.insert(0, [current[0], current[1]])
        
    return path


def a_star(grid_map, start, goal):
    '''A* algorithm'''
    rows, cols = grid_map.shape

    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))

    # g - actual cost
    g_score = np.full((rows, cols), np.inf)
    # f - heuristic
    f_score = np.full((rows, cols), np.inf)
    
    g_score[start[0], start[1]] = 0
    f_score[start[0], start[1]] = heuristic(start, goal)
    
    open_set = np.zeros((rows, cols), dtype=bool)
    open_set[start[0], start[1]] = True
    
    came_from = {}
    
    while np.any(open_set):
        f_score_open = np.where(open_set, f_score, np.inf)
   
        min_idx = np.unravel_index(np.argmin(f_score_open), f_score_open.shape)
        current = (int(min_idx[0]), int(min_idx[1]))
        
        # if goal point is reached return path
        if current == goal:
            return reconstruct_path(came_from, current, start, rows)
        
        # set of discoverd but not explored nodes
        open_set[current[0], current[1]] = False
        
        neighbors, distances = get_neighbors(current, grid_map, rows, cols)
        
        for i, neighbor in enumerate(neighbors):
            nr, nc = neighbor

            # considering a node
            tentative_g = g_score[current[0], current[1]] + distances[i]
            
            # check if it's better node
            if tentative_g < g_score[nr, nc]:
                came_from[(int(nr), int(nc))] = current
                g_score[nr, nc] = tentative_g
                f_score[nr, nc] = tentative_g + heuristic(neighbor, goal)
                
                if not open_set[nr, nc]:
                    open_set[nr, nc] = True
                    
    print('Path does not exist.')
    return []