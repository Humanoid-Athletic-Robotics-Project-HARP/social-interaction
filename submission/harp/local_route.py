"""Small bounded geometric route planner for the native test driver.

The submission image already uses Nav2 for this responsibility. Inputs here are
measured object tracks, never simulator truth; MPC checks the resulting velocity.
"""
import heapq
import math
import numpy as np


def route_direction(start, goal, tracks, resolution=.2):
    start, goal = np.asarray(start,float), np.asarray(goal,float)
    lower = np.minimum(start,goal)-[2.,3.]
    upper = np.maximum(start,goal)+[2.,3.]
    shape = np.ceil((upper-lower)/resolution).astype(int)+1
    if max(shape) > 101:
        raise ValueError('native local route exceeds bounded map')
    coords = np.stack(np.meshgrid(np.arange(shape[0]),np.arange(shape[1]),indexing='ij'),axis=-1)
    world = lower+coords*resolution
    blocked = np.zeros(tuple(shape),bool)
    risk = np.zeros(tuple(shape),float)
    arrival = np.minimum(4.,np.linalg.norm(world-start,axis=-1)/.35)
    for t in tracks:
        radius = .4+.08+t.radius+min(t.uncertainty,.15)
        vx,vy = getattr(t,'vx',0.),getattr(t,'vy',0.)
        if math.hypot(vx,vy) < .12:
            blocked |= np.linalg.norm(world-[t.x,t.y],axis=-1) < radius
        else:
            # Moving people are not permanent walls. Bias the route away from
            # their estimated arrival-time positions; MPC retains hard checks.
            future = np.stack([t.x+vx*arrival,t.y+vy*arrival],axis=-1)
            separation = np.linalg.norm(world-future,axis=-1)
            risk += .8*np.exp(-.5*(separation/max(radius,.2))**2)
    begin = tuple(np.rint((start-lower)/resolution).astype(int))
    end = tuple(np.rint((goal-lower)/resolution).astype(int))
    if blocked[end]:
        # The final goal may currently be occupied. Plan toward a reachable
        # staging point; never replace route planning with a straight-line drive.
        available = np.argwhere(~blocked)
        if not len(available):
            return np.zeros(2)
        positions = lower+available*resolution
        score = np.linalg.norm(positions-goal,axis=1)+.001*np.linalg.norm(positions-start,axis=1)
        end = tuple(available[np.argmin(score)])
    queue, cost, parent = [(0.,begin)], {begin:0.}, {}
    while queue:
        _, cell = heapq.heappop(queue)
        if cell == end:
            path = [cell]
            while path[-1] != begin:
                path.append(parent[path[-1]])
            path.reverse()
            target = lower+np.array(path[min(4,len(path)-1)])*resolution
            direction = target-start
            return direction/max(np.linalg.norm(direction),1e-9)
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            nxt = (cell[0]+dx,cell[1]+dy)
            if not (0<=nxt[0]<shape[0] and 0<=nxt[1]<shape[1]) or blocked[nxt]:
                continue
            if dx and dy and (blocked[cell[0]+dx,cell[1]] or blocked[cell[0],cell[1]+dy]):
                continue
            value = cost[cell]+math.hypot(dx,dy)*(1+risk[nxt])
            if value < cost.get(nxt,math.inf):
                cost[nxt],parent[nxt] = value,cell
                heapq.heappush(queue,(value+math.dist(nxt,end),nxt))
    direct = goal-start
    return direct/max(np.linalg.norm(direct),1e-9)  # MPC may find an immediate escape.
