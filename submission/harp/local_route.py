"""Small bounded geometric route planner for the native test driver.

The submission image already uses Nav2 for this responsibility. Inputs here are
measured object tracks, never simulator truth; MPC checks the resulting velocity.
"""
import heapq
import math
import numpy as np

UNSEEN_COST = 3.   # extra cost per cell the LiDAR cannot see now (behind observed obstacles)


def visible_cells(world, start, tracks, reach=6.):
    """Cells within lidar range not hidden behind an observed obstacle.

    Unobserved space is not known to be free: a route through a doorway into a
    room the robot cannot see led into a dead end. This is the current view
    only, from the same measured tracks the route uses. Only fixed occluders
    count: a walking person's shadow moves on and is seen again a moment later.
    """
    offset = world-start
    distance = np.linalg.norm(offset, axis=-1)
    visible = distance <= reach
    unit = offset/np.maximum(distance, 1e-9)[..., None]
    for t in tracks:
        if math.hypot(getattr(t,'vx',0.), getattr(t,'vy',0.)) >= .12:
            continue
        center = np.array([t.x, t.y])-start
        gap = float(np.linalg.norm(center))
        if gap <= t.radius:
            continue
        along = unit @ center
        across = np.abs(unit[..., 0]*center[1]-unit[..., 1]*center[0])
        visible &= ~((across < t.radius) & (along > 0) & (distance > gap))
    return visible


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
    # The robot may already stand inside an inflated zone (near a wall); let the
    # search leave it rather than fall back to a straight line to the goal.
    blocked &= np.linalg.norm(world-start, axis=-1) > .45
    unseen = ~visible_cells(world, start, tracks)
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
            value = cost[cell]+math.hypot(dx,dy)*(1+risk[nxt]+UNSEEN_COST*unseen[nxt])
            if value < cost.get(nxt,math.inf):
                cost[nxt],parent[nxt] = value,cell
                heapq.heappush(queue,(value+math.dist(nxt,end),nxt))
    direct = goal-start
    return direct/max(np.linalg.norm(direct),1e-9)  # MPC may find an immediate escape.


def route_command(pose, goal, direction, speed):
    """Body-frame (vx, vy, wz) along the route direction, facing travel until near the goal."""
    x, y, yaw = pose
    distance = math.hypot(goal[0]-x, goal[1]-y)
    v = min(.9*speed, distance*.5)
    rx, ry = direction
    vx = v*(math.cos(yaw)*rx+math.sin(yaw)*ry)
    vy = v*(-math.sin(yaw)*rx+math.cos(yaw)*ry)
    heading = math.atan2(ry, rx) if distance > .8 else 0.
    error = math.atan2(math.sin(heading-yaw), math.cos(heading-yaw))
    return vx, vy, float(np.clip(1.2*error, -.5, .5))
