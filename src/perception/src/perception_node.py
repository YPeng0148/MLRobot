
"""

LIDAR to 2D grid map example

based on Erno Horvath, Csaba Hajdu and Atsushi Sakai's scripts
edited by Senior Design Robot Team

"""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

import math
from collections import deque

import numpy as np

rospy.init_node('perception_node', anonymous=True)
pub = rospy.Publisher('/map', OccupancyGrid, queue_size = None)

grid = OccupancyGrid()

EXTEND_AREA = 1.0

angles = []
distances = []
    
def bresenham(start, end):
    """
    Implementation of Bresenham's line drawing algorithm
    This is basically an algorith tham figures out which boxes
    in the grid a line passes through
    Bresenham's Line Algorithm
    Produces a np.array from start and end (original from roguebasin.com)
    >>> points1 = bresenham((4, 4), (6, 10))
    >>> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points


def calc_grid_map_config(ox, oy, xy_resolution):
    """
    Calculates the size, and the maximum distances according to the the
    measurement center
    """
    min_x = round(min(ox) - EXTEND_AREA / 2.0)
    min_y = round(min(oy) - EXTEND_AREA / 2.0)
    max_x = round(max(ox) + EXTEND_AREA / 2.0)
    max_y = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((max_x - min_x) / xy_resolution))
    yw = int(round((max_y - min_y) / xy_resolution))
    print("The grid map is ", xw, "x", yw, ".")
    return min_x, min_y, max_x, max_y, xw, yw


def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle


def init_flood_fill(center_point, obstacle_points, xy_points, min_coord,
                    xy_resolution):
    """
    center_point: center point
    obstacle_points: detected obstacles points (x,y)
    xy_points: (x,y) point pairs
    """
    center_x, center_y = center_point
    prev_ix, prev_iy = center_x - 1, center_y
    ox, oy = obstacle_points
    xw, yw = xy_points
    min_x, min_y = min_coord
    occupancy_map = (np.ones((xw, yw))) * 0.5
    for (x, y) in zip(ox, oy):
        # x coordinate of the the occupied area
        ix = int(round((x - min_x) / xy_resolution))
        # y coordinate of the the occupied area
        iy = int(round((y - min_y) / xy_resolution))
        free_area = bresenham((prev_ix, prev_iy), (ix, iy))
        for fa in free_area:
            occupancy_map[fa[0]][fa[1]] = 0  # free area 0.0
        prev_ix = ix
        prev_iy = iy
    return occupancy_map


def flood_fill(center_point, occupancy_map):
    """
    center_point: starting point (x,y) of fill
    occupancy_map: occupancy map generated from Bresenham ray-tracing
    """
    # Fill empty areas with queue method
    sx, sy = occupancy_map.shape
    fringe = deque()
    fringe.appendleft(center_point)
    while fringe:
        n = fringe.pop()
        nx, ny = n
        # West
        if nx > 0:
            if occupancy_map[nx - 1, ny] == 0.5:
                occupancy_map[nx - 1, ny] = 0.0
                fringe.appendleft((nx - 1, ny))
        # East
        if nx < sx - 1:
            if occupancy_map[nx + 1, ny] == 0.5:
                occupancy_map[nx + 1, ny] = 0.0
                fringe.appendleft((nx + 1, ny))
        # North
        if ny > 0:
            if occupancy_map[nx, ny - 1] == 0.5:
                occupancy_map[nx, ny - 1] = 0.0
                fringe.appendleft((nx, ny - 1))
        # South
        if ny < sy - 1:
            if occupancy_map[nx, ny + 1] == 0.5:
                occupancy_map[nx, ny + 1] = 0.0
                fringe.appendleft((nx, ny + 1))


def generate_ray_casting_grid_map(ox, oy, xy_resolution, breshen=True):
    """
    The breshen boolean tells if it's computed with bresenham ray casting
    (True) or with flood fill (False)
    """

    
    min_x, min_y, max_x, max_y, x_w, y_w = calc_grid_map_config(
        ox, oy, xy_resolution)
    # default 0.5 -- [[0.5 for i in range(y_w)] for i in range(x_w)]
    occupancy_map = np.ones((x_w, y_w)) / 2
    center_x = int( round(-min_x / xy_resolution) )  # center x coordinate of the grid map
    center_y = int( round(-min_y / xy_resolution) )  # center y coordinate of the grid map
    
    print("ox and oy:", center_x, center_y)

    # occupancy grid computed with bresenham ray casting
    if breshen:
        for (x, y) in zip(ox, oy):
            # x coordinate of the the occupied area
            ix = int(round((x - min_x) / xy_resolution))
            # y coordinate of the the occupied area
            iy = int(round((y - min_y) / xy_resolution))
            laser_beams = bresenham((center_x, center_y), (
                ix, iy))  # line form the lidar to the occupied point
            for laser_beam in laser_beams:
                # occupancy_map[x_point][y_point]
                occupancy_map[laser_beam[0]-1][laser_beam[1]-1] = 0.0  # free area 0.0
            occupancy_map[ix][iy] = 1.0  # occupied area 1.0
            occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
            occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
            occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
    # occupancy grid computed with with flood fill
    else:
        occupancy_map = init_flood_fill((center_x, center_y), (ox, oy),
                                        (x_w, y_w),
                                        (min_x, min_y), xy_resolution)
        flood_fill((center_x, center_y), occupancy_map)
        occupancy_map = np.array(occupancy_map, dtype=float)
        for (x, y) in zip(ox, oy):
            ix = int(round((x - min_x) / xy_resolution))
            iy = int(round((y - min_y) / xy_resolution))
            occupancy_map[ix][iy] = 1.0  # occupied area 1.0
            occupancy_map[ix + 1][iy] = 1.0  # extend the occupied area
            occupancy_map[ix][iy + 1] = 1.0  # extend the occupied area
            occupancy_map[ix + 1][iy + 1] = 1.0  # extend the occupied area
    return occupancy_map, min_x, max_x, min_y, max_y, xy_resolution, center_x, center_y

def row_to_x(center_x, resolution):
    return( - resolution * center_x )

def col_to_y(center_y, resolution):
    return ( - resolution * center_y)



def callback(data):
    # print out time taken to send data
    print("received data:")
    global angles
    global distances
    global grid
    angles = []
    distances = []
    # NDArray of angles and distances are collected:
    print(data.ranges)
    #for i in range(int((1.74533 - 1.39626) / data.angle_increment)):
    for i in range(int((data.angle_max - data.angle_min) / data.angle_increment)):
        if(data.ranges[i] <= data.range_max and data.ranges[i] >= data.range_min): 
            # print(i, ": ", data.range_max, data.ranges[i], data.intensities[i])
            angles.append(float(i * data.angle_increment))
            distances.append(float(data.ranges[i]))
    
    ang = np.array(angles)
    dist = np.array(distances)
    
    xy_resolution = 0.02  # x-y grid resolution
    # calculate the center points of each angle/distance
    ox = np.sin(ang) * dist
    oy = np.cos(ang) * dist
    occupancy_map, min_x, max_x, min_y, max_y, xy_resolution, center_x, center_y = \
        generate_ray_casting_grid_map(ox, oy, xy_resolution, True)
    xy_res = np.array(occupancy_map).shape
    #for i in occupancy_map:
    #    print(i)
    #print(type(occupancy_map))
    occupancy_map = (occupancy_map*100).astype(np.int8)
    print("measuring points between", data.range_min, "and", data.range_max) 
    

    grid.info.resolution = xy_resolution
    grid.info.width = xy_res[1]
    grid.info.height = xy_res[0]
    # calculate offset 
    print("center", center_x, center_y)
    print("res", xy_res[1], xy_res[0])
    grid.info.origin.position.x = -center_y*xy_resolution
    #row_to_x(center_x, xy_resolution)
    #grid.info.origin.position.x = row_to_x(xy_res[1], (xy_res[1])/2-center_x, xy_resolution)
    
    grid.info.origin.position.y = -center_x*xy_resolution
    #col_to_y(center_y, xy_resolution)

    #grid.info.origin.position.y = col_to_y(xy_res[0], (xy_res[0])/2-center_y, xy_resolution)
    grid.header.frame_id = "laser"
    grid_temp = []
    #print("occupancy map:", occupancy_map, len(occupancy_map))
    for i in range(xy_res[0]):
        for j in range(xy_res[1]):
            grid_temp.append(occupancy_map[i][j])
            #print(occupancy_map[i][j])
    grid.data = grid_temp

def main():
    rospy.Subscriber("scan", LaserScan, callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(grid)
        print("occupancy grid is published")
        rate.sleep()


if __name__ == '__main__':
    main()
