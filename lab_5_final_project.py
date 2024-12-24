import cv2
import os
import numpy as np
import threading
import pybullet as p
import pybullet_data
import math
import time
import socket
import heapq
def changeCameraCoordinateToRobot(x_camera, y_camera):
    x_rob = ((x_camera * -0.189560) - 69.31)*0.001
    y_rob = ((y_camera * 0.192) - 455.7032)*0.001

    return [x_rob, y_rob, 0.1]


def detect_lines_corners():
    cap = cv2.VideoCapture(0)  # Replace 0 with the video file path if needed
    line_endpoints=[]
    corners=[]
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        x_start, y_start, width, height = 600, 160, 700, 700  # Example values
        roi = frame[y_start:y_start+height, x_start:x_start+width]
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Threshold to isolate black lines (invert binary for black-on-white)
        _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)       


        # Hough Line Transform for line detection
        min_line_length = 50  # Minimum line length in pixels
        max_line_gap = 10      # Maximum gap between line segments
        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, threshold=120,
                                minLineLength=min_line_length,
                                maxLineGap=max_line_gap)
        x_mn, x_mx, y_mn, y_mx = math.inf, 0, math.inf, 0
        line_endpoints = []
        corners=[]
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_mn, x_mx = min(x_mn, x1, x2), max(x_mx, x1, x2)
                y_mn, y_mx = min(y_mn, y1, y2), max(y_mx, y1, y2)
                if abs(x2 - x1) > abs(y2 - y1):
                    y1 = y2 = round((y1 + y2) / 2)
                else:
                    x1 = x2 = round((x1 + x2) / 2)

                line_endpoints.append([x_start+x1, y_start+y1, x_start+x2, y_start+y2])
                # Draw detected lines on the original frame
                cv2.line(frame, (x_start+x1, y_start+y1), (x_start+x2, y_start+y2), (0, 255, 0), 3)

        corners = [[x_start+x_mn, y_start+y_mn], [x_start+x_mx, y_start+y_mn], [x_start+x_mn, y_start+y_mx], [x_start+x_mx, y_start+y_mx]]
        cv2.imshow('Detected Lines', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    for line in line_endpoints:
        print(f"Line coordinates: ({line[0]}, {line[1]}) to ({line[2]}, {line[3]})")
    for corner in corners:
        print(corner[0]," ", corner[1])
    cap.release()
    cv2.destroyAllWindows()
    return [line_endpoints, corners]    
    


def pixel2grid(x, y, x_mn, y_mn, x_mx, y_mx, grid_size):
    step_x = (x_mx - x_mn) / (grid_size - 1)
    step_y = (y_mx - y_mn) / (grid_size - 1)
    print("step_x", step_x,"step_y", step_y)
    row = max(0, min(grid_size - 1, round((x - x_mn) / step_x)))
    col = max(0, min(grid_size - 1, round((y - y_mn) / step_y)))
    return [row, col]


def grid2pixel(grid_point,x_mn,y_mn,x_mx,y_mx,grid_size):
    row = grid_point[0]
    col = grid_point[1]
    pixel_x = x_mn + row*(x_mx-x_mn)/(grid_size-1)
    pixel_y = y_mn + col*(y_mx-y_mn)/(grid_size-1)
    print ("x_mn",x_mn)
    return [pixel_x,pixel_y]

def find_start_and_end(grid_table):
    start_point = None
    end_point = None
    grid_size = len(grid_table)

    # Check top and bottom edges
    for i in range(grid_size):
        if grid_table[0][i] == 1 and start_point is None:
            start_point = [0, i]
            break
    
    if start_point:
        # If start is on top, look for end on bottom
        for i in range(grid_size):
            if grid_table[grid_size-1][i] == 1:
                end_point = [grid_size-1, i]
                break
    else:
        # Check left and right edges
        for i in range(grid_size):
            if grid_table[i][0] == 1 and start_point is None:
                start_point = [i, 0]
                break
        
        if start_point:
            # If start is on left, look for end on right
            for i in range(grid_size):
                if grid_table[i][grid_size-1] == 1:
                    end_point = [i, grid_size-1]
                    break

    return start_point, end_point


def find_path(grid, start, end):
    def heuristic(a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    def get_neighbors(point):
        x, y = point
        neighbors = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        return [(nx, ny) for nx, ny in neighbors if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 1]

    start = tuple(start)  # Convert start to tuple
    end = tuple(end)  # Convert end to tuple

    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_list:
        current = heapq.heappop(open_list)[1]

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return [list(p) for p in path[::-1]]  # Convert back to list of lists

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end)
                heapq.heappush(open_list, (f_score[neighbor], neighbor))

    return None  # No path found


detectLinesCorners = detect_lines_corners()
lines = detectLinesCorners[0]
print("detected " , len(lines))
corners=detectLinesCorners[1]
grid_size = 9
x_mn = corners[0][0]
y_mn = corners[0][1]
x_mx = corners[3][0]
y_mx = corners[3][1]
grid_table = np.ones((grid_size,grid_size),dtype=int)
for line in lines:
    grid_point1 = pixel2grid(line[0],line[1],x_mn,y_mn,x_mx,y_mx,grid_size)
    grid_point2 = pixel2grid(line[2],line[3],x_mn,y_mn,x_mx,y_mx,grid_size)
    x1, y1 = grid_point1
    x2, y2 = grid_point2
    for x in range(min(x1, x2), max(x1, x2) + 1):
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if 0 <= x < grid_size and 0 <= y < grid_size:
                grid_table[x][y] = 0

print("lenth of grid_table",  len(grid_table))
for grid in grid_table:
    for i in grid:
        print(i,end=" ")
    print("\n")
    
one_count=0
for grid in grid_table:
    for i in grid:
        if i==1:
            one_count=one_count+1
print("one_count",one_count)
start_point, end_point = find_start_and_end(grid_table)
print("start_point",start_point," end_point",end_point)
if start_point and end_point:
    print(f"Start Point: {start_point}, End Point: {end_point}")
else:
    print("Start point or End point not found")


pixel_coordinates_path = []
path = []
path = find_path(grid_table,start_point,end_point)
for grid_coord in path:
    print(grid_coord)
for grid_point in path:
    pixel_coordinates_path.append(grid2pixel(grid_point,x_mn,y_mn,x_mx,y_mx,grid_size))
for pixel_coord in pixel_coordinates_path:
    print(pixel_coord[0],pixel_coord[1])
robot_coordinates_path = []
for pixel_coordinate in pixel_coordinates_path:
    robot_coord = changeCameraCoordinateToRobot(pixel_coordinate[0],pixel_coordinate[1])
    robot_coordinates_path.append(robot_coord)
for robot_coord in robot_coordinates_path:
    print(robot_coord[0],robot_coord[1])