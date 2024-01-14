import os

import numpy as np
import cv2
from PIL import Image

from search import Search

WORKSPACE = os.path.dirname(os.path.abspath(__file__))

class Tester:
    
    def __init__(self, config_path:str, map_path:str) -> None:
        self.__map = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        self.__map:np.ndarray
        self.__map[self.__map < 175] = 0
        self.__map[np.logical_and(self.__map > 175, self.__map < 205)] = 205
        self.__map[np.logical_and(self.__map > 205, self.__map < 255)] = 255
        self.__map = cv2.cvtColor(self.__map, cv2.COLOR_GRAY2BGR)
            
        self.__init_point = None
        self.__goal_point = None
        self.__waypoints = None
        self.__search = Search(config_path)
        self.__origin_row, self.__origin_col = self.__search.gridmap.position2pixel(0, 0)
        self.__rotate_degrees = 0
        print(self.__origin_row, self.__origin_col)
        cv2.circle(self.__map, (self.__origin_col, self.__origin_row), 1, (0, 0, 255), cv2.FILLED)
        
        self.__redraw_map()
        cv2.setMouseCallback('map', self.__mouse_callback)
        
        while True:
            try:
                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    print(key, repr(chr(key)))
                if key in [ord('q'), ord("\x1b")]:
                    break
                if key == ord(' '):
                    self.__calculate_path()
                if key == ord('\x08'):
                    self.__clear_map()
                if key == ord('='):
                    self.__rotate_map(1)
                if key == ord('-'):
                    self.__rotate_map(-1)
                self.__redraw_map()
            except KeyboardInterrupt:
                break
            
        Image.fromarray(self.__map).save(os.path.join(WORKSPACE, 'map.pgm'))
        
            
    def __redraw_map(self) -> None:
        show_map = self.__map.copy()
        
        if self.__init_point is not None:
            cv2.circle(show_map, self.__init_point, 4, (0, 255, 0), 1)
            
        if self.__goal_point is not None:
            cv2.circle(show_map, self.__goal_point, 4, (0, 0, 255), 1)
            
        if self.__waypoints is not None:
            if self.__waypoints.shape[1] == 2:
                for x_world, y_world in self.__waypoints:
                    y_pixel, x_pixel = self.__search.gridmap.position2pixel(x_world, y_world)
                    cv2.circle(show_map, (x_pixel, y_pixel), 1, (255, 0, 0), -1)
            elif self.__waypoints.shape[1] == 3:
                for x_world, y_world, yaw in self.__waypoints:
                    y_pixel, x_pixel = self.__search.gridmap.position2pixel(x_world, y_world)
                    cv2.arrowedLine(show_map, (x_pixel, y_pixel), (x_pixel + int(10 * np.cos(yaw)), y_pixel + int(10 * np.sin(yaw))), (255, 0, 0), 1)
        
        cv2.imshow('map', show_map)
        
    def __calculate_path(self) -> None:
        if self.__init_point is None or self.__goal_point is None:
            return
        else:
            init_point = self.__search.gridmap.pixel2position(*reversed(self.__init_point))
            goal_point = self.__search.gridmap.pixel2position(*reversed(self.__goal_point))
            self.__waypoints = self.__search.astar_search(*init_point, *goal_point)
            if self.__waypoints is None:
                print('No path found!')
            else:
                print(self.__waypoints.shape)
                
    def __rotate_map(self, angle:float) -> None:
        self.__rotate_degrees += angle
        print(self.__rotate_degrees)
        center = (self.__origin_col, self.__origin_row)
        rotate_matrix = cv2.getRotationMatrix2D(center, angle, 1)
        self.__map = cv2.warpAffine(self.__map, rotate_matrix, self.__map.shape[:2][::-1])
    
    def __clear_map(self) -> None:
        self.__init_point = None
        self.__goal_point = None
        self.__waypoints = None
        
    def __mouse_callback(self, event:int, x:int, y:int, flags:int, param:int) -> None:
        if event == cv2.EVENT_LBUTTONDBLCLK:
            self.__init_point = (x, y)
            print(self.__init_point)
        
        if event == cv2.EVENT_MBUTTONDBLCLK:
            self.__goal_point = (x, y)
            print(self.__goal_point)
    
if __name__ == '__main__':
    tester = Tester(os.path.join(WORKSPACE, 'scans.yaml'), os.path.join(WORKSPACE, 'scans.pgm'))
    # tester = Tester("/home/kzj18/Projects/GUOQIANG_INSTITUTE_CUP_2023/plan/P008599-B0001-F0007/P008599-B0001-F0007.yaml", "/home/kzj18/Projects/GUOQIANG_INSTITUTE_CUP_2023/plan/P008599-B0001-F0007/P008599-B0001-F0007.pgm")
