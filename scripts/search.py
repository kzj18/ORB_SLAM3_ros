#!/usr/bin/env python3
# -*- encoding: utf-8 -*-

import os
import math
from glob import glob

import cv2
import yaml
import heapq
import numpy as np
from scipy.interpolate import make_interp_spline

class HeapQueue:
    """堆实现的优先级队列 开节点表
    """    
    def __init__(self):
        self.q = []

    def push(self, node):
        heapq.heappush(self.q, node)

    def top(self):
        return self.q[0]

    def pop(self):
        return heapq.heappop(self.q)

    def empty(self):
        return not len(self.q)

    def clear(self):
        self.q = []

class Hash:
    """闭节点表
    """    
    def __init__(self):
        self.hash_base = 122777
        self.hash_close = [set() for i in range(self.hash_base)]
        self.length = 0

    def reset(self):
        self.hash_close = [set() for i in range(self.hash_base)]
        self.length = 0

    def add(self, state_str):
        self.length += 1
        hash_value = int(state_str) % self.hash_base
        self.hash_close[hash_value].add(state_str)

    def find(self, state_str):
        hash_value = int(state_str) % self.hash_base
        return True if (state_str in self.hash_close[hash_value]) else False

    def __len__(self):
        return self.length

class Node:
    """存放每个搜索节点
    """
    def __init__(self, row:int, col:int, path_cost:float=0., distance:float=0., parent:object=None) -> None:
        self.row = row
        self.col = col
        self.parent = parent
        self.distance = distance
        self.path_cost = path_cost

    def __repr__(self) -> str:
        return "<Node ({:3},{:3})(g={:.2f},h={:.2f})>".format(self.row, self.col, self.path_cost, self.distance)

    def __lt__(self, __o: object) -> bool:
        """地图代价+启发函数+路径代价

        Parameters
        ----------
        __o : object
            比较对象 

        Returns
        -------
        bool
            优先级高于比较对象
        """
        K1 = 1. # distance cost
        K2 = 0. # path length cost
        return (K1*self.distance+K2*self.path_cost) < (K1*__o.distance+K2*__o.path_cost)

    def __eq__(self, __o: object) -> bool:
        return (self.row == __o.row and self.col == __o.col)

    def child_node(self, problem, nrow, ncol) -> object:
        g, h = problem.gh(self.row, self.col, nrow, ncol, self.path_cost)
        next_node = Node(nrow, ncol, g, h, self)
        return next_node

    def get_sample_cnt(self) -> int:
        """距离父节点的曼哈顿距离
        """        
        return (abs(self.row-self.parent.row) + abs(self.col-self.parent.col)) if (self.parent is not None) else 0

    def get_path(self) -> list:
        """返回生成的完整路径

        Returns
        -------
        list
            list[0]:shape = [n,3] 路径长度为n个节点,每行包括[行 列 距离父节点路径的采样点个数]
            list[1]:路径长度 等同于节点的路径代价 不限距离度量方式 此处为欧氏距离和
        """        
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return [[rc.row, rc.col, rc.get_sample_cnt()] for rc in reversed(path_back)], self.path_cost

class GridMap:
    """表示搜索地图
    """
    sample = [0, 2, 3, 4, 6, 7] # related to resolution of navigation path and gridmap
    def __init__(self, config_path:str) -> None:
        """根据地图配置文件加载搜索地图

        Parameters
        ----------
        config_path : str
            地图配置文件路径 *.yaml
        """
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
        self.map_origin = config['origin']
        self.map_resolution = config['resolution']
        map_path = config['image']
        pgm_readable = False
        if not pgm_readable:
            if os.path.exists(map_path):
                self.showmap = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
                if self.showmap is not None:
                    pgm_readable = True
            else:
                map_path = os.path.abspath(os.path.join(config_path, os.pardir, "test.pgm"))

        if not pgm_readable:
            if os.path.exists(map_path):
                self.showmap = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
                if self.showmap is not None:
                    pgm_readable = True
            else:
                map_paths = glob(os.path.abspath(os.path.join(config_path, os.pardir, "*.pgm")))
                if len(map_paths) > 0:
                    map_path = map_paths[0]
            
        if not pgm_readable:
            if os.path.exists(map_path):
                self.showmap = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
                if self.showmap is not None:
                    pgm_readable = True
            else:
                map_path = "/root/test.pgm"

        print("map_path:{}".format(map_path))
        if not pgm_readable:
            self.showmap = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
            
        self.showmap[self.showmap == np.max(self.showmap)] = 255

        # kernel_size = 39
        kernel_size = 7
        self.kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        for i in range(kernel_size):
            for j in range(kernel_size):
                if math.hypot(kernel_size//2-i, kernel_size//2-j) > kernel_size/2:
                    self.kernel[i,j] = 0
        procimg = self.showmap.copy()
        procimg[procimg < 255] = 0
        self.gridmap = cv2.erode(procimg, self.kernel, iterations=1)

        self.mapsize = self.gridmap.shape
        self.search_map = None
        self.goalposi = None
        self.centreArray = np.zeros(2) # control centre to geometry centre

    def setRobotSize(self, robot) -> None:
        self.centreArray[0] = robot[2] - (robot[0]/2.)
        self.centreArray[1] = robot[3] - (robot[1]/2.)

    def CentreControl2Geometry(self, cx, cy, yaw):
        gx = cx + self.centreArray[0] * np.cos(yaw) - self.centreArray[1] * np.sin(yaw)
        gy = cy + self.centreArray[0] * np.sin(yaw) + self.centreArray[1] * np.cos(yaw)
        return gx, gy

    def CentreGeometry2Control(self, gx, gy, yaw):
        cx = gx - self.centreArray[0] * np.cos(yaw) + self.centreArray[1] * np.sin(yaw)
        cy = gy - self.centreArray[0] * np.sin(yaw) - self.centreArray[1] * np.cos(yaw)
        return cx, cy

    def position2pixel(self, x:float, y:float) -> tuple:
        """转换:世界坐标->地图像素点下标

        Parameters
        ----------
        x : float
            map坐标系下x坐标 单位是m
        y : float
            map坐标系下y坐标 单位是m

        Returns
        -------
        tuple
            shape=(2,) 返回对应地图上的像素点位置[row,col]
        """        
        return (int(self.mapsize[0]+(self.map_origin[1]-y)/self.map_resolution), int((x-self.map_origin[0])/self.map_resolution))

    def pixel2position(self, row:int, col:int) -> tuple:
        """转换:地图像素点下标->世界坐标

        Parameters
        ----------
        row : int
        col : int

        Returns
        -------
        tuple
            shape=(2,) 返回世界坐标系下的位置[x,y] 单位是m 
        """        
        return (col*self.map_resolution+self.map_origin[0], (self.mapsize[0]-row)*self.map_resolution+self.map_origin[1])

    def set_origin(self, ox:float, oy:float) -> None:
        """设置起点 确保起点附近是连通的

        Parameters
        ----------
        ox : float
            map坐标系下x坐标 单位是m
        oy : float
            map坐标系下y坐标 单位是m
        """        
        if self.search_map is None:
            print("set goal first")
        else:
            pixel = self.position2pixel(ox, oy)
            if not self.check_accessable(pixel[0], pixel[1]):
                cv2.circle(self.search_map, (pixel[1], pixel[0]), 2, 102, 2)

    def set_goal(self, gx:float, gy:float, obs:list=[]) -> None:
        """设置终点 

        Parameters
        ----------
        gx : float
            目标位置 map坐标系下x坐标 单位是m
        gy : float
            目标位置 map坐标系下y坐标 单位是m
        trans : list
            positions w.r.t. in map
        """
        
        pixel = self.position2pixel(gx, gy)
        gridmap_local = self.gridmap.copy()
        mask = np.zeros_like(self.gridmap)

        self.search_map = cv2.addWeighted(gridmap_local, 0.5, mask, 0.5, 0)
        if len(obs):
            obsmask = np.ones_like(self.gridmap) * 255
            for o in obs:
                r1, c1 = self.position2pixel(o[0], o[1])
                r2, c2 = self.position2pixel(o[2], o[3])
                r1, r2 = min(r1, r2), max(r1, r2)
                c1, c2 = min(c1, c2), max(c1, c2)
                obsmask[r1:r2,c1:c2] = 0
            obsmask = cv2.erode(obsmask, self.kernel, iterations=1)
            self.search_map = cv2.bitwise_and(obsmask, self.search_map)

        cv2.circle(self.search_map, (pixel[1], pixel[0]), 0, 255, 3)

        self.goalposi = (gx, gy)
        
    def clear_map(self) -> None:
        self.search_map = None
        self.goalposi = None

    def linear_interpolation(self, pa:list, pb:list, ratio:float) -> list:
        """线性插值 pn = (1.0-ratio)*pa + ratio*pb

        Parameters
        ----------
        pa : list
            起点
        pb : list
            终点
        ratio : float
            调节比例 range from [0,1]

        Returns
        -------
        list
            the same shape as pa and pb
        """        
        pn = [0.] * len(pa)
        for i in range(len(pa)):
            pn[i] = pa[i] * (1.0-ratio) + pb[i] * ratio
        return pn

    def path_proc(self, path:list, path_dis:float) -> np.ndarray:
        """路径后处理:

            1. 路径节点密插值 让距离变得均匀

            2. 路径节点降采样 减少点的个数 使用样条插值做平滑处理

            3. 平滑后做升采样 控制点间距为设定值 此处为0.025m

        Note
        ---------
        - 如果type(gyaw)==NoneType:围绕终点距离~8pixel*0.05m/pixel=40cm处的圆环为可通行区域
        - 如果type(gyaw)==float:终点即为(gx,gy,gyaw)

        Parameters
        ----------
        path : list
        path_dis : float
            路径的长度

        Returns
        -------
        np.ndarray
            shape = (n,2)
        """        
        path_wrt_map = []
        last_point = None
        for rc in path:
            point = list(self.pixel2position(rc[0], rc[1]))
            if last_point is not None:
                sample_cnt = self.sample[rc[2]]
                for i in range(1, sample_cnt):
                    path_wrt_map.append(self.linear_interpolation(last_point, point, i/sample_cnt))
            path_wrt_map.append(point)
            last_point = point

        # smooth
        navi_path = np.array(path_wrt_map)
        if len(navi_path) < 4:
            print("short path")
            return np.array([[navi_path[-1,0], navi_path[-1,1]]])
        else:
            return navi_path

    def check_accessable(self, row:int, col:int) -> bool:
        """检查节点是否可通行

        Parameters
        ----------
        row : int
        col : int

        Returns
        -------
        bool
        """        
        return self.search_map[row, col] > 0

    def check_goal(self, row:int, col:int) -> bool:
        """检查是否为终点

        Parameters
        ----------
        row : int
        col : int

        Returns
        -------
        bool
        """        
        return self.search_map[row, col] == 255

class Problem:
    costlst = [0., 1., 1.4142135623730951, 2.23606797749979, 3.1622776601683795, 3.6055512754639896]
    def __init__(self, gridmap:GridMap, init_x:float, init_y:float, goal_x:float, goal_y:float, obs:list=[]) -> None:
        self.gridmap = gridmap
        self.gridmap.set_goal(goal_x, goal_y, obs)
        self.gridmap.set_origin(init_x, init_y)
        self.initstate_row, self.initstate_col = self.gridmap.position2pixel(init_x, init_y)
        self.goalstate_row, self.goalstate_col = self.gridmap.position2pixel(goal_x, goal_y)
        self.initstate = Node(self.initstate_row, self.initstate_col, distance=math.hypot(self.initstate_row-self.goalstate_row, self.initstate_col-self.goalstate_col))
        
    def gh(self, row:int, col:int, nrow:int, ncol:int, pathcost:float) -> tuple:
        """代价函数各项

        Parameters
        ----------
        row : int            
        col : int
            当前节点
        nrow : int
        ncol : int
            新节点
        pathcost : float
            当前节点的路径代价

        Returns
        -------
        tuple
            (path cost, goal distance)
        """
        g = pathcost + self.costlst[int(abs(row-nrow) + abs(col-ncol))]
        h = math.hypot(nrow-self.goalstate_row, ncol-self.goalstate_col)
        return g, h

    def actions(self, row:int, col:int) -> list:
        """扩展开节点表 有多种不同的连通域 {4,8,16,32}

        Parameters
        ----------
        row : int
        col : int
            中心节点坐标

        Returns
        -------
        list
            周围可通行区域
        """        
        candidates = [                  [row-3, col-2], [row-3, col-1],               [row-3, col+1], [row-3, col+2],
                        [row-2, col-3],                 [row-2, col-1],               [row-2, col+1],                 [row-2, col+3],
                        [row-1, col-3], [row-1, col-2], [row-1, col-1], [row-1, col], [row-1, col+1], [row-1, col+2], [row-1, col+3],
                                                        [row,   col-1],               [row,   col+1],
                        [row+1, col-3], [row+1, col-2], [row+1, col-1], [row+1, col], [row+1, col+1], [row+1, col+2], [row+1, col+3],
                        [row+2, col-3],                 [row+2, col-1],               [row+2, col+1],                 [row+2, col+3],
                                        [row+3, col-2], [row+3, col-1],               [row+3, col+1], [row+3, col+2],]

        valid_candidates = [item for item in candidates if self.is_valid(item[0], item[1])]
        return valid_candidates

    def is_goal(self, row:int, col:int) -> bool:
        """检查地图下标是否为终点

        Parameters
        ----------
        row : int
        col : int

        Returns
        -------
        bool
        """
        return self.gridmap.check_goal(row, col)

    def is_valid(self, row:int, col:int) -> bool:
        """检查地图下标是否合法(不越界)

        Parameters
        ----------
        row : int
        col : int

        Returns
        -------
        bool
        """
        if 0 <= row < self.gridmap.mapsize[0] and 0 <= col < self.gridmap.mapsize[1]:
            return  self.gridmap.check_accessable(row, col)
        else:
            return False

    def state2index(self, row:int, col:int) -> int:
        """row*sizeof(map)[0] + col

        Parameters
        ----------
        row : int
        col : int

        Returns
        -------
        int
        """        
        return self.gridmap.mapsize[1] * row + col

class Search:
    def __init__(self, config_path) -> None:
        self.gridmap = GridMap(config_path)

    def is_accessable(self, x:float, y:float) -> bool:
        r, c = self.gridmap.position2pixel(x,y)
        return self.gridmap.check_accessable(r, c)
    
    def setRobotSize(self, robot) -> None:
        self.gridmap.setRobotSize(robot)

    def astar_search(self, ori_x:float, ori_y:float, target_x:float, target_y:float, obs:list=[]) -> np.ndarray:
        """搜索接口函数

        Notes
        ---------
        - 如果type(target_yaw)==NoneType:围绕终点距离~8pixel*0.05m/pixel=40cm处的圆环为可通行区域
        - 如果type(target_yaw)==float:终点即为(target_x,target_y,target_yaw)

        Parameters
        ----------
        ori_x : float
            世界坐标系下起点的x坐标 单位m
        ori_y : float
            世界坐标系下起点的y坐标 单位m
        target_x : float
            世界坐标系下终点的x坐标 单位m
        target_y : float
            世界坐标系下终点的y坐标 单位m
        target_yaw : float, optional
            世界坐标系下终点的角度, by default None

        Returns
        -------
        np.ndarray
            搜索得到的路径 shape=(n,3) n*[x,y,yaw]
        """
        pbm = Problem(self.gridmap, ori_x, ori_y, target_x, target_y, obs)
        set_visited_idx = Hash()
        Q_heap_pfs = HeapQueue()
        set_visited_idx.add(pbm.state2index(pbm.initstate_row, pbm.initstate_col))
        Q_heap_pfs.push(pbm.initstate)
        navi_path = None
        while not Q_heap_pfs.empty():
            cur_state = Q_heap_pfs.pop()
            if pbm.is_goal(cur_state.row, cur_state.col):
                path, path_dis = cur_state.get_path()
                navi_path = self.gridmap.path_proc(path, path_dis)
                self.gridmap.clear_map()
                break
            else:
                new_states = pbm.actions(cur_state.row, cur_state.col)
                for st in new_states:
                    stt_idx = pbm.state2index(st[0], st[1])
                    if not set_visited_idx.find(stt_idx):
                        chd_node = cur_state.child_node(pbm, st[0], st[1])
                        Q_heap_pfs.push(chd_node)
                        set_visited_idx.add(stt_idx)
        return navi_path

    def smoothPath(self, navi_path:list, startYaw:float, endYaw:float) -> list:
        t = np.linspace(0, 1, len(navi_path))
        t_smooth = np.linspace(0, 1, max(math.ceil(len(navi_path)/5), 5))
        x_smooth = make_interp_spline(t, navi_path[:,0])(t_smooth)
        y_smooth = make_interp_spline(t, navi_path[:,1])(t_smooth)
        x_ssmooth = make_interp_spline(t_smooth, x_smooth)(t)
        y_ssmooth = make_interp_spline(t_smooth, y_smooth)(t)

        yaw_lst = np.zeros_like(x_ssmooth)
        PREPOINT = min(100, int(len(x_ssmooth)/2-1))

        yaw_lst[PREPOINT:-PREPOINT] = np.arctan2(y_ssmooth[PREPOINT+PREPOINT:]-y_ssmooth[PREPOINT:-PREPOINT], x_ssmooth[PREPOINT+PREPOINT:]-x_ssmooth[PREPOINT:-PREPOINT])
        yaw_lst[0] = startYaw
        dangle = yaw_lst[PREPOINT] - yaw_lst[0]
        if dangle > np.pi:
            dangle -= 2.0 * np.pi
        elif dangle < -np.pi:
            dangle += 2.0 * np.pi
        edangle = dangle / PREPOINT
        for i in range(1, PREPOINT):
            yaw_lst[i] = yaw_lst[i-1] + edangle
            if yaw_lst[i] > np.pi:
                yaw_lst[i] -= 2.0 * np.pi
            elif yaw_lst[i] < -np.pi:
                yaw_lst[i] += 2.0 * np.pi
        yaw_lst[-1] = endYaw
        dangle = yaw_lst[-1] - yaw_lst[-PREPOINT-1]
        if dangle > np.pi:
            dangle -= 2.0 * np.pi
        elif dangle < -np.pi:
            dangle += 2.0 * np.pi
        edangle = dangle / PREPOINT
        for i in range(2, PREPOINT+1):
            yaw_lst[-i] = yaw_lst[1-i]  - edangle
            if yaw_lst[-i] > np.pi:
                yaw_lst[-i] -= 2.0 * np.pi
            elif yaw_lst[-i] < -np.pi:
                yaw_lst[-i] += 2.0 * np.pi
        spath = np.vstack((x_ssmooth, y_ssmooth, yaw_lst))
        return spath.transpose()

    def smoothPathNoSidewards(self, navi_path:list, startYaw:float, endYaw:float) -> list:
        t = np.linspace(0, 1, len(navi_path))
        t_smooth = np.linspace(0, 1, max(math.ceil(len(navi_path)/100), 5))
        x_smooth = make_interp_spline(t, navi_path[:,0])(t_smooth)
        y_smooth = make_interp_spline(t, navi_path[:,1])(t_smooth)
        x_ssmooth = make_interp_spline(t_smooth, x_smooth)(t)
        y_ssmooth = make_interp_spline(t_smooth, y_smooth)(t)

        yaw = np.arctan2(y_ssmooth[1:]-y_ssmooth[:-1], x_ssmooth[1:]-x_ssmooth[:-1])
        newposi = np.repeat(np.vstack([x_ssmooth, y_ssmooth]).transpose(), 2, 0)
        newyaw = np.vstack([startYaw, np.repeat(yaw, 2).reshape(-1,1), endYaw])
        newpath = np.hstack([newposi, newyaw])
        return newpath

    def drawPath(self, navipath:list, arrow_length:float=0.5, arrow_thickness:int=2, arrow_color:tuple=(255,0,0), gap_cnt:int=0, img=None) -> np.ndarray:
        if img is None:
            gdmap = cv2.cvtColor(self.gridmap.showmap, cv2.COLOR_GRAY2BGR)        
        else:
            gdmap = img
        point = navipath[0]
        pix = self.gridmap.position2pixel(point[0], point[1])
        cv2.circle(gdmap, (pix[1], pix[0]), arrow_thickness+1, arrow_color, arrow_thickness+2)
        for i, point in enumerate(navipath):
            yaw = point[2] if len(point) > 2 else 0.
            if i % (gap_cnt+1) == 0:
                pix0 = self.gridmap.position2pixel(point[0], point[1])
                pix1 = self.gridmap.position2pixel(point[0] + arrow_length * np.cos(yaw), point[1] + arrow_length * np.sin(yaw))
                cv2.arrowedLine(gdmap, (pix0[1], pix0[0]), (pix1[1], pix1[0]), arrow_color, arrow_thickness, cv2.LINE_AA)
        if (len(navipath)-1) % (gap_cnt+1):
            point = navipath[-1]
            yaw = point[2] if len(point) > 2 else 0.
            pix0 = self.gridmap.position2pixel(point[0], point[1])
            pix1 = self.gridmap.position2pixel(point[0] + arrow_length * np.cos(yaw), point[1] + arrow_length * np.sin(yaw))
            cv2.arrowedLine(gdmap, (pix0[1], pix0[0]), (pix1[1], pix1[0]), arrow_color, arrow_thickness, cv2.LINE_AA)
        return gdmap

    def getRobotPosition(self, robot, posi):
        w1, w2, h = robot[2], robot[0]-robot[2], robot[3]
        cx, cy = posi[0], posi[1]
        ang = posi[2] if len(posi) > 2 else 0.
        
        x1 = cx + w1 * np.cos(ang) - h * np.sin(ang)
        y1 = cy + w1 * np.sin(ang) + h * np.cos(ang)
        x2 = cx + w1 * np.cos(ang) + h * np.sin(ang)
        y2 = cy + w1 * np.sin(ang) - h * np.cos(ang)
        x3 = cx - w2 * np.cos(ang) + h * np.sin(ang)
        y3 = cy - w2 * np.sin(ang) - h * np.cos(ang)
        x4 = cx - w2 * np.cos(ang) - h * np.sin(ang)
        y4 = cy - w2 * np.sin(ang) + h * np.cos(ang)
        
        return x1, y1, x2, y2, x3, y3, x4, y4

    def drawRobot(self, img, robot, posi, thickness:int=1, color:tuple=(255,0,0)):
        x1, y1, x2, y2, x3, y3, x4, y4 = self.getRobotPosition(robot, posi)

        pix1 = self.gridmap.position2pixel(x1, y1)
        pix2 = self.gridmap.position2pixel(x2, y2)
        pix3 = self.gridmap.position2pixel(x3, y3)
        pix4 = self.gridmap.position2pixel(x4, y4)

        # 绘制直线构成闭合的矩形
        cv2.line(img, (pix1[1], pix1[0]), (pix2[1], pix2[0]), color, thickness)
        cv2.line(img, (pix2[1], pix2[0]), (pix3[1], pix3[0]), color, thickness)
        cv2.line(img, (pix3[1], pix3[0]), (pix4[1], pix4[0]), color, thickness)
        cv2.line(img, (pix4[1], pix4[0]), (pix1[1], pix1[0]), color, thickness)

    def drawRobotPath(self, robot, navipath:list, arrow_length:float=0.5, arrow_thickness:int=2, arrow_color:tuple=(255,0,0), gap_cnt:int=0, img=None) -> np.ndarray:
        if img is None:
            gdmap = cv2.cvtColor(self.gridmap.showmap, cv2.COLOR_GRAY2BGR)
        else:
            gdmap = img
        point = navipath[0]
        pix = self.gridmap.position2pixel(point[0], point[1])
        cv2.circle(gdmap, (pix[1], pix[0]), arrow_thickness, arrow_color, arrow_thickness+2)
        for i, point in enumerate(navipath):
            if i % (gap_cnt+1) == 0:
                yaw = point[2] if len(point) > 2 else 0.
                pix0 = self.gridmap.position2pixel(point[0], point[1])
                pix1 = self.gridmap.position2pixel(point[0] + arrow_length * np.cos(yaw), point[1] + arrow_length * np.sin(yaw))
                cv2.circle(gdmap, (pix0[1], pix0[0]), arrow_thickness, arrow_color, arrow_thickness+2)
                cv2.arrowedLine(gdmap, (pix0[1], pix0[0]), (pix1[1], pix1[0]), arrow_color, arrow_thickness, cv2.LINE_AA)
                self.drawRobot(gdmap, robot, point, arrow_thickness, arrow_color)
        if (len(navipath)-1) % (gap_cnt+1):
            point = navipath[-1]
            yaw = point[2] if len(point) > 2 else 0.
            pix0 = self.gridmap.position2pixel(point[0], point[1])
            pix1 = self.gridmap.position2pixel(point[0] + arrow_length * np.cos(yaw), point[1] + arrow_length * np.sin(yaw))
            cv2.circle(gdmap, (pix0[1], pix0[0]), arrow_thickness, arrow_color, arrow_thickness+2)
            cv2.arrowedLine(gdmap, (pix0[1], pix0[0]), (pix1[1], pix1[0]), arrow_color, arrow_thickness, cv2.LINE_AA)
            self.drawRobot(gdmap, robot, point, arrow_thickness, arrow_color)
        return gdmap