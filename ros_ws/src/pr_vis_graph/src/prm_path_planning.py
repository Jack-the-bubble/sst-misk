import random
import math
import numpy as np
# import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import cv2

# parameter
N_SAMPLE = 1200  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 50.0  # [m] Maximum edge length
MIN_EDGE_LEN = 5.0
DISTANCE_TO_OBSTACLE = 5.0

show_animation = True
plot_graph = not(show_animation)


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
            str(self.cost) + "," + str(self.parent_index)


class PRVG:
    """
    Class for probability visibility diagram
    """
    def __init__(self) -> None:
        self.graph = []
        self.map_obs_x = []
        self.map_obs_y = []
        self.map_obs = tuple()


    # def generate

    def imageToArray(self,img_path = '../maps/test_map_round.png'):
        image = cv2.imread(img_path)
        ox = []
        oy = []
        for i in range(image.shape[0]):
            for j in range(image.shape[1]):
                if np.sum(image[i][j]) == 0:
                    ox.append(j)
                    oy.append(i)

        return ox, oy
        # exit()
        # cv2.imshow('image', image)
        # cv2.waitKey()

    def setup(self, distance_to_obstacle, n_sample, n_knn, max_edge_len, min_edge_len):
        self.distance_to_obstacle = distance_to_obstacle
        self.n_sample = n_sample
        self.n_knn = n_knn
        self.max_edge_len = max_edge_len
        self.min_edge_len = min_edge_len


    def loadImage(self,img_path = '../maps/test_map_round.png'):
        self.map_obs_x, self.map_obs_y = self.imageToArray(img_path)
        self.map_obs = zip(self.map_obs_x,self.map_obs_y)
    
    def generateGraph(self) -> None:
        try:
            assert self.map_obs_x, 'Map not loaded'
        except:
            self.loadImage()
        obstacle_kd_tree = cKDTree(np.vstack((self.map_obs_x, self.map_obs_y)).T)

        sample_x, sample_y = self.generatePoints(obstacle_kd_tree)
        if show_animation:
            plt.plot(sample_x, sample_y, ".b")

        self.graph = self.generate_road_map(sample_x, sample_y,  obstacle_kd_tree)


    def prm_planning(self,sx, sy, gx, gy, ox, oy):

        obstacle_kd_tree = cKDTree(np.vstack((ox, oy)).T)

        sample_x, sample_y = self.sample_points(sx, sy, gx, gy,
                                         ox, oy, obstacle_kd_tree)
        if show_animation:
            plt.plot(sample_x, sample_y, ".b")

        road_map = self.generate_road_map(sample_x, sample_y,  obstacle_kd_tree)

        rx, ry = self.dijkstra_planning(
            sx, sy, gx, gy, road_map, sample_x, sample_y)

        return rx, ry


    def  is_collision(self,sx, sy, gx, gy,  obstacle_kd_tree):
        x = sx
        y = sy
        dx = gx - sx
        dy = gy - sy
        yaw = math.atan2(gy - sy, gx - sx)
        d = math.hypot(dx, dy)

        if d >= MAX_EDGE_LEN or d <= MIN_EDGE_LEN:
            return True

        D = self.distance_to_obstacle
        n_step = round(d / D)

        for i in range(n_step):
            dist, _ = obstacle_kd_tree.query([x, y])
            if dist <= D:
                return True  # collision
            x += D * math.cos(yaw)
            y += D * math.sin(yaw)

        # goal point check
        dist, _ = obstacle_kd_tree.query([gx, gy])
        if dist <= D:
            return True  # collision

        return False  # OK


    def generate_road_map(self,sample_x, sample_y, obstacle_kd_tree):
        """
        Road map generation
        sample_x: [m] x positions of sampled points
        sample_y: [m] y positions of sampled points
        obstacle_kd_tree: KDTree object of obstacles
        """

        road_map = []
        n_sample = len(sample_x)
        sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)

        for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

            dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
            edge_id = []

            for ii in range(1, len(indexes)):
                nx = sample_x[indexes[ii]]
                ny = sample_y[indexes[ii]]

                if not self.is_collision(ix, iy, nx, ny, obstacle_kd_tree):
                    edge_id.append(indexes[ii])

                if len(edge_id) >= N_KNN:
                    break

            road_map.append(edge_id)

        if plot_graph:
            print("plotting graph")
            self.plot_road_map(road_map, sample_x, sample_y)

        return road_map


    def dijkstra_planning(self,sx, sy, gx, gy, road_map, sample_x, sample_y):
        """
        s_x: start x position [m]
        s_y: start y position [m]
        gx: goal x position [m]
        gy: goal y position [m]
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        road_map: visibility graph [m]
        sample_x: ??? [m]
        sample_y: ??? [m]
        @return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
        """

        start_node = Node(sx, sy, 0.0, -1)
        goal_node = Node(gx, gy, 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[len(road_map) - 2] = start_node

        path_found = True

        while True:
            if not open_set:
                print("Cannot find path")
                path_found = False
                break

            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation and len(closed_set.keys()) % 2 == 0:
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(current.x, current.y, "xg")
                plt.pause(0.001)

            if c_id == (len(road_map) - 1):
                print("goal is found!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]
            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for i in range(len(road_map[c_id])):
                n_id = road_map[c_id][i]
                dx = sample_x[n_id] - current.x
                dy = sample_y[n_id] - current.y
                d = math.hypot(dx, dy)
                node = Node(sample_x[n_id], sample_y[n_id],
                            current.cost + d, c_id)

                if n_id in closed_set:
                    continue
                # Otherwise if it is already in the open set
                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].cost = node.cost
                        open_set[n_id].parent_index = c_id
                else:
                    open_set[n_id] = node

        if path_found is False:
            return [], []

        # generate final course
        rx, ry = [goal_node.x], [goal_node.y]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(n.x)
            ry.append(n.y)
            parent_index = n.parent_index

        return rx, ry


    def plot_road_map(self,road_map, sample_x, sample_y):  # pragma: no cover

        for i, _ in enumerate(road_map):
            for ii in range(len(road_map[i])):
                ind = road_map[i][ii]

                plt.plot([sample_x[i], sample_x[ind]],
                        [sample_y[i], sample_y[ind]], "-k")
        plt.show()

    def generatePoints(self,obstacle_kd_tree):
        max_x = max(self.map_obs_x) 
        max_y = max(self.map_obs_y)
        min_x = min(self.map_obs_x)
        min_y = min(self.map_obs_y)

        sample_x, sample_y = [], []

        while len(sample_x) <= N_SAMPLE:
            tx = (random.random() * (max_x - min_x)) + min_x
            ty = (random.random() * (max_y - min_y)) + min_y

            dist, index = obstacle_kd_tree.query([tx, ty])

            if dist >= self.distance_to_obstacle:
                sample_x.append(tx)
                sample_y.append(ty)

        #this need to be changed to hmmm sth not based on one start/target
        # sample_x.append(sx)
        # sample_y.append(sy)
        # sample_x.append(gx)
        # sample_y.append(gy)

        return sample_x, sample_y

    def sample_points(self, ox, oy, obstacle_kd_tree):
        max_x = max(ox)
        max_y = max(oy)
        min_x = min(ox)
        min_y = min(oy)

        sample_x, sample_y = [], []

        while len(sample_x) <= N_SAMPLE:
            tx = (random.random() * (max_x - min_x)) + min_x
            ty = (random.random() * (max_y - min_y)) + min_y

            dist, index = obstacle_kd_tree.query([tx, ty])

            if dist >= self.distance_to_obstacle:
                sample_x.append(tx)
                sample_y.append(ty)

        #this need to be changed to hmmm sth not based on one start/target
        # sample_x.append(sx)
        # sample_y.append(sy)
        # sample_x.append(gx)
        # sample_y.append(gy)

        return sample_x, sample_y


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 100.0  # [m]
    sy = 200.0  # [m]
    gx = 1250.0  # [m]
    gy = 550.0  # [m]
    robot_size = DISTANCE_TO_OBSTACLE  # [m]

    ox = []
    oy = []

    prvg = PRVG()
    ox, oy = prvg.imageToArray()

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "^r")
        plt.plot(gx, gy, "^c")
        plt.grid(True)
        plt.axis("equal")

    rx, ry = prvg.prm_planning(sx, sy, gx, gy, ox, oy, DISTANCE_TO_OBSTACLE)
    print(rx,ry)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()

def test():
    prvg = PRVG()
    prvg.setup(DISTANCE_TO_OBSTACLE,N_SAMPLE,N_KNN,MAX_EDGE_LEN,MIN_EDGE_LEN)
    prvg.generateGraph()

if __name__ == '__main__':
    #test case
    # main()
    test()
