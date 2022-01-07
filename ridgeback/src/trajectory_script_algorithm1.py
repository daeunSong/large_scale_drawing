import time, math
import matplotlib.pyplot as plt
import numpy as np

class Iidgeback:
    def __init__(self, id, rx, ry, wall, radius=0.622):
        self.id = id
        self.r_center = [rx, ry]
        self.i_center = [rx, ry]
        self.r_radius = radius
        self.i_radius = 0.3
        self.cover_wall_amount = 0
        self.cover_point = []
        self.min_x = 0
        self.max_x = 0
        self.wall = wall

    def set_angle(self):

        self.cover_point.sort()
        x1, y1 = self.cover_point[0]
        x2, y2 = self.cover_point[-1]

        hypot = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        angle = math.acos((x2-x1)/hypot)
        self.angle = math.pi/2 - angle
        if y1 < y2:
            self.direction = 'l'
            print('left')
        else:
            self.direction = 'r'
            print('right')
        print('--------------------angle: ',math.degrees(self.angle))
        self.set_ridgeback()

        return self.angle

    def set_ridgeback(self, hypot= 0.8):
        x1, y1 = self.cover_point[0]
        x2, y2 = self.cover_point[-1]
        direction = (y2-y1)/(x2-x1)
        if direction >= 0:
            self.r_center[0] = self.i_center[0] + hypot*math.cos(self.angle)
            self.r_center[1] = self.i_center[1] - hypot*math.sin(self.angle)
            if not self.ridgeback_can_go(self.wall):
                self.r_center[0] = self.i_center[0] + hypot * math.cos(self.angle)
            if not self.ridgeback_can_go(self.wall):
                self.r_center[1] = self.i_center[1] + hypot*math.sin(self.angle)

        else:
            self.r_center[0] = self.i_center[0] - hypot*math.cos(self.angle)
            self.r_center[1] = self.i_center[1] - hypot*math.sin(self.angle)
            if not self.ridgeback_can_go(self.wall):
                self.r_center[0] = self.i_center[0] + hypot * math.cos(self.angle)
            if not self.ridgeback_can_go(self.wall):
                self.r_center[1] = self.i_center[1] + hypot*math.sin(self.angle)

    def in_limit(self, i, j):
        return False

    def can_be_generated(self, wall):
        for (i, j) in wall.uncovered + wall.covered:
            if self.in_limit(i, j):
                return False
            else:
                continue
        return True
    def ridgeback_can_go(self, wall):
        for (i, j) in self.wall.uncovered + self.wall.covered:
            if ((i - self.r_center[0]) ** 2 + (j - self.r_center[1]) ** 2 - (self.r_radius + 0.1) ** 2) <=0:
                return False
        return True
    def in_iiwa_range(self, i, j):
        return ((i - self.i_center[0]) ** 2 + (j - self.i_center[1]) ** 2 - self.i_radius ** 2) < 0
    def print_map(self):
        i_circle = plt.Circle((self.i_center[0], self.i_center[1]), self.i_radius, fill=None, alpha=1, color='orange')
        r_circle = plt.Circle((self.r_center[0], self.r_center[1]), self.r_radius, fill=None, alpha=1)
        plt.gca().add_patch(i_circle)
        plt.gca().add_patch(r_circle)

    def cover_amount(self, wall):
        count = 0
        for (x, y) in wall.uncovered:
            if self.in_iiwa_range(x, y):
                count += 1
            else:
                continue
        self.cover_wall_amount = count
        return count

    def calc_cover_points(self, wall):
        for (x, y) in zip(wall.xpoints, wall.ypoints):
            if self.in_iiwa_range(x, y):
                self.cover_point.append((x, y))
            else:
                continue
        return self.cover_point

    def plot_direction(self):
        plt.plot([self.r_center[0], self.i_center[0]], [self.r_center[1], self.i_center[1]], 'm-')

    def print_id(self):
        plt.text(self.r_center[0], self.r_center[1], self.id, fontsize=1)

class Wall:
    def __init__(self, x, y):
        self.covered = []
        self.uncovered = list(zip(x, y))
        self.xpoints = x
        self.ypoints = y

    def add_covered(self, candidate):
        cx = []
        cy = []
        for (x, y) in candidate.calc_cover_points(self):
            if (x, y) in self.uncovered:
                # print('(' + str(x) + ',' + str(y) + ')', end=' ')
                self.covered.append((x, y))
                self.uncovered.remove((x, y))
                cx.append(x)
                cy.append(y)
            else:
                continue
        plt.scatter(cx, cy, marker="1")
        print()
        print('length covered:', len(self.covered), '/', len(self.uncovered) + len(self.covered))

    def allcovered(self):
        if len(self.uncovered) <3:
            return True
        else:
            return False

    def print_map(self):
        plt.plot(self.xpoints, self.ypoints)

class Candidate:
    def __init__(self, wall, input_wall):
        self.wall = wall
        self.candidate = self.generate_c()
        self.input_wall = input_wall

    def generate_c(self):
        start = time.time()
        IR = []
        id = 0
        limit = 0.2
        count = 1
        x_interval = generate_interval(self.wall.xpoints, count)
        y_interval = generate_interval(self.wall.ypoints, count)
        for i in range(len(x_interval)):
            for j in np.arange(y_interval[i] - 2, y_interval[i] - limit, 0.05):
                generated_circle = Iidgeback(id, round(x_interval[i], 3), round(j, 3), self.wall)
                if generated_circle.can_be_generated(self.wall):
                    IR.append(generated_circle)
                else:
                    continue
                id += 1
        print('points generated. Time:', time.time() - start, len(IR))
        return IR

    def delete_c(self, c):
        self.candidate.remove(c)

    def draw_candidates(self):
        for i in self.candidate:
            i.print_map()

def max_coverage(wall, C):
    max_covered_points = 0
    candidates = []
    final_candidates = []
    for c in C:
        covered_points = c.cover_amount(wall)
        if covered_points >= max_covered_points:
            max_covered_points = covered_points
            candidates.append(c)
        else:
            continue
    for c in candidates:
        covered_points = c.cover_amount(wall)
        if covered_points == max_covered_points:
            final_candidates.append(c)
        else:
            continue
    selected = final_candidates[len(final_candidates) // 3]
    print("++++++++++++++++++++++", len(final_candidates))
    print("max_coverage id:", selected.id, 'covers:', max_covered_points)
    if max_covered_points == 0:
        return None
    return selected

    
def greedy_cover_iiwa(wall, C):
    print('in greedy')
    t_start = time.time()
    steps = []
    while not wall.allcovered():
        max_circle = max_coverage(wall, C.candidate)
        if max_circle == None:
            return steps

        wall.add_covered(max_circle)
        C.delete_c(max_circle)
        steps.append(max_circle)
        max_circle.set_angle()
        max_circle.print_map()
        max_circle.print_id()
        max_circle.plot_direction()
    print('wall all covered')
    for s in steps:
        # print(s.i_center, end="")
        pass
    print("path generated with", len(steps), "circles")
    t_end = time.time()
    print('time: ', t_end - t_start)
    return steps

def generate_interval(wall, count=1):
    interval_wall = []
    for i in range(len(wall) - 1):
        interval_wall.extend(np.linspace(wall[i], wall[i + 1], count))
    rounded_wall = [round(x, 3) for x in interval_wall]
    return rounded_wall


def setting(circle):
    print('c:', circle.r_center)
    return circle.r_center[0]

def open_file(filename, fileext):
    import os

    cwd = os.getcwd()  # Get the current working directory (cwd)
    # files = os.listdir(cwd) 
    point = []
    wall = []
    
    # with open(cwd[:-13]+'input/' + filename + '.' + fileext) as f:
    with open('/home/jypark/catkin_ws/src/large_scale_drawing/wall/' + filename + '.' + fileext) as f:
        for line in f:
            if line[0] != 'v' or line[:2] == 'vn':
                continue
            for word in line.split():
                if word == 'v':
                    point = []
                    continue
                else:
                    point.append(float(word))
            wall.append(point)
    return wall

def plot_wall_draw(wall, size=0.4):
    x_wall = []
    y_wall = []
    z_wall = []

    marker_shape="o"

    for [x, y, z] in wall:
        x_wall.append(x)
        y_wall.append(y)
        z_wall.append(z)
    if len([i for i in x_wall[:10] if i==0]) > 3:
        z_wall = [-z for z in z_wall]
        # plt.scatter(y_wall, z_wall, c='indigo', s=size, marker=marker_shape)
        print('zero: x')
        return y_wall, z_wall
    elif len([i for i in y_wall[:10] if i==0]) > 3:
        z_wall = [-z for z in z_wall]
        # plt.scatter(x_wall, z_wall, c='indigo', s=size, marker=marker_shape)
        print('zero: y')
        return x_wall, z_wall
    elif len([i for i in z_wall[:10] if i==0]) > 3:
        y_wall = [-y for y in y_wall]
        # plt.scatter(x_wall, y_wall, c='indigo', s=size, marker=marker_shape)
        print('zero: z')
        return x_wall, y_wall
    print('done')

def to_gazebo_cmd_format(steps):
    sorted_path = sorted(steps, key=setting)
    min_x_list, max_x_list= [], []
    path_x, path_y = [], []
    path_angle = []
    for s in sorted_path:
        min_x_list.append(s.cover_point[0][0])
        max_x_list.append(s.cover_point[-1][0])
        path_x.append(s.r_center[0])
    for s in sorted_path:
        path_y.append(s.r_center[1])
    for s in sorted_path:
        path_angle.append(s.direction+' '+str(math.degrees(s.angle)))

    return min_x_list, max_x_list, path_x, path_y, path_angle

def to_iiwa_range(min_x_list, max_x_list):
    to_iiwa=[]
    to_iiwa.append(min_x_list[0])
    for i in range(len(min_x_list)-1):
        to_iiwa.append((max_x_list[i]+min_x_list[i+1])/2)
    to_iiwa.append(min_x_list[-1])
    print('\nto iiwa:')
    print(to_iiwa)
    return to_iiwa

def re_set_length(x_wall, y_wall):
    img_size = 2/2 # m
    center = (min(x_wall)+max(x_wall))/2
    x_wall_limit, y_wall_limit = [], []
    for i in range(len(x_wall)):
        if center-img_size<x_wall[i]<center+img_size:
            x_wall_limit.append(x_wall[i])
            y_wall_limit.append(y_wall[i])
    print(f'{min(x_wall)}, {max(x_wall)}')
    print(f'center: {center}, from: {center-img_size}, to: {center+img_size}')
    return x_wall_limit, y_wall_limit

def run_algorithm(file_name = 'bee_hive'):
    
    input_wall = open_file(file_name, 'obj')
    # print(f'Opened file {file_name}')
    y_wall, x_wall = plot_wall_draw(input_wall)
    # x_wall, y_wall = re_set_length(x_wall, y_wall)

    x = generate_interval(x_wall)
    y = generate_interval(y_wall)
    wall = Wall(x, y)

    C = Candidate( wall, input_wall)
    print('Candidates generated')
    steps = greedy_cover_iiwa(wall, C)

    min_x_list, max_x_list, path_x, path_y, path_angle = to_gazebo_cmd_format(steps)

    iiwa_range_list = to_iiwa_range(min_x_list, max_x_list)
    print()
    # print(min_x_list)
    # print(max_x_list)
    
    print(iiwa_range_list)
    print(path_angle)

    plt.plot(x, y, color="grey")
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()
    return path_angle, iiwa_range_list, path_x, path_y

if __name__ == "__main__":
    run_algorithm()