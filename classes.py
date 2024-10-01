import sympy as sym
import numpy as np
from numpy.linalg import inv
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox, Button, Slider


bar_colour = "#8e8e8e"
scatter_gradient = "plasma"

initial_omega = 1
target_arm_length = 8
node_library = {"a": [0,0],
                "b": [2,2.073],
                "c": [1.174,-1.521],
                "d": [2.824,1.351]}

class Joint:
    def __init__(self, name, position, type, v):
        self.name = name
        self.position = position
        self.type = type
        self.v = v

    def properties(self):
        print(f"{self.name}: ({self.position}), {self.type} joint, velocity is {self.v} m/s")

    def relative_position(target, origin):
        final = np.array(target.position) - np.array(origin.position)
        return final
    
    def known_velocity(self, bar, second_node):
        initial_velocity = second_node.v
        omega = np.array([0,0,bar.w])
        relative = Joint.relative_position(self, second_node)
        velocity = initial_velocity + np.squeeze(np.cross(omega, relative))
        self.v = velocity
        return velocity


class Bar:
    def __init__(self, name, nodes, length, angle, w, input):
        self.name = name
        self.nodes = nodes
        self.length = length
        self.angle = angle
        self.w = w
        self.input = input

    def properties(self):
        print(f"{self.name}: length of {self.length}, angle of {self.angle} degrees, angular velocity is {self.w} rad/s")

    # complete data function?? to fill in unknown fields??


def find_scaled_point(origin, reference, target, distance):
    point_A = np.array(origin.position)
    point_B = np.array(reference.position)
    vector_AB = point_B - point_A
    magnitude = np.linalg.norm(vector_AB)
    multiplier = distance / magnitude
    target.position = list(point_A + multiplier * vector_AB)
    return target.position

node_A = Joint("a", node_library["a"], "pin", 0)
node_B = Joint("b", node_library["b"], "free", "unknown")
node_C = Joint("c", node_library["c"], "pin", 0)
node_D = Joint("d", node_library["d"], "free", "unknown")

target_node = Joint("target", "unknown", "free", "unknown")

bar_AB = Bar("ab", [node_A, node_B], math.dist(node_A.position, node_B.position), "unknown", initial_omega, True)
bar_CD = Bar("cd", [node_C, node_D], math.dist(node_C.position, node_D.position), "unknown", "unknown", False)
bar_BD = Bar("bd", [node_B, node_D], math.dist(node_B.position, node_D.position), "unknown", "unknown", False)

target_initial = find_scaled_point(node_B, node_D, target_node, target_arm_length)

nodes = [node_A, node_B, node_C, node_D]
bars = [bar_AB, bar_BD, bar_CD]
moving_nodes = [node_B, node_D] # could automate this down the line

def reset_positions(list):
    for node in list:
        node.position = node_library[node.name]

def determine_input(list):
    input_bar = 0
    for position, bar in enumerate(list):
        input_bar += position * bar.input
    return list[input_bar]

def calculate_values():
    
    V_b = Joint.known_velocity(node_B, bar_AB, node_A)
    print(V_b)

    D_rel_B = Joint.relative_position(node_D, node_B)
    D_rel_C = Joint.relative_position(node_D, node_C)

    coefficients = np.array([[D_rel_B[1], -D_rel_C[1]],
                            [-D_rel_B[0], D_rel_C[0]]])

    inverse = inv(coefficients)

    solution = np.array([V_b[0], V_b[1]])

    bar_BD.w, bar_CD.w = np.matmul(inverse, solution)

    V_d = Joint.known_velocity(node_D, bar_CD, node_C)

    # print(node_D.v)
    # print(Joint.known_velocity(node_D, bar_BD, node_B))

def update_position(node_list, timestep):
    for node in node_list:
        displacement = timestep * np.delete(node.v, 2)
        new_position = node.position + displacement
        node.position = new_position
        print(node.position)

def determine_path(target_node, target_node_2, timestep = 0.001, scale_limit = 1.01):
    initial_lengths = [bar_AB.length, bar_BD.length, bar_CD.length]
    anticlockwise = True
    complete = False
    counter = 0

    x_points = []
    y_points = []

    x_points_2 = []
    y_points_2 = []

    while anticlockwise == True:
        find_scaled_point(node_B, node_D, target_node, target_arm_length)
        x_points.append(target_node.position[0])
        y_points.append(target_node.position[1])
        if target_node_2 != False:
            x_points_2.append(target_node_2.position[0])
            y_points_2.append(target_node_2.position[1])
        calculate_values()
        update_position(moving_nodes, timestep)
        if math.dist(node_A.position, node_B.position) / initial_lengths[0] > scale_limit:
            anticlockwise = False
        elif math.dist(node_B.position, node_D.position) / initial_lengths[1] > scale_limit:
            anticlockwise = False
        elif math.dist(node_C.position, node_D.position) / initial_lengths[2] > scale_limit:
            anticlockwise = False
        counter += 1
        print(counter)
        print("\n")

    reset_positions(nodes)
    bar_AB.w = bar_AB.w * -1

    while complete ==  False:
        find_scaled_point(node_B, node_D, target_node, target_arm_length)
        x_points.append(target_node.position[0])
        y_points.append(target_node.position[1])
        if target_node_2 != False:
            x_points_2.append(target_node_2.position[0])
            y_points_2.append(target_node_2.position[1])
        calculate_values()
        update_position(moving_nodes, timestep)
        if math.dist(node_A.position, node_B.position) / initial_lengths[0] > scale_limit:
            complete = True
        elif math.dist(node_B.position, node_D.position) / initial_lengths[1] > scale_limit:
            complete = True
        elif math.dist(node_C.position, node_D.position) / initial_lengths[2] > scale_limit:
            complete = True
        counter += 1
        print(counter)
        print("\n")
    
    return x_points, y_points, x_points_2, y_points_2
    
node_x, node_y, node_2_x, node_2_y = determine_path(target_node, node_B, 0.001, 1.01)

plt.scatter(node_x, node_y, marker = ".", s = 2, c = node_x, cmap = scatter_gradient)

if len(node_2_x) == len(node_x):
    plt.scatter(node_2_x, node_2_y, marker = ".", s = 1, c = node_x, cmap = scatter_gradient)

plt.grid(True)

def plot_link(bar):
    node_1 = node_library[bar.nodes[0].name]
    node_2 = node_library[bar.nodes[1].name]
    plt.plot([node_1[0], node_2[0]],
             [node_1[1], node_2[1]],
             color = bar_colour,
             linewidth = 2,
             solid_capstyle = "round")

plot_link(bar_AB)
# plot_link(bar_BD)
plot_link(bar_CD)

plt.plot([node_library["b"][0], target_initial[0]],
         [node_library["b"][1], target_initial[1]],
         color = bar_colour,
         linewidth = 2,
         solid_capstyle = "round")

for key in node_library:
    value = node_library[key]
    label = f"{key}: {value}"
    plt.annotate(label, value, textcoords = "offset points", xytext = (10,0))

plt.title("Four Bar Linkage Simulator")

# ADD IN MATHS TO CALCULATE MAXIMUM ACHIEVABLE DEFORMATION
# BOTH DEFOMRATION (ABSOLUTE MAX & PURELY HORIZONTAL MAX)
# NEED TO CREATE ARRAYS WITH ANGLES AS FIRST COLUMN
# PRESENT DEFORMATION AS PERCENTAGE (FOR FAIR TESTING)
# COULD POTENTIALLY USE MDOE TO DETERMINE WHERE LINE IS FLAT

# add slider to change length of target rod
# add slider to change position of the linkage
# add ability to change node coordinates
# speed indicator? (change size of dots)
# add indicator for overlapping of links

plt.show()

