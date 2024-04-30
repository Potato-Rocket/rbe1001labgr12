# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       oscar                                                        #
# 	Created:      3/27/2024, 10:12:23 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
import math

# W1---L1---L3---L5---L7---W4
#       |    |    |    |
#      B2   T2   T5    |
#       |    |    |    |
#      B1   T1   T4    |
#       |    |    |    |
#      B0   T0   T3    |
#       |    |    |    |
# W0---L0---L2---L4---L6---W3

# Node coordinates
NODE_X = 0, 41, 164, 227, 331, 371
"""Horizontal coordinates for line intersections."""
NODE_Y = 28, 221
"""Vertical coordinates for line intersections."""
START_NODE = "W0"
"""Node to start the robot on."""
START_ORIENTATION = 0
"""The absolute direction to start the robot facing."""
PICK_TREES = "T0", "T3", "T1", "T4", "T2", "T5"
"""The order in which to pick trees."""

# Mathematical
MAX = 1e6
"""Arbitrarily large distance for use in pathfinding."""

# Constants for turning
KP_TURN = 3
"""Proportional control constant for slowing in a turn."""
STOP_TURN = 1
"""Stopping threshold, in degrees, for a turn."""

# Constants for driving
KP_STRAIGHT = 2
"""Propotional control constant for slowing when driving."""
STOP_STRAIGHT = 1
"""Stopping threshold, in mm, for driving."""
KP_LINE = 0.006
"""Line following proportional control constant."""
NODE_THRESHOLD = 115
"""Total reflectivity threshold for identifying a node."""

# Constants for the rack
KP_RACK = 0.3
"""Proportional control constant for slowing when moving the rack."""
STOP_RACK = 1
"""Stopping threshold, in degrees, for moving the rack."""

# Constants for grabbing
KP_CENTERING = -0.5
"""Turning the robot ato face the fruit."""
KP_RAISING = 0
"""Raising the tower to the level of the fruit."""
KP_CLOSING = 0.2
"""Moving toward the fruit."""
X_GOAL = 160
"""Desired x coordinate of fruit."""
Y_GOAL = 100
"""Desired y coordinate of fruit."""
H_GOAL = 180
"""Desired height of fruit."""
H_CUTOFF = 10
"""Minimum visible fruit height."""
LIN_OFFSET = 50
"""Distance to drive forward after centering fruit."""
HEIGHT_OFFSET = 3
"""Distance to lower tower after centering fruit."""

# Other constants
CHASSIS_MIN_EFFORT = 5
"""Minimum effort to be supplied to motors."""
MAX_EFFORT_STRAIGHT = 50
"""Maximum effort to be supplied to motors."""
MAX_EFFORT_TURN = 50
"""Maximum effort to be supplied to motors."""
RACK_MIN_EFFORT = 10
"""Mimimum effort to be supplied to the motors."""
GEAR_RATIO = 5
"""Gear ration (speed ratio) between motor and wheels."""

# Dimensions
LINE_TO_TURN = 230
"""Distance in mm between line sensors and turning center."""
SONAR_TO_TURN = 140
"""Distance in mm between sonar sensors and turning center."""
WHEEL_DIA = 105
"""Wheel diameter in mm."""
LINE_WIDTH = 55
"""Thickness of the tape lines."""
HEIGHT = 2610
"""The entire range of the rack, in motor degrees."""

# Grab routine thresholds
TREE_THRESHOLD = 350
"""Maximum distance at which to register a tree."""
BASKET_TO_VISION = 500
"""The distance from which to view the bucket."""


def sign(num: float) -> int:
    """Determines the sign of a number.

    Parameters
    ----------
    num : float
        The number to check.

    
    Returns
    -------
    int
        The sign of the number.
    """

    if num == 0:
        return 0
    elif num < 0:
        return -1
    else:
        return 1


def fix_degrees(degrees: float) -> float:
    """Adjusts degree values that wrap around between 0 and 360 to range from -180 to 180. Output will be an equivalent angle to the input.

    Parameters
    ----------
    degrees: float
        The input in degrees.
    
    Returns
    -------
    float
        The equivalent output in degrees.
    """

    if degrees > 180:
        return fix_degrees(degrees - 360)
    elif degrees < -180:
        return fix_degrees(degrees + 360)
    else:
        return degrees


class Point:
    """A class representing a point in 2D space.

    Attributes
    ----------
    x : float
        The X position of the point.
    y : float
        The Y position of the point.
    
    Methods
    -------
    add(point)
        Creates a point representing the vector addition of this point's position with another point's position.
    subtract(point):
        Creates a point representing the vector subtraction of another point's position from this point's position.
    scale(factor):
        Creates a point representing the multiplication of this point's vector position with a scalar.
    distance(point):
        Calculates the scalar distance between this point and another point.
    angle(point):
        Calculates the angle above the positive X-axis of a vector pointing from this point to another point.
    """

    def __init__(self, x: float, y: float) -> None:
        """Constructs the point with a given X and Y position.

        Parameters
        ----------
        x : float
            The X position of this point.
        y : float
            THe Y position of this point.
        """

        self.x = x
        self.y = y
    

    def add(self, point: "Point") -> "Point":
        """Creates a point representing the vector addition of this point's position with another point's position.

        Parameters
        ----------
        point : Point
            The other point.
        
        Returns
        -------
        Point
            The vector result of the operation as a point.
        """

        return Point(self.x + point.x, self.y + point.y)

    def subtract(self, point: "Point") -> "Point":
        """Creates a point representing the vector subtraction of another point's position from this point's position.

        Parameters
        ----------
        point : Point
            The other point.
        
        Returns
        -------
        Point
            The vector result of the operation as a point.
        """

        return Point(self.x - point.x, self.y - point.y)
    
    def scale(self, factor: float) -> "Point":
        """Creates a point representing the multiplication of this point's vector position with a scalar.

        Parameters
        ----------
        factor : float
            The factor by which to scale the point.
        
        Returns
        -------
        Point
            The vector result of the operation as a point.
        """

        return Point(self.x * factor, self.y * factor)

    def distance(self, point: "Point") -> float:
        """Calculates the scalar, Euclidian distance between this point and another point.

        Parameters
        ----------
        point : Point
            The other point.
        
        Returns
        -------
        float
            The scalar, Euclidian distance between both points.
        """

        diff = point.subtract(self)
        return math.sqrt(diff.x**2 + diff.y**2)

    def angle(self, point: "Point") -> float:
        """Calculates the angle above the positive X-axis of a vector pointing from this point to another point.

        Parameters
        ----------
        point : Point
            The other point.
        
        Returns
        -------
        float
            The angle to the other point in degrees.
        """

        diff = point.subtract(self)
        return math.degrees(math.atan2(diff.y, diff.x))
    
    def __str__(self) -> str:
        return "({}, {})".format(self.x, self.y)


class Node:
    """A class representing a node in a graph.

    Stores the node's name, as well as the node's connections to other nodes.

    Attributes
    ----------
    name : any
        The name of the node, a unique identifier.
    edges : dict
        The connections between this node and other nodes. A dict where the keys are other `Node` objects, and the values are the weights of the connections.

    Methods
    -------
    attach(node, weight):
        Creates a connection between this node and another node.
    attach_all(nodes, weights):
        Creates a connection between this node and other nodes.
    detach(node):
        Removes the connection between this node and another node.
    detach_all():
        Removes all connections between this node and other nodes.
    """

    # stores the total number of nodes that have been created
    _counter = 0

    def __init__(self, name: str="") -> None:
        """Constructs the node with a given position as its name.
        
        Since a node is constructed without any connections to other nodes, `edges` is initiated as an empty dict.

        Parameters
        ----------
        name : str
            The name of the node, a unique identifier. Defaults to a counter if not given.
        """
        
        if name == "":
            self.name = "Node" + str(Node._counter)
            Node._counter += 1
        else:
            self.name = name
        self.edges = {}

    def attach(self, node: "Node", weight: float) -> None:
        """Creates a connection between this node and another node.

        Adds this node to the other node's edges, and adds the other node to this node's edges.
        
        Parameters
        ----------
        node : Node
            The other node to create a connection to.
        weight : float
            The weight of the connection, a positive numerical value.
        """

        self.edges.update({node : float(abs(weight))})
        node.edges.update({self : float(abs(weight))})
    
    def attach_all(self, nodes: list["Node"], weights: list[float]) -> None:
        """Creates a connection between this node and other nodes.
        
        Parameters
        ----------
        nodes : list[Node]
            The other nodes to create connections to. Should be a list of `Node` objects.
        weights : list[float]
            The weights of the connections. Should be a list of positive numerical values, the same length as `nodes`.
        """

        for i in range(len(nodes)):
            self.attach(nodes[i], weights[i])

    def detach(self, node: "Node") -> None:
        """Removes the connection between this node and another node.

        Removes this node from the other node's edges, and removes the other node from this node's edges.
        
        Parameters
        ----------
        node : Node
            The other node to remove the connection to.
        """
        
        self.edges.pop(node)
        node.edges.pop(self)

    def detach_all(self) -> None:
        """Removes all connections between this node and other nodes.

        Removes this node from the edges of each node this node is connected to, then clears this node's edges.
        """

        for node in self.edges:
            node.edges.pop(self)
        self.edges.clear()

    def __str__(self) -> str:
        return "Node {} with neighbors {}".format(self.name, r"{" + ", ".join([str(node.name) + ": " + str(dist) for node, dist in self.edges.items()]) + r"}")


class Node2D(Node):
    """A class representing a node in a graph where the node represents a point in 2D space.
    
    Stores the node's position in addition to its name, as well as the node's connections to other nodes.

    Attributes
    ----------
    pos : Point
        The position of the node in 2D space.

    Methods
    -------
    update():
        Changes the position of this node to a new point.
    angle():
        Calculates the angle from this node ot another node.
    attach(node):
        Creates a connection between this node and another node.
    attach_all(nodes):
        Creates a connection between this node and other nodes.
    """

    def __init__(self, name: str, pos: Point) -> None:
        """Constructs the node with a given position and a name.
        
        Since a node is constructed without any connections to other nodes, `edges` is initiated as an empty dict.

        Parameters
        ----------
        name : str
            The name of the node, a unique identifier. Defaults to a counter if not given.
        pos : Point
            The position of the node in 2D space.
        """

        super().__init__(name)
        self.pos = pos
    
    def update(self, pos: Point) -> None:
        """Changes the position of this node to a new point.
        
        Updates all edge weights based on new distances.
        
        Parameters
        ----------
        pos : Point
            The new position of the node in 2D space.
        """

        self.pos = pos
        self.attach_all(list(self.edges.keys()))
    
    def angle(self, node: "Node2D") -> float:
        """Calculates the angle above the positive X-axis of a vector pointing from this node to another node.

        Parameters
        ----------
        node : Node2D
            The other node.
        
        Returns
        -------
        float
            The angle to the other node in degrees.
        """
        return self.pos.angle(node.pos)

    def attach(self, node: "Node2D") -> None:
        """Creates a connection between this node and another node.

        Calculates the distance between this node and the other node using the pythagorean theorem. Adds this node to the other node's edges, and adds the other node to this node's edges.
        
        Parameters
        ----------
        node : Node
            The other node to create a connection to.
        """
        
        # calls the usual attach function with the distance as the weight
        super().attach(node, self.pos.distance(node.pos))

    def attach_all(self, nodes: list["Node2D"]) -> None:
        """Creates a connection between this node and other nodes.
        
        Parameters
        ----------
        nodes : list[Node2D]
            The other nodes to create connections to. Should be a list of `Node` objects.
        """

        for node in nodes:
            self.attach(node)

    def __str__(self) -> str:
        return "Node {} at {} with neighbors {}".format(self.name, self.pos, r"{" + ", ".join([str(node.name) + ": " + str(dist) for node, dist in self.edges.items()]) + r"}")
    

class Tree:
    """A class representing a tree.
    
    Stores information about the tree and how to pick from it.

    Attributes
    ----------
    color : Signature
        The vision color signature the robot should use to detect the tree's fruit.
    height : float
        The height of the tree, as a percentage of the tower height.
    fruit : int
        The number of fruit remaining on the tree.

    Methods
    -------
    pick():
        Remove one fruit from the tree.
    """

    def __init__(self, color: Signature, height: float) -> None:
        """Constructs a new tree object and initializes atributes.

        Perameters
        ----------
        color : Signature
            The vision color signature to use when picking.
        height : float
            The height of the tree, as a percentage of the tower height.
        fruit : int
            The number of fruit remaining on the tree.
        """

        self.color = color
        self.height = height
        self.fruit = 2
    
    def pick(self) -> None:
        """Remove one fruit from the tree.
        
        Ensures the fruit count is no less than zero.
        """

        if self.fruit > 0:
            self.fruit -= 1
        

class TreeAccess(Node2D):
    """A class representing a tree access point as a 2D node.
    
    Stores the node's position, its name, and its connections to other nodes. Further keeps track of which color signature to use on each side, the number of remaining fruit on each side, and the tree height on each side.

    Methods
    -------
    pick():
        Select a tree to pick from and return information about how to pick from it.
    has_fruit():
        Whether either tree has fruit remaining.
    """

    def __init__(self, name: str, pos: Point, left: Tree, right: Tree) -> None:
        """Constructs the node with a given position, name, and trees it can access.
        
        Since a node is constructed without any connections to other nodes, `edges` is initiated as an empty dict.

        Parameters
        ----------
        name : str
            The name of the node, a unique identifier. Defaults to a counter if not given.
        pos : Point
            The position of the node in 2D space.
        left : Tree
            The tree on the left of this node.
        right : Tree
            The tree on the right of this node.
        """

        super().__init__(name, pos)
        self._left_tree = left
        self._right_tree = right
    
    def pick(self) -> tuple[float, Signature, float]:
        """Select a tree to pick from and return information about how to pick from it.
        
        Selects the tree with more fruit remaining. If both have the same number of fruit, picks the left tree.

        Returns
        -------
        float
            The absolute direction the robot should be facing to pick from the selected tree.
        Signature
            The vision signature the the robot should use to detect the tree's fruit.
        float
            The height of the tree, as a percentage of the tower height.
        """

        if self._right_tree.fruit > self._left_tree.fruit:
            angle = 180
            tree = self._right_tree
        else:
            angle = 0
            tree = self._left_tree
        tree.pick()
        return angle, tree.color, tree.height

    def has_fruit(self) -> bool:
        """Returns true if either tree has fruit remaining."""

        return self._left_tree.fruit + self._right_tree.fruit > 0

    def __str__(self) -> str:
        return "Tree {} at {} with neighbors {}".format(self.name, self.pos, r"{" + ", ".join([str(node.name) + ": " + str(dist) for node, dist in self.edges.items()]) + r"}")


class BasketAccess(Node2D):
    """A class representing a basket access point as a 2D node.
    
    Stores the node's position, its name, and its connections to other nodes. Further keeps track of the color signatures corresponding to the types of fruit that can be put in the basket.

    Attributes
    ----------
    colors : list[Signature, ...]
        List of applicable color signatures.
    """

    def __init__(self, name: str, pos: Point) -> None:
        """Constructs the node with a given position, name, and trees it can access.
        
        Since a node is constructed without any connections to other nodes, `edges` is initiated as an empty dict. The list of color signatures is also initiated as an empty list.

        Parameters
        ----------
        name : str
            The name of the node, a unique identifier. Defaults to a counter if not given.
        pos : Point
            The position of the node in 2D space.
        """

        super().__init__(name, pos)
        self.colors = []

    def __str__(self) -> str:
        return "Basket {} at {} with neighbors {}".format(self.name, self.pos, r"{" + ", ".join([str(node.name) + ": " + str(dist) for node, dist in self.edges.items()]) + r"}")


class Dijkstra:
    """A class to solve a graph for the shortest path to any node given a source node.
     
    Uses Dijkstra's shortest path algorithm, which is simple and will provide a complete solution provided the graph is constructed correctle.
    
    Methods
    -------
    get_name_path(target):
        Returns the list of node names for the path from the source node to a target node.
    """

    def __init__(self, graph: set[Node], source: Node) -> None:
        """Constructs internal path and distance tables given a graph and a source node.
        
        Parameters
        ----------
        graph : dict[str, Node]
            The graph, a dict where the keys are the node names, and the values are the corresponding `Node` objects.
        source : Any
            The name of the source node.
        """

        # ensures the source node is included in the graph and that there is
        # at least one node in the graph
        self._nodes = graph
        assert source in graph

        # initializes the distances and paths tables
        self._distances = {}
        self._paths = {}
        for node in self._nodes:
            self._distances.update({node: MAX})
            self._paths.update({node: []})
        self._distances[source] = 0
        self._paths[source].append(source)

        # creates a set of unvisited nodes and sets the current node
        unvisited = self._nodes.copy()
        current = source

        # repeat indefinitely
        while True:
            # for each node attached to the current node
            for node, dist in current.edges.items():
                # make sure it is unvisited
                if node not in unvisited:
                    continue
                # then if the distance through the current node is less than the node's marked distance
                total_dist = self._distances[current] + dist
                if total_dist < self._distances[node]:
                    # update the node's marked distance and path
                    self._distances[node] = total_dist
                    self._paths[node] = self._paths[current] + [node]
            
            # mark the current node as visited
            unvisited.remove(current)

            # visit the unvisited node with the smallest distance
            min_dist, current = self.__get_min_dist(unvisited)
            # if every path has been solved, excluding unreachable nodes
            if min_dist >= MAX:
                # exit the loop
                break
    
    def __get_min_dist(self, nodes: set[Node]) -> tuple[float,Node]:
        """Gets node in a set of nodes with the smallest marked distance.

        Returns no node and a distace of infinity if the set is empty or none of the nodes have been marked with a distance.
        
        Parameters
        ----------
        nodes : set[Node]
            The set of nodes to evaluate.
        
        Returns
        -------
        float
            The least marked distance.
        Node
            The node with the least marked distance.
        """

        min_dist = MAX
        current = list(self._nodes)[0]
        # iterate through nodes and set the shortest distance if it is smaller than the current minimum
        for node in nodes:
            dist = self._distances[node]
            if dist < min_dist:
                min_dist = dist
                current = node
        return min_dist, current
    
    def get_path(self, target: Node) -> list:
        """Returns the list of nodes for the shortest path from the source node to a target node.
        
        Parameters
        ----------
        target : Any
            The name of the targed node.
            
        Returns
        -------
        list
            The list of nodes for the path.
        """

        return self._paths[target]

    def __str__(self) -> str:
        string = "Source | End | Distance | Path"
        for node in self._nodes:
            path = self._paths[node]
            string += "\n{} | {} | {} | {}".format(
                path[0].name,
                path[-1].name,
                self._distances[node],
                " > ".join(n.name for n in path))
        
        return string


class NodeType:
    """An enumeration of the various types of nodes."""

    LINE = 0
    """Stops at the intersection of two lines."""
    WALL = 2
    """Stops when a wall is detected with the bumper."""
    TREE = 3
    """Stops when a tree is detected with the sonar sensors."""
    BASKET = 4
    """Stops based on encoder distance measurements."""


class TreeHeight:
    """An enumerastion of the possible tower heights."""

    SHORT = 25
    """The shortest height."""
    NORMAL = 50
    """The middle height."""
    TALL = 75
    """The tallest height."""

    TREE_HEIGHTS = ((NORMAL, NORMAL, TALL),
                    (TALL, SHORT, NORMAL),
                    (NORMAL, TALL, SHORT))
    """The heights of trees in the challenge map."""


class Fruit:
    """The vision signatures for different types of fruit."""

    LEMON = Signature(1, 2521, 2893, 2707, -3613, -3411, -3512, 2.5, 0)
    """The yellow fruit."""
    LIME = Signature(2, -5299, -4901, -5100, -3355, -3029, -3192, 4, 0)
    """The green fruit."""
    TANGERINE = Signature(3, 5269, 6699, 5984, -2719, -2463, -2591, 2.5, 0)
    """The orange fruit."""
    GRAPEFRUIT = Signature(4, 6063, 6895, 6479, 865, 1097, 981, 2.5, 0)
    """The pink fruit."""

    TREE_COLORS = TANGERINE, LEMON, LIME
    """The order of tree colors in the orchard."""

    LEMON_BASKET = Code(GRAPEFRUIT, LEMON)
    """The yellow and pink basket."""
    LIME_BASKET = Code(GRAPEFRUIT, LIME)
    """The green and pink basket."""
    TANGERINE_BASKET = Code(GRAPEFRUIT, TANGERINE)
    """The orange and pink basket."""

    FRUIT_TABLE = ((LEMON_BASKET, LEMON, "Lemon"),
                   (LIME_BASKET, LIME, "Lime"),
                   (TANGERINE_BASKET, TANGERINE, "Tangerine"))


class Orchard:

    def __init__(self, start: str) -> None:
        self._graph = {}
        self.baskets = []
        self.trees = []
        self.__make_graph()
    
        self._path = [self._graph[start]]
        self._targets = 0

    def __make_graph(self):
        prev_top = None
        prev_bottom = None
        count = {"W": 0, "L": 0, "T": 0, "B": 0}
        for x in range(len(NODE_X)):
            end = x == 0 or x == len(NODE_X) - 1
            name = "W" if end else "L"
            bottom = Node2D(name + str(count[name]),
                            Point(NODE_X[x], NODE_Y[0]))
            count[name] += 1
            top = Node2D(name + str(count[name]),
                         Point(NODE_X[x], NODE_Y[1]))
            count[name] += 1
            self._graph.update({top.name: top, bottom.name: bottom})
            if x > 0 and x < len(NODE_X) - 2:
                diff = top.pos.subtract(bottom.pos).scale(0.25)
                prev = bottom
                if x == 1:
                    name = "B"
                    for y in range(3):
                        node = BasketAccess(name + str(count[name]),
                                            prev.pos.add(diff))
                        count[name] += 1
                        node.attach(prev)
                        self._graph.update({node.name: node})
                        self.baskets.append(node)
                        prev = node
                else:
                    name = "T"
                    left_color = Fruit.TREE_COLORS[x - 2]
                    left_height = TreeHeight.TREE_HEIGHTS[x - 2]
                    right_color = Fruit.TREE_COLORS[x - 1]
                    right_height = TreeHeight.TREE_HEIGHTS[x - 1]
                    for y in range(3):
                        left = Tree(left_color, left_height[y])
                        right = Tree(right_color, right_height[y])
                        node = TreeAccess(name + str(count[name]),
                                          prev.pos.add(diff),
                                          left, right)
                        count[name] += 1
                        node.attach(prev)
                        self._graph.update({node.name: node})
                        self.trees.append(node)
                        prev = node
                prev.attach(top)
            if x == 4:
                top.attach(bottom)
            if not prev_top is None:
                top.attach(prev_top)
            if not prev_bottom is None:
                bottom.attach(prev_bottom)
            prev_top = top
            prev_bottom = bottom

    def set_target(self, target: str) -> None:
        dijkstra = Dijkstra(set(self._graph.values()), self._path[0])
        self._path = dijkstra.get_path(self._graph[target])

    def is_end(self, index=1) -> bool:
        return len(self._path) <= index
    
    def node_type(self, index=1) -> int:
        node = self._path[index]
        if node.name[0] == "L":
            return NodeType.LINE
        elif node.name[0] == "W":
            return NodeType.WALL
        elif node.name[0] == "T":
            return NodeType.TREE
        else:
            return NodeType.BASKET
    
    def stop_at_next(self) -> bool:
        if self.is_end(2):
            return True
        else:
            return not self.angle_to() == self.angle_to(2)
    
    def next_distance(self) -> float:
        return self._path[0].edges[self._path[1]] * 10
    
    def angle_to(self, index=1) -> float:
        if self.is_end(index):
            return 0
        return self._path[0].pos.angle(self._path[index].pos)
    
    def current(self) -> Node2D:
        return self._path[0]
    
    def target(self) -> Node2D:
        return self._path[-1]
    
    def get(self, name: str) -> Node2D:
        return self._graph[name]

    def move(self) -> None:
        self._path.pop(0)
        print(self._path[0])


class Chassis:
    
    # Creates a new chassis with the appropriate motors and sensors
    def __init__(self, brain: Brain) -> None:
        self.left_motor = Motor(Ports.PORT9, True)
        self.right_motor = Motor(Ports.PORT10)
        self.motors = MotorGroup(self.left_motor, self.right_motor)

        self.left_line = Line(brain.three_wire_port.e)
        self.right_line = Line(brain.three_wire_port.f)
        self.inertial = Inertial(Ports.PORT11)

        self.target = 0.0
        self.heading = 0.0

    # Finds the absolute heading, agnostic to resets
    def get_heading(self):
        return -fix_degrees(self.heading + self.inertial.heading())
    
    # Calibrates the IMU, and waits for it to complete its process
    def calibrate(self) -> None:
        self.inertial.calibrate()
        while self.inertial.is_calibrating():
            pass

    # Returns the position of the wheels in mm traveled, based on the motors' average encoder positions
    def get_position_mm(self) -> float:
        enc = (self.left_motor.position() + self.right_motor.position()) / 2
        return ((enc * math.pi * WHEEL_DIA) / (360 * GEAR_RATIO))

    # Whether both line sensors are entirely on a line, using a reflectivity threshold
    def on_node(self) -> bool:
        refl = self.left_line.reflectivity() + self.right_line.reflectivity()
        return refl > NODE_THRESHOLD

    # Returns the total desired effort, based on distance in mm left to travel
    # Limits the value to a given max_effort and a defined MIN_EFFORT
    def get_effort(self, scaled: float, max_effort: float) -> float:
        effort = (scaled + CHASSIS_MIN_EFFORT * sign(scaled))
        if abs(effort) > max_effort:
            effort = max_effort * sign(effort)
        return effort

    # Rotate the robot based on a given number of degrees remaining
    # Returns whether the turn should continue
    def rotate(self, remaining: float=1000) -> bool:
        # stop if the turn has been completed
        if abs(remaining) < STOP_TURN:
            self.halt()
            return False

        # scale the robot's effort based on how much of the turn is left
        effort = self.get_effort(remaining * KP_TURN, MAX_EFFORT_TURN)
        
        # spin the motors in opposite directions
        self.left_motor.spin(FORWARD, effort, PERCENT)
        self.right_motor.spin(REVERSE, effort, PERCENT)
        return True
    
    # Rotates the robot using the IMU to keep track of heading
    # Degrees remaining is based on set target
    def rotate_imu(self) -> bool:
        heading = fix_degrees(self.inertial.heading())
        remaining = self.target - heading
        return self.rotate(remaining)
    
    # Set a new rotation target, between 180 and -180 degrees
    # Resets current heading to 0, adding change to total heading variable
    def set_rot_target(self, angle: float) -> None:
        self.heading += self.inertial.heading()
        self.inertial.reset_heading()
        self.target = angle
    
    # Drive based on the distance in mm remaining to travel and max effort
    # Uses the reflectivity sensors to line follow
    # Returns whether the chassis should continue driving
    def line_follow(self, remaining: float=1000, slow: bool=True) -> bool:
        # stop if the trip has been completed
        if abs(remaining) < STOP_STRAIGHT:
            self.halt()
            return False
        
        if slow:
            effort = self.get_effort(remaining * KP_STRAIGHT,
                                     MAX_EFFORT_STRAIGHT)
        else:
            effort = MAX_EFFORT_STRAIGHT

        # find the distance from the desired wall following distance
        diff = (self.left_line.reflectivity() -
                self.right_line.reflectivity())
        diff = diff**2 * sign(diff) * KP_LINE

        # turn both motors based on a differential
        self.left_motor.spin(FORWARD, effort + diff, PERCENT)
        self.right_motor.spin(FORWARD, effort - diff, PERCENT)
        return True
    
    # Drive based on distance in mm remaining to travel and max effort
    # Returns whether the chassis should continue driving
    def drive_straight(self, remaining: float=1000, slow: bool=True) -> bool:
        if abs(remaining) < STOP_STRAIGHT:
            self.halt()
            return False
        
        if slow:
            effort = self.get_effort(remaining * KP_STRAIGHT,
                                     MAX_EFFORT_STRAIGHT)
        else:
            effort = MAX_EFFORT_STRAIGHT
        
        self.motors.spin(FORWARD, effort, PERCENT)
        return True

    # Applies either drive function, slowing based on encoder position
    # Distance remaining in mm is based on set target
    # Returns whether the chassis should continue driving
    def drive_enc(self, drive_func: Callable, slow: bool=True) -> bool:
        remaining = self.target - self.get_position_mm()
        return drive_func(remaining, slow)
    
    # Set a new distance target in mm
    def set_lin_target(self, distance: float) -> None:
        self.target = self.get_position_mm() + distance

    # Stop the motors
    def halt(self) -> None:
        self.motors.stop(HOLD)


class Tower:
    
    # creates a new tower with the appropriate motors and sensors
    def __init__(self, brain: Brain) -> None:
        self.rack = Motor(Ports.PORT1)
        self.limit = Limit(brain.three_wire_port.g)
        self.gripper = Motor(Ports.PORT3, True)

        self.limit.pressed(self.rack.reset_position)

        self._target = 0

    # Opens gripper until position is reset
    # Returns whether the gripper should continue opening
    def open_gripper(self) -> bool:
        if self.gripper.position() < 1:
            self.gripper.stop(HOLD)
            return False
        self.gripper.spin(REVERSE, 360, DPS)
        return True

    # Closes gripper until torque approaches limit
    # Returns whether the gripper should continue closing
    def close_gripper(self) -> bool:
        if self.gripper.torque() > 1:
            self.gripper.stop(HOLD)
            return False
        self.gripper.spin(FORWARD, 60, DPS)
        return True
    
    # Calibrates the tower using the limit switch
    # Gripper should start open
    def calibrate(self) -> None:
        self.rack.spin_for(FORWARD, 1, SECONDS, RACK_MIN_EFFORT, PERCENT)
        self.rack.set_position(HEIGHT + 360, DEGREES)

        self.rack.spin(REVERSE, RACK_MIN_EFFORT, PERCENT)
        while not self.rack.position() < 1:
            pass
        self.rack.stop(HOLD)

    # Sets a target for the tower, in percents of total height
    def set_target(self, position: float=0) -> None:
        self._target = position

    # Sets the position of the tower to a position between 0 and 100
    def move_to(self, max_effort: float=50) -> bool:
        target = HEIGHT * (self._target / 100)
        if target > HEIGHT:
            target = HEIGHT
        if target < 0:
            target = 0
        return self.move(target - self.rack.position(), max_effort)
    
    # Moves the tower based on degrees left to travel
    # Returns whether the tower should keep traveling
    def move(self, remaining: float=1000, max_effort: float=50) -> bool:
        top = HEIGHT - self.rack.position()
        bottom = -self.rack.position()
        if remaining > top:
            remaining = top
        if remaining < bottom:
            remaining = bottom
        
        if abs(remaining) < STOP_RACK:
            self.rack.stop(HOLD)
            return False
        
        effort = remaining * KP_RACK + RACK_MIN_EFFORT * sign(remaining)
        if abs(effort) > max_effort:
            effort = max_effort * sign(effort)

        self.rack.spin(FORWARD, effort, PERCENT)
        return True
    
    # Return the tower's position, between 0 and 100
    def get_position(self) -> float:
        return (self.rack.position() / HEIGHT) * 100

    # Stop the rack and gripper motors
    def halt(self) -> None:
        self.rack.stop(HOLD)
        self.gripper.stop(HOLD)


class Robot:

    def __init__(self) -> None:
        self.brain = Brain()

        self.chassis = Chassis(self.brain)
        self.tower = Tower(self.brain)
        self.orchard = Orchard(START_NODE)

        self.left_sonar = Sonar(self.brain.three_wire_port.a)
        self.right_sonar = Sonar(self.brain.three_wire_port.c)
        self.bumper = Bumper(self.brain.three_wire_port.h)

        self.vision_fruit = Vision(Ports.PORT2, 50,
                                   Fruit.LEMON,
                                   Fruit.LIME,
                                   Fruit.TANGERINE,
                                   Fruit.GRAPEFRUIT)
        self.vision_basket = Vision(Ports.PORT2, 50,
                                    Fruit.LEMON_BASKET,
                                    Fruit.LIME_BASKET,
                                    Fruit.TANGERINE_BASKET)

        self.orientation = START_ORIENTATION

        self.chassis.calibrate()
        self.tower.calibrate()

    def at_tree(self) -> bool:
        left = self.left_sonar.distance(MM)
        right = self.right_sonar.distance(MM)
        return min(left, right) < TREE_THRESHOLD
    
    def orient_toward(self, angle: float) -> None:
        diff = angle - self._orientation
        self._orientation = angle
        self.chassis.set_rot_target(-fix_degrees(diff))


class TravelState:
    IDLE = 0
    """Robot is stopped, currently waiting for an updated target."""
    TURN = 1
    """Robot is turning, using the IMU to maintain heading."""
    LINE = 2
    """Robot is following the line."""
    THRU_LINE = 3
    """Robot is traveling forward until it passes a line intersection."""
    THRU_TREE = 3
    """Robot is traveling forward until it passes a tree."""
    SLOW = 4
    """Robot is slowing as it approaches a known distance."""


class TravelStateMachine:

    def __init__(self) -> None:
        self._state = TravelState.IDLE
    
    def update(self, robot: Robot) -> None:
        if self._state == TravelState.IDLE:
            self.__handle_idle(robot)
        elif self._state == TravelState.TURN:
            self.__handle_turn(robot)
        elif self._state == TravelState.LINE:
            self.__handle_line(robot)
        elif self._state == TravelState.THRU_LINE:
            self.__handle_thru_line(robot)
        elif self._state == TravelState.THRU_TREE:
            self.__handle_thru_tree(robot)
        elif self._state == TravelState.SLOW:
            self.__handle_slow(robot)

    def __handle_idle(self, robot: Robot) -> None:
        if not robot.orchard.is_end():
            robot.orient_toward(robot.orchard.angle_to())
            self._state = TravelState.TURN
            print("turning toward next node")

    def __handle_turn(self, robot: Robot) -> None:
        if not robot.chassis.rotate_imu():
            if robot.orchard.node_type() == NodeType.BASKET:
                robot.chassis.set_lin_target(robot.orchard.next_distance())
                self._state = TravelState.SLOW
                print("slowing to stop")
            else:
                self._state = TravelState.LINE
                print("line following")

    def __handle_line(self, robot: Robot) -> None:
        robot.chassis.line_follow()
        
        if robot.orchard.node_type() == NodeType.WALL:
                if robot.bumper.pressing():
                    robot.chassis.set_lin_target(-LINE_WIDTH)
                    self._state = TravelState.SLOW
                    print("stopping at wall")
            
        elif robot.orchard.node_type() == NodeType.TREE:
            if robot.at_tree():
                if robot.orchard.stop_at_next():
                    robot.chassis.set_lin_target(SONAR_TO_TURN)
                    self._state = TravelState.SLOW
                    print("slowing to stop")
                else:
                    self._state = TravelState.THRU_TREE
                    print("driving past tree")

        elif robot.orchard.node_type() == NodeType.LINE:
            if robot.chassis.on_node():
                if robot.orchard.stop_at_next():
                    robot.chassis.set_lin_target(LINE_TO_TURN)
                    self._state = TravelState.SLOW
                    print("slowing to stop")
                else:
                    self._state = TravelState.THRU_LINE
                    print("driving through line intersection")

    def __handle_thru_line(self, robot: Robot) -> None:
        robot.chassis.drive_straight()
        
        if not robot.chassis.on_node():
            self._state = TravelState.IDLE
            robot.orchard.move()
            print("reached node")
    
    def __handle_thru_tree(self, robot: Robot) -> None:
        robot.chassis.line_follow()
        
        if not robot.at_tree():
            self._state = TravelState.IDLE
            robot.orchard.move()
            print("reached node")

    def __handle_slow(self, robot: Robot) -> None:
        if not robot.chassis.drive_enc(robot.chassis.line_follow,
                                        slow=robot.orchard.stop_at_next()):
            self._state = TravelState.IDLE
            robot.orchard.move()
            print("reached node")


class DepositState:
    TURN_AWAY = 0
    """Turns away from the basket by 90 degrees."""
    DRIVE_AWAY = 1
    """Drive away from the basket a given distance."""
    SCAN = 2
    """Runs a given action until completion."""
    DRIVE_TOWARD = 3
    """Drive toward the basket until hitting the bumper."""
    RELEASE = 4
    """Open the gripper."""
    RETURN = 5
    """Drives back to its initial position."""
    END = 6
    """Deposit sequence has been completed."""


class DepositStateMachine:

    def __init__(self) -> None:
        self._state = DepositState.END
        self._release = False
        self._init = 0
    
    def set_relase(self) -> None:
        self._release = True

    def start(self, robot: Robot):
        robot.orient_toward(0)
        self._state = DepositState.TURN_AWAY
        print("turning away from basket")

    def update(self, robot: Robot) -> bool:
        if self._state == DepositState.TURN_AWAY:
            self.__handle_turn_away(robot)
        elif self._state == DepositState.DRIVE_AWAY:
            self.__handle_drive_away(robot)
        elif self._state == DepositState.SCAN:
            self.__handle_scan(robot)
        elif self._state == DepositState.DRIVE_TOWARD:
            self.__handle_drive_toward(robot)
        elif self._state == DepositState.RELEASE:
            self.__handle_release(robot)
        elif self._state == DepositState.RETURN:
            self.__handle_return(robot)
        elif self._state == DepositState.END:
            return False
        return True
        
    def __handle_turn_away(self, robot: Robot) -> None:
        if not robot.chassis.rotate_imu():
            if self._release:
                self._init = robot.chassis.get_position_mm()
                self._state = DepositState.DRIVE_TOWARD
                print("driving toward basket")
            else:
                robot.chassis.set_lin_target(BASKET_TO_VISION)
                self._state = DepositState.DRIVE_AWAY
                print("driving away from basket")

    def __handle_drive_away(self, robot: Robot) -> None:
        if not robot.chassis.drive_enc(robot.chassis.drive_straight):
            self._state = DepositState.SCAN
            print("scanning basket")

    def __handle_scan(self, robot: Robot) -> None:
        basket = robot.orchard.current()
        assert isinstance(basket, BasketAccess)
        for code, fruit, name in Fruit.FRUIT_TABLE:
            snapshot = robot.vision_basket.take_snapshot(code)
            if snapshot:
                basket.colors.append(fruit)
                print(name)
        
        if len(basket.colors) > 0:
            robot.chassis.set_lin_target(-BASKET_TO_VISION)
            self._state = DepositState.RETURN
            print("driving back")

    def __handle_drive_toward(self, robot: Robot) -> None:
        robot.chassis.drive_straight(-1000)

        if robot.bumper.pressing():
            robot.chassis.halt()
            self._state = DepositState.RELEASE
            print("releasing fruit")

    def __handle_release(self, robot: Robot) -> None:
        if not robot.tower.close_gripper():
            robot.chassis.set_lin_target(self._init -
                                         robot.chassis.get_position_mm())
            self._state = DepositState.RETURN
            print("driving back")

    def __handle_return(self, robot: Robot) -> None:
        if not robot.chassis.drive_enc(robot.chassis.drive_straight):
            self._state = DepositState.END
            print("back on the line")


class GrabState:
    ORIENT = 0
    """Turns to face the tree, raises tower halfway."""
    BACKUP = 1
    """Backs away from the tree a given amount."""
    SEARCH = 2
    """Identifies the fruit."""
    CENTER = 3
    """Centers on and approaches the fruit."""
    APPROACH = 4
    """Drives toward the fruit a fixed amount."""
    GRAB = 5
    """Closes the gripper on the fruit."""
    RETURN = 6
    """Backs up the same distance it drove forward."""
    REORIENT = 7
    """Rotates until it finds the line, and update orientation."""
    END = 8
    """Robot has grabbed the fruit"""


class GrabStateMachine:

    def __init__(self) -> None:
        self._state = GrabState.END
        self._init_position = 0
        self._init_heading = 0
        self._search = 0
        self._signature = None

    def get_signature(self) -> Signature:
        assert self._signature
        return self._signature

    def start(self, robot: Robot) -> None:
        tree = robot.orchard.current()
        assert isinstance(tree, TreeAccess)
        dir, self._signature, self._init_height = tree.pick()
        robot.orient_toward(dir)
        robot.tower.set_target(50)
        self._init_heading = robot.chassis.get_heading()
        self.grab_state = GrabState.ORIENT
        print("turning to face tree and raising tower")

    def update(self, robot: Robot) -> bool:
        if self._state == GrabState.ORIENT:
            self.__handle_orient(robot)
        elif self._state == GrabState.BACKUP:
            self.__handle_backup(robot)
        elif self._state == GrabState.SEARCH:
            self.__handle_search(robot)
        elif self._state == GrabState.CENTER:
            self.__handle_center(robot)
        elif self._state == GrabState.APPROACH:
            self.__handle_approach(robot)
        elif self._state == GrabState.GRAB:
            self.__handle_grab(robot)
        elif self._state == GrabState.RETURN:
            self.__handle_return(robot)
        elif self._state == GrabState.REORIENT:
            self.__handle_reorient(robot)
        elif self._state == GrabState.END:
            return False
        return True

    def __handle_orient(self, robot: Robot) -> None:
        robot.tower.move_to()
        if not robot.chassis.rotate_imu():
            robot.chassis.set_lin_target(LINE_TO_TURN)
            self.grab_state = GrabState.BACKUP
            print("backing up from tree and raising tower")

    def __handle_backup(self, robot: Robot) -> None:
        if not (robot.chassis.rotate_imu() or robot.tower.move_to()):
            robot.chassis.set_rot_target(30)
            self._search = 60
            self.grab_state = GrabState.SEARCH
            print("searching for tree")

    def __handle_search(self, robot: Robot) -> None:
        snapshot = robot.vision_fruit.take_snapshot(self._signature)
        print(snapshot)
        if snapshot:
            robot.chassis.halt()
            self._init_position = robot.chassis.get_position_mm()
            self.grab_state = GrabState.CENTER
            print("centering on tree")
        
        if not robot.chassis.rotate_imu():
            self._search = -self._search
            robot.chassis.set_rot_target(self._search)

    def __handle_center(self, robot: Robot) -> None:
            print(robot.vision_fruit.installed())
            robot.vision_fruit.take_snapshot(self._signature)
            largest = robot.vision_fruit.largest_object()

            x_diff = X_GOAL - largest.centerX
            y_diff = Y_GOAL - largest.centerY
            h_diff = H_GOAL - largest.height
            print(x_diff, y_diff, h_diff)

            effort = robot.chassis.get_effort(h_diff * KP_CLOSING,
                                              MAX_EFFORT_TURN)
            diff = x_diff * KP_CENTERING
            robot.chassis.left_motor.spin(FORWARD, effort + diff, PERCENT)
            robot.chassis.right_motor.spin(FORWARD, effort - diff, PERCENT)

            robot.tower.move(y_diff * KP_RAISING)

            if h_diff < H_CUTOFF:
                robot.tower.set_target(robot.tower.get_position() -
                                       HEIGHT_OFFSET)
                robot.chassis.set_lin_target(-LIN_OFFSET)
                self.grab_state = GrabState.APPROACH
                print("making final approach")

    def __handle_approach(self, robot: Robot) -> None:
        if not (robot.chassis.drive_enc(robot.chassis.drive_straight) or
                robot.tower.move_to()):
            self.grab_state = GrabState.GRAB
            print("grabbing fruit")

    def __handle_grab(self, robot: Robot) -> None:
        if not robot.tower.close_gripper():
            robot.chassis.set_lin_target(self._init_position -
                                         robot.chassis.get_position_mm() +
                                         LINE_TO_TURN)
            self.grab_state = GrabState.RETURN
            print("returning to the line")

    def __handle_return(self, robot: Robot) -> None:
        if not robot.chassis.drive_enc(robot.chassis.drive_straight):
            robot.tower.set_target()
            robot.chassis.set_rot_target(
                fix_degrees(robot.chassis.get_heading() - self._init_heading))
            self.grab_state = GrabState.REORIENT
            print("turning back toward initial heading")

    def __handle_reorient(self, robot: Robot) -> None:
        if not (robot.chassis.rotate_imu() or robot.tower.move_to()):
            self.grab_state = GrabState.END
            print("back in initial position")


class MasterState:
    TRAVEL_SETUP = 0
    """Driving toward the next basket node to scan."""
    DEPOSIT_SETUP = 1
    """Scanning the current basket node."""
    TRAVEL_FRUIT = 2
    """Traversing the currently defined path to a fruit."""
    GRAB = 3
    """Locating and grabbing a fruit, and storing the color."""
    TRAVEL_BASKET = 4
    """Traversing the currently defined path to a basket."""
    DEPOSIT_FRUIT = 5
    """Depositing the current fruit in the correct basket."""


class MasterStateMachine:

    def __init__(self) -> None:
        self._travel_state = TravelStateMachine()
        self._deposit_state = DepositStateMachine()
        self._grab_state = GrabStateMachine()
        self._robot = Robot()

        self.__next_setup()

        self._pick_trees = list(PICK_TREES)
        self._unsetup = self._robot.orchard.baskets.copy()

    def update(self) -> None:
        if self._state == MasterState.TRAVEL_SETUP:
            self.__handle_travel_setup()
        elif self._state == MasterState.DEPOSIT_SETUP:
            self.__handle_deposit_setup()
        elif self._state == MasterState.TRAVEL_FRUIT:
            self.__handle_travel_fruit()
        elif self._state == MasterState.GRAB:
            self.__handle_grab()
        elif self._state == MasterState.TRAVEL_BASKET:
            self.__handle_travel_basket()
        elif self._state == MasterState.DEPOSIT_FRUIT:
            self.__handle_deposit_fruit()

    def __handle_travel_setup(self) -> None:
        self._travel_state.update(self._robot)
        if self._robot.orchard.is_end():
            self._state = MasterState.DEPOSIT_SETUP
            self._deposit_state.start(self._robot)
            
    def __handle_deposit_setup(self) -> None:
        if not self._deposit_state.update(self._robot):
            self.__next_setup()

    def __handle_travel_fruit(self) -> None:
        self._travel_state.update(self._robot)
        if self._robot.orchard.is_end():
            self._state = MasterState.GRAB
            self._grab_state.start(self._robot)

    def __handle_grab(self) -> None:
        if not self._grab_state.update(self._robot):
            signature = self._grab_state.get_signature()
            for basket in self._robot.orchard.baskets:
                if signature in basket.colors:
                    self._robot.orchard.set_target(basket.name)
                    self._state = MasterState.TRAVEL_BASKET
                    return
                    
    def __handle_travel_basket(self) -> None:
        self._travel_state.update(self._robot)
        if self._robot.orchard.is_end():
            self.__select_fruit()

    def __handle_deposit_fruit(self) -> None:
        if not self._deposit_state.update(self._robot):
            self.__select_fruit()

    def __select_fruit(self) -> None:
        if len(self._pick_trees) > 0:
            tree = self._robot.orchard.get(self._pick_trees[0])
            assert isinstance(tree, TreeAccess)
            if tree.has_fruit():
                self._robot.orchard.set_target(self._pick_trees[0])
                self._state = MasterState.TRAVEL_FRUIT
            else:
                self._pick_trees.pop(0)
                self.__select_fruit()
    
    def __next_setup(self) -> None:
            if len(self._unsetup) > 0:
                self._robot.orchard.set_target(self._unsetup.pop().name)
                self._state = MasterState.TRAVEL_SETUP
            else:
                self.__select_fruit()


state_machine = MasterStateMachine()

while True:
    state_machine.update()
