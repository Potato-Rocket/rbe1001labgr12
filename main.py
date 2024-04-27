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
pick_trees = ["T0", "T3", "T1", "T4", "T2", "T5"]
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


class TravelState:
    """An enumeration of the different modes when driving."""

    IDLE = 0
    """Robot is stopped, currently waiting for an updated target."""
    TURN = 1
    """Robot is turning, using the IMU to maintain heading."""
    LINE = 2
    """Robot is following the line."""
    THRU = 3
    """Robot is traveling forward until it passes a node trigger."""
    SLOW = 4
    """Robot is slowing as it approaches a known distance."""


class DepositState:
    """An enumeration of the sequence to face a basket."""

    START = 0
    """Sets orientation target for next state."""
    TURN_AWAY = 1
    """Turns away from the basket by 90 degrees."""
    DRIVE_AWAY = 2
    """Drive away from the basket a given distance."""
    ACTION = 3
    """Runs a given action until completion."""
    DRIVE_BACK = 4
    """Drives back to its initial position."""
    END = 5
    """Deposit sequence has been completed."""


class SetupState:
    """An enumeration of the steps in the basket setup process."""

    START = 0
    """Robot is driving toward the correct line node."""
    TRAVEL = 1
    """Robot is driving toward next basket node to scan."""
    DEPOSIT = 2
    """Robot is turning away from the box."""
    END = 3
    """Robot has completed its scanning routine."""
 

class GrabState:
    """An enumeration of the steps in the grabbing sequence."""

    START = 0
    """Setting up the color signature and rotation goal."""
    ORIENT = 1
    """Turns to face the tree, raises tower halfway."""
    BACKUP = 2
    """Backs away from the tree a given amount."""
    SEARCH = 3
    """Identifies the fruit."""
    CENTER = 4
    """Centers on and approaches the fruit."""
    FORWARD = 5
    """Drives toward the fruit a fixed amount."""
    GRAB = 6
    """Closes the gripper on the fruit."""
    RETURN = 7
    """Backs up the same distance it drove forward."""
    REORIENT = 8
    """Rotates until it finds the line, and update orientation."""
    END = 9
    """Robot has grabbed the fruit"""


class Routine:
    """An enumeration of the different routines the robot follows."""

    SETUP = 0
    """Detecting and storing the locations and colors of the baskets."""
    TRAVEL_FRUIT = 1
    """Traversing the currently defined path to a fruit."""
    GRAB = 2
    """Locating and grabbing a fruit, and storing the color."""
    TRAVEL_BASKET = 3
    """Traversing the currently defined path to a basket."""
    DEPOSIT = 4
    """Depositing the current fruit in the correct basket."""


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
    
    def __init__(self, lm: Motor, rm: Motor, ll: Line, rl: Line, imu: Inertial) -> None:
        self.left_motor = lm
        self.right_motor = rm
        self.motors = MotorGroup(rm, lm)

        self.left_line = ll
        self.right_line = rl
        self.inertial = imu

        self.thru = self.on_node
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
        diff = ((self.left_line.reflectivity() -
                 self.right_line.reflectivity()))
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
    
    def __init__(self, rack: Motor, lim: Limit, grip: Motor) -> None:
        self.rack = rack
        self.limit = lim
        self.gripper = grip

        self._target = 0

        self.limit.pressed(self.rack.reset_position)

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

    def __init__(self, ch: Chassis, tw: Tower, vs: Vision, ls: Sonar, rs: Sonar, bm: Bumper, oc: Orchard, orientation: float=0) -> None:
        self.chassis = ch
        self.tower = tw
        self.left_sonar = ls
        self.right_sonar = rs
        self.bumper = bm
        self.vision = vs
        self.orchard = oc

        self.travel_state = TravelState.IDLE
        self.deposit_state = DepositState.END
        self.setup_state = SetupState.START
        self.grab_state = GrabState.END

        self._orientation = orientation

        self._unsetup = self.orchard.baskets.copy()

        self._init = 0
        self._init_heading = 0
        self._init_height = 0
        self._signature = None
        self._search = 60

    def select_fruit(self) -> None:
        if len(pick_trees) > 0:
            tree = self.orchard.get(pick_trees[0])
            assert isinstance(tree, TreeAccess)
            if tree.has_fruit():
                orchard.set_target(pick_trees[0])
            else:
                pick_trees.pop(0)
                self.select_fruit()

    def select_basket(self) -> None:
        for basket in self.orchard.baskets:
            if self._signature in basket.colors:
                orchard.set_target(basket.name)
                return

    def color_basket(self) -> bool:
        for code, fruit, name in Fruit.FRUIT_TABLE:
            snapshot = self.vision.take_snapshot(code)
            if snapshot:
                basket = self.orchard.current()
                assert isinstance(basket, BasketAccess)
                basket.colors.append(fruit)
                print(name)
        return False

    def at_tree(self) -> bool:
        left = self.left_sonar.distance(MM)
        right = self.right_sonar.distance(MM)
        return min(left, right) < TREE_THRESHOLD
    
    def orient_toward(self, angle: float) -> None:
        diff = angle - self._orientation
        self._orientation = angle
        self.chassis.set_rot_target(-fix_degrees(diff))
    
    def travel_line(self) -> None:
        if self.orchard.node_type() == NodeType.WALL:
                if self.bumper.pressing():
                    self.chassis.set_lin_target(-LINE_WIDTH)
                    self.travel_state = TravelState.SLOW
            
        elif orchard.node_type() == NodeType.TREE:
            if self.at_tree():
                if self.orchard.stop_at_next():
                    self.chassis.set_lin_target(SONAR_TO_TURN)
                    self.travel_state = TravelState.SLOW
                    print("slowing to stop")
                else:
                    self.chassis.thru = self.at_tree
                    self.travel_state = TravelState.THRU
                    print("driving through tree")

        elif orchard.node_type() == NodeType.LINE:
            if self.chassis.on_node():
                if self.orchard.stop_at_next():
                    self.chassis.set_lin_target(LINE_TO_TURN)
                    self.travel_state = TravelState.SLOW
                    print("slowing to stop")
                else:
                    self.chassis.thru = self.chassis.on_node
                    self.travel_state = TravelState.THRU
                    print("driving through line")
    
    def travel(self) -> None:
        if self.travel_state == TravelState.IDLE:
            if not self.orchard.is_end():
                self.orient_toward(self.orchard.angle_to())
                self.travel_state = TravelState.TURN
                print("turning")

        elif self.travel_state == TravelState.TURN:
            if not self.chassis.rotate_imu():
                if self.orchard.node_type() == NodeType.BASKET:
                    self.chassis.set_lin_target(self.orchard.next_distance())
                    self.travel_state = TravelState.SLOW
                    print("slowing to stop")
                else:
                    self.travel_state = TravelState.LINE
                    print("line following")

        elif self.travel_state == TravelState.LINE:
            self.chassis.line_follow()
            self.travel_line()

        elif self.travel_state == TravelState.THRU:
            if self.chassis.thru == self.at_tree:
                self.chassis.line_follow()
            else:
                self.chassis.drive_straight()
            if not self.chassis.thru():
                self.travel_state = TravelState.IDLE
                self.orchard.move()

        elif self.travel_state == TravelState.SLOW:
            if not self.chassis.drive_enc(chassis.line_follow,
                                          slow=self.orchard.stop_at_next()):
                self.travel_state = TravelState.IDLE
                self.orchard.move()

    def deposit(self, action, gripper: bool) -> None:
        if self.deposit_state == DepositState.START:
            self.orient_toward(0)
            self.deposit_state = DepositState.TURN_AWAY
            print("turning away")

        elif self.deposit_state == DepositState.TURN_AWAY:
            if not self.chassis.rotate_imu():
                if gripper:
                    self._init = self.chassis.get_position_mm()
                else:
                    self.chassis.set_lin_target(BASKET_TO_VISION)
                self.deposit_state = DepositState.DRIVE_AWAY
                print("driving away")
        
        elif self.deposit_state == DepositState.DRIVE_AWAY:
            if gripper:
                self.chassis.drive_straight(-1000)
                drive = self.bumper.pressing()
            else:
                drive = not self.chassis.drive_enc(self.chassis.drive_straight)
            if drive:
                self.chassis.halt()
                self.deposit_state = DepositState.ACTION
                print("executing action")
        
        elif self.deposit_state == DepositState.ACTION:
            if not action():
                if gripper:
                    self.chassis.set_lin_target(self._init -
                                                self.chassis.get_position_mm())
                else:
                    self.chassis.set_lin_target(-BASKET_TO_VISION)
                self.deposit_state = DepositState.DRIVE_BACK
                print("driving back")
        
        elif self.deposit_state == DepositState.DRIVE_BACK:
            if not self.chassis.drive_enc(self.chassis.drive_straight):
                self.deposit_state = DepositState.END
                print("complete")

    def next_basket(self) -> None:
        if len(self._unsetup) > 0:
            self.orchard.set_target(self._unsetup.pop().name)
            self.setup_state = SetupState.TRAVEL
        else:
            self.setup_state = SetupState.END
        
    def setup(self) -> None:
        if self.setup_state == SetupState.START:
            self.travel()
            if self.orchard.is_end() and self.orchard.current().name == "L1":
                self.next_basket()
                print("got to baskets")
            elif (self.orchard.target().name != "L1" and
                  self.travel_state == TravelState.IDLE):
                self.orchard.set_target("L1")
            
        elif self.setup_state == SetupState.TRAVEL:
            self.travel()
            if self.orchard.is_end():
                self.setup_state = SetupState.DEPOSIT
                self.deposit_state = DepositState.START
                print("got to node")

        elif self.setup_state == SetupState.DEPOSIT:
            self.deposit(self.color_basket, False)
            if self.deposit_state == DepositState.END:
                self.next_basket()
                print("found baasket colors")
    
    def grab(self) -> None:
        if self.grab_state == GrabState.START:
            tree = self.orchard.current()
            assert isinstance(tree, TreeAccess)
            dir, self._signature, self._init_height = tree.pick()
            self.orient_toward(dir)
            self.tower.set_target(50)
            self._init_heading = self.chassis.get_heading()
            self.grab_state = GrabState.ORIENT

        elif self.grab_state == GrabState.ORIENT:
            if ((not self.chassis.rotate_imu()) and
                (not self.tower.move_to())):
                self.chassis.set_rot_target(30)
                self._search = 60
                self.grab_state = GrabState.SEARCH

        elif self.grab_state == GrabState.SEARCH:
            if not self.chassis.rotate_imu():
                self._search = -self._search
                self.chassis.set_rot_target(self._search)
            snapshot = self.vision.take_snapshot(self._signature)
            print(snapshot)
            if snapshot:
                self.chassis.halt()
                self._init = self.chassis.get_position_mm()
                self.grab_state = GrabState.CENTER

        elif self.grab_state == GrabState.CENTER:
            print(vision.installed())
            self.vision.take_snapshot(self._signature)
            x_diff = X_GOAL - self.vision.largest_object().centerX
            y_diff = Y_GOAL - self.vision.largest_object().centerY
            h_diff = H_GOAL - self.vision.largest_object().height
            print(x_diff, y_diff, h_diff)

            self.chassis.rotate(x_diff * KP_CENTERING)
            self.tower.move(y_diff * KP_RAISING)
            self.chassis.drive_straight(-h_diff * KP_CLOSING)

            if h_diff < H_CUTOFF:
                self.tower.set_target(self.tower.get_position() - HEIGHT_OFFSET)
                self.chassis.set_lin_target(-LIN_OFFSET)
                self.grab_state = GrabState.FORWARD

        elif self.grab_state == GrabState.FORWARD:
            if ((not self.chassis.drive_straight()) and
                (not self.tower.move_to())):
                self.grab_state = GrabState.GRAB

        elif self.grab_state == GrabState.GRAB:
            if not self.tower.close_gripper():
                self.chassis.set_lin_target(-self._init)
                self.grab_state = GrabState.RETURN

        elif self.grab_state == GrabState.RETURN:
            if not self.chassis.drive_enc(self.chassis.drive_straight):
                self.grab_state = GrabState.REORIENT
                self.tower.set_target()
                self.chassis.set_rot_target(
                    fix_degrees(self.chassis.get_heading() -
                                self._init_heading))

        elif self.grab_state == GrabState.REORIENT:
            if ((not self.chassis.rotate_imu()) and
                (not self.tower.move_to())):
                self.grab_state = GrabState.END


brain = Brain()

chassis = Chassis(Motor(Ports.PORT9, True),
                  Motor(Ports.PORT10),
                  Line(brain.three_wire_port.e),
                  Line(brain.three_wire_port.f),
                  Inertial(Ports.PORT11))

tower = Tower(Motor(Ports.PORT1),
              Limit(brain.three_wire_port.g),
              Motor(Ports.PORT3, True))

vision = Vision(Ports.PORT2, 50,
                Fruit.LEMON,
                Fruit.LEMON_BASKET,
                Fruit.LIME,
                Fruit.LIME_BASKET,
                Fruit.TANGERINE,
                Fruit.TANGERINE_BASKET,
                Fruit.GRAPEFRUIT)

orchard = Orchard(START_NODE)

robot = Robot(chassis,
              tower,
              vision,
              Sonar(brain.three_wire_port.a),
              Sonar(brain.three_wire_port.c),
              Bumper(brain.three_wire_port.h),
              orchard, START_ORIENTATION)

chassis.calibrate()
tower.calibrate()

routine = Routine.SETUP

while True:
    if routine == Routine.SETUP:
        robot.setup()
        if robot.setup_state == SetupState.END:
            robot.select_fruit()
            routine = Routine.TRAVEL_FRUIT

    elif routine == Routine.TRAVEL_FRUIT:
        robot.travel()
        if orchard.is_end():
            routine = Routine.GRAB
            robot.grab_state = GrabState.START

    elif routine == Routine.GRAB:
        robot.grab()
        if robot.grab_state == GrabState.END:
            robot.select_basket()
            routine = Routine.TRAVEL_BASKET
    
    elif routine == Routine.TRAVEL_BASKET:
        robot.travel()
        if orchard.is_end():
            routine = Routine.DEPOSIT
    
    elif routine == Routine.DEPOSIT:
        robot.deposit(tower.open_gripper, True)
        if robot.deposit_state == DepositState.END:
            robot.select_fruit()
            routine = Routine.TRAVEL_FRUIT
