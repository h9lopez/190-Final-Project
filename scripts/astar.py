import itertools
import Queue
# astar implementation needs to go here

class Path(object):
    ARRIVAL_METHOD_LEFT = 1
    ARRIVAL_METHOD_RIGHT = 2
    ARRIVAL_METHOD_UP = 3
    ARRIVAL_METHOD_DOWN = 4
    ARRIVAL_METHOD_ROOT = 0

    def __init__(self, initialPath = []):
        self.path = []

        self.path += initialPath

    def appendNode(self, node, arrivalMethod):
        self.path.append( (node, arrivalMethod) )

    def __len__(self):
        return len(self.path)

    def clearPath(self):
        self.path = []

    def getTerminalNode(self):
        if len(self.path) <= 0:
            return -1

        return self.path[len(self.path)-1]

    def getRawList(self):
        return self.path

    def __str__(self):
        res = ""
        for state, method in self.path:
            res += str(state)
        return res

class State(object):
    def __init__(self, point):
        # Init
        self.loc = point
        self.isWall = False
        self.isPit = False

    @staticmethod
    def parseMultiple(points):
        res = map(lambda p: State(p), points)
        return res

    def __eq__(self, state2):
        if isinstance(state2, list):
            for x in state2:
                if self.loc == x.loc:
                    return True
            return False
        elif isinstance(state2, State):
            return self.loc == state2.loc
        super.__eq__(state2)

    def __ne__(self, state2):
        return not (self == state2)

    def __hash__(self):
        return hash( (self.loc.row, self.loc.col) )

    def __str__(self):
        return "[" + str(self.loc.row) + "," + str(self.loc.col) + "]"

class Point(object):
    def __init__(self, row, col):
        self.row = row
        self.col = col

    def __eq__(self, p2):
        return (self.row == p2.row) and (self.col == p2.col)

    @staticmethod
    def parseSingle(single):
        return Point(single[0], single[1])

    @staticmethod
    def parseList(listOfPoints):
        res = map(lambda x: Point.parseSingle(x), listOfPoints)
        return res

class Map(object):
    def __init__(self, iWallLoc = [], iPitLoc = [], iGoalLoc = [], iMapWidth = 0, iMapHeight = 0):
        self.wall_locs = []
        self.pit_locs = []
        self.goal_locs = []
        self.map_width = iMapWidth
        self.map_height = iMapHeight

        self.wall_locs += iWallLoc
        self.pit_locs += iPitLoc
        self.goal_locs += iGoalLoc

    def calcHeuristic(self, state):
        # Just calc manhattan distance.
        if len(self.goal_locs) == 0:
            print "No goal states registered."
            return 0
        goal_state = self.goal_locs[0]
        return (abs(state.loc.row - goal_state.loc.row) + abs(state.loc.col - goal_state.loc.col))

    def isGoalState(self, state):
        return (state in self.goal_locs)

    def advanceState(self, state, action):
        newState = State( Point(state.loc.row, state.loc.col) )

        if action == AStarSearch.ACTION_LEFT:
            # Hit wall?
            newState.loc.col -= 1
        elif action == AStarSearch.ACTION_RIGHT:
            newState.loc.col += 1
        elif action == AStarSearch.ACTION_UP:
            newState.loc.row -= 1
        elif action == AStarSearch.ACTION_DOWN:
            newState.loc.row += 1

        # Check if its in the explicit wall locations list
        if newState in self.wall_locs:
            newState.isWall = True
        # Check if it implicitly hit a wall. (i.e. goes past outer edges)
        if (newState.loc.row >= self.map_height) or (newState.loc.row < 0) or \
           (newState.loc.col >= self.map_width)  or (newState.loc.col < 0):
           newState.isWall = True

        # Check if we land in a pit
        if newState in self.pit_locs:
            newState.isPit = True

        # Otherwise, we good.
        return newState


    def expandNodeNeighbors(self, state):
        # Return neighbors that are not walls or pits, with their appropriate arrival methods
        neighbors = [
            self.advanceState(state, AStarSearch.ACTION_LEFT),
            self.advanceState(state, AStarSearch.ACTION_RIGHT),
            self.advanceState(state, AStarSearch.ACTION_UP),
            self.advanceState(state, AStarSearch.ACTION_DOWN)
        ]

        arrivalMethods = [Path.ARRIVAL_METHOD_LEFT,
                          Path.ARRIVAL_METHOD_RIGHT,
                          Path.ARRIVAL_METHOD_UP,
                          Path.ARRIVAL_METHOD_DOWN]

        return (neighbors, arrivalMethods)

class AStarSearch(object):
    ACTION_LEFT = 1
    ACTION_RIGHT = 2
    ACTION_UP = 3
    ACTION_DOWN = 4

    @staticmethod
    def astar(startState, refMap):
        # Visited hashtable
        visited = {}
        counter = itertools.count()

        initPath = Path()
        initPath.appendNode(startState, Path.ARRIVAL_METHOD_ROOT)

        # Mark start node as Visited
        visited[ str(startState) ] = True

        # Create frontier and add start node to it
        frontier = Queue.PriorityQueue()
        frontier.put( (0, next(counter), initPath) )

        while frontier.empty() == False:
            # Select node from frontier
            targetPriority, targetCounter, targetPath = frontier.get()
            targetNode, targetArrivalMethod = targetPath.getTerminalNode()

            # Check if node is goal state
            if refMap.isGoalState(targetNode):
                return targetPath
            else:
                neighbors, arrivalMethods = \
                    refMap.expandNodeNeighbors(targetNode)

                for neighbor, arrivalMethod in zip(neighbors, arrivalMethods):
                    if neighbor.isWall or neighbor.isPit:
                        continue
                    if visited.has_key( str(neighbor) ) == False:
                        totalCost = len(targetPath) + refMap.calcHeuristic(neighbor)
                        frontier.put( (totalCost, next(counter), Path(targetPath.getRawList() + [(neighbor, arrivalMethod)]) ) )
                        visited[ str(neighbor) ] = True

        return None
