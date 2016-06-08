# mdp implementation needs to go here
from astar import *
from itertools import *
import random

class MapOracle(object):
    def __init__(self, refMap, r_w, r_s, r_g, r_p):
        self.map = refMap
        self.r_wall = r_w
        self.r_step = r_s
        self.r_goal = r_g
        self.r_pit  = r_p
        print "Deterministic MapOracle initialized."

    def getReward(self, state, action, landedState):
        if state in self.map.pit_locs:
            # Absorbing state so reward needs to be 0 or neg.
            return 0

        if state == landedState and (not state.isPit): #Hit a wall
            return self.r_wall

        # Pit case.
        if state == landedState and state.isPit and landedState.isPit:
            print "This actually shouldnt be happening (pit condition)"
            return (self.r_pit)
        elif landedState.isPit:
            return (self.r_pit + self.r_step)

        # Goal case.
        if self.map.isGoalState(landedState):
            return (self.r_goal + self.r_step)

        return self.r_step

    def getStates(self):
        states = []
        for row, col in product(xrange(0, self.map.map_height), xrange(0, self.map.map_width)):
            s = State(Point(row, col))
            if (not (s in self.map.wall_locs)) and (not (s in self.map.pit_locs)):
                states.append(s)
        return states

    def neighborExpansionAdaptor(self, state):
        return self.__neighborExpansionAdaptor(state)

    def __neighborExpansionAdaptor(self, state):
        neighbors, arrivalMethods = self.map.expandNodeNeighbors(state)
        full = []
        for x, arrivalMethod in zip(neighbors, arrivalMethods):
            res = None
            if arrivalMethod == Path.ARRIVAL_METHOD_LEFT:
                res = AStarSearch.ACTION_LEFT
            if arrivalMethod == Path.ARRIVAL_METHOD_RIGHT:
                res = AStarSearch.ACTION_RIGHT
            if arrivalMethod == Path.ARRIVAL_METHOD_UP:
                res = AStarSearch.ACTION_UP
            if arrivalMethod == Path.ARRIVAL_METHOD_DOWN:
                res = AStarSearch.ACTION_DOWN

            # Did we bump into a wall? If so, we should end up at our own state.
            if x.isWall:
                x = State(Point(state.loc.row, state.loc.col))
            if state.isPit:
                x = State(Point(state.loc.row, state.loc.col))
                continue
            if self.map.isGoalState(state):
                continue

            full.append( (x, res) )
        return full

    def getPossibleActions(self, state):
        # Really dirty adapter where we actually have to advance each state and test whether we end up
        # in a NONE state or a valid state and then return corresponding good actions. Ew.
        good_actions = []
        for x, arrivalMethod in self.__neighborExpansionAdaptor(state):
            if x is not None:
                good_actions.append(arrivalMethod)
        # Note that these actions are GLOBALLY-JUSTIFIED to the board, not to the current dir of the bot
        return good_actions

    def makeMove(self, state, action):
        newState = self.map.advanceState(state, action)

        if newState.isWall:
            newState = State(Point(state.loc.row, state.loc.col))
        if state.isPit:
            newState = State(Point(state.loc.row, state.loc.col))

        return newState


class UncertainMapOracle(MapOracle):
    def __init__(self, refMap, r_w, r_s, r_g, r_p, p_f, p_b, p_l, p_r):
        super(UncertainMapOracle, self).__init__(refMap, r_w, r_s, r_g, r_p)
        self.p_forward = p_f
        self.p_backward = p_b
        self.p_left = p_l
        self.p_right = p_r
        print "Uncertain MapOracle initialized."

    def makeMove(self, state, action):
        move_bindings = {
            AStarSearch.ACTION_LEFT: self.p_forward,
            AStarSearch.ACTION_RIGHT: self.p_backward,
            AStarSearch.ACTION_DOWN: self.p_left,
            AStarSearch.ACTION_UP: self.p_right
        }

        if action == AStarSearch.ACTION_LEFT:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_forward,
                AStarSearch.ACTION_RIGHT: self.p_backward,
                AStarSearch.ACTION_DOWN: self.p_left,
                AStarSearch.ACTION_UP: self.p_right
            }
        elif action == AStarSearch.ACTION_RIGHT:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_backward,
                AStarSearch.ACTION_RIGHT: self.p_forward,
                AStarSearch.ACTION_DOWN: self.p_right,
                AStarSearch.ACTION_UP: self.p_left
            }
        elif action == AStarSearch.ACTION_UP:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_left,
                AStarSearch.ACTION_RIGHT: self.p_right,
                AStarSearch.ACTION_DOWN: self.p_backward,
                AStarSearch.ACTION_UP: self.p_forward
            }
        elif action == AStarSearch.ACTION_DOWN:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_right,
                AStarSearch.ACTION_RIGHT: self.p_left,
                AStarSearch.ACTION_DOWN: self.p_forward,
                AStarSearch.ACTION_UP: self.p_backward
            }
        else:
            print "This should have never happened."

        tsum = 0
        rnum = random.random()
        actual_action = action
        for pick in move_bindings.keys():
            if move_bindings[pick] <= 0.0:
                continue
            tsum += move_bindings[pick]
            if rnum <= tsum:
                #print "Changing actual action from " + str(action) + " to " + str(actual_action)
                actual_action = pick
                break

        newState = self.map.advanceState(state, actual_action)
        if newState.isWall:
            newState = State(Point(state.loc.row, state.loc.col))
        if state.isPit:
            newState = State(Point(state.loc.row, state.loc.col))

        return newState


class MDPOracle(object):
    def __init__(self, mapOracle, p_f, p_b, p_l, p_r):
        self.map_oracle = mapOracle
        self.p_forward = p_f
        self.p_backward = p_b
        self.p_left = p_l
        self.p_right = p_r

    def getPossibleNeighbors(self, state, action):
        # Now this is where we have to take into account probabilities and whatnot and the
        # relative direction of where we want to go which is action

        move_bindings = {
            AStarSearch.ACTION_LEFT: self.p_forward,
            AStarSearch.ACTION_RIGHT: self.p_backward,
            AStarSearch.ACTION_DOWN: self.p_left,
            AStarSearch.ACTION_UP: self.p_right
        }

        if action == AStarSearch.ACTION_LEFT:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_forward,
                AStarSearch.ACTION_RIGHT: self.p_backward,
                AStarSearch.ACTION_DOWN: self.p_left,
                AStarSearch.ACTION_UP: self.p_right
            }
        elif action == AStarSearch.ACTION_RIGHT:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_backward,
                AStarSearch.ACTION_RIGHT: self.p_forward,
                AStarSearch.ACTION_DOWN: self.p_right,
                AStarSearch.ACTION_UP: self.p_left
            }
        elif action == AStarSearch.ACTION_UP:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_left,
                AStarSearch.ACTION_RIGHT: self.p_right,
                AStarSearch.ACTION_DOWN: self.p_backward,
                AStarSearch.ACTION_UP: self.p_forward
            }
        elif action == AStarSearch.ACTION_DOWN:
            move_bindings = {
                AStarSearch.ACTION_LEFT: self.p_right,
                AStarSearch.ACTION_RIGHT: self.p_left,
                AStarSearch.ACTION_DOWN: self.p_forward,
                AStarSearch.ACTION_UP: self.p_backward
            }
        else:
            print "This should have never happened."

        res = []
        for n, method in self.map_oracle.neighborExpansionAdaptor(state):
            res.append( (n, move_bindings[method]) )
        return res

class QLearner(object):
    def __init__(self, mapOracle, discount, learning_rate, epsilon):
        self.map_oracle = mapOracle
        self.discount = discount
        self.qvalues = {}
        self.alpha = learning_rate
        self.epsilon = epsilon
        self.currentState = None
        print "Q-Learning controller initialized."

    def single_iteration(self):
        if self.currentState is None: # First round iter
            #print "Picking initial state at random."
            # NOTE: This starting state *may* end up being a goal state. Make sure that doesn't happen.
            tempState = random.sample(self.map_oracle.getStates(), 1)[0]
            while self.map_oracle.map.isGoalState(tempState):
                tempState = random.sample(self.map_oracle.getStates(), 1)[0]
            self.currentState = tempState
            #print "\tPicked " + str(self.currentState)

        #print "Currently at state " + str(self.currentState)

        if self.map_oracle.map.isGoalState(self.currentState) or self.currentState.isPit:
            self.currentState = None
            #print "Current state is goal state or Pit, end episode."
            return False # CANNOT continue.

        action = self.getNextAction(self.currentState)
        #print "\tAction to be taken: " + str(action)

        #newState = self.map_oracle.map.advanceState(self.currentState, action)
        newState = self.map_oracle.makeMove(self.currentState, action)
        #print "\tEnded in state: " + str(newState)
        # Decide what reward we got.
        reward = self.map_oracle.getReward(self.currentState, action, newState)

        # Update the learner.
        self.update(self.currentState, action, newState, reward)

        self.currentState = newState
        return True

    def update(self, state, action, nextState, reward):
        # Main eq:
        # Q(s,a) = Q(s,a) + alpha*[R(s) + gamma*max_a{Q(s',a')} - Q(s,a)]

        # Pick the best Q(s',a') pair.============================================

        ns_list = map(lambda a: self.qvalues[(nextState, a)] if self.qvalues.has_key((nextState, a)) else 0, self.map_oracle.getPossibleActions(nextState))

        ns_qval = max(ns_list) if len(ns_list) > 0 else 0.0

        old_qval = self.qvalues[ (state, action) ] if self.qvalues.has_key( (state, action) ) else 0

        self.qvalues[ (state, action) ] = old_qval + self.alpha*(reward + self.discount*ns_qval - old_qval)

    def getOptimalAction(self, state):
        action_pairs = map(lambda a: tuple((self.qvalues[(state, a)] if self.qvalues.has_key((state,a)) else 0, a)), self.map_oracle.getPossibleActions(state))

        max_action = max(action_pairs)[1] if len(action_pairs) > 0 else None
        return max_action

    def getNextAction(self, state):
        legalActions = self.map_oracle.getPossibleActions(state)
        if len(legalActions) <= 0:
            return None

        if random.random() < self.epsilon:
            #print "Picking random choice."
            return random.choice(legalActions)
        return self.getOptimalAction(state)

    def gatherPolicies(self):
        arr = [[0 for x in xrange(0, self.map_oracle.map.map_width)] for y in xrange(0, self.map_oracle.map.map_height)]
        for row, col in product(xrange(0, self.map_oracle.map.map_height), xrange(0, self.map_oracle.map.map_width)):
            s = State(Point(row, col))
            arr[row][col] = self.getOptimalAction(s)
        return arr

class FeatureExtractor(object):
    def __init__(self, map_oracle):
        self.mapRef = map_oracle

    def getFeatures(self, state, action):
        features = {}
        # F1: 1 if action will take robot into a pit
        # F2: 1 if action will take robot into a wall
        # F3: 1 if action will take robot into goal

        # At least this weight is always here.
        features["default"] = 1.0
        features["hit_pit"] = 0.0
        features["hit_wall"] = 0.0
        features["hit_goal"] = 0.0

        #newState = self.mapRef.makeMove(state, action)
        newState = self.mapRef.map.advanceState(state, action)

        if newState.isPit:
            features["hit_pit"] = 1.0
        if newState.isWall:
            features["hit_wall"] = 1.0
        if self.mapRef.map.isGoalState(newState):
            features["hit_goal"] = 1.0

        # Calculate closeness to the goal.
        goal_heuristic = self.mapRef.map.calcHeuristic(newState)
        features["goal_closeness"] = float(goal_heuristic)/(self.mapRef.map.map_width * self.mapRef.map.map_height)

        # Don't think I need normalization for this.

        return features


class ApproximateQLearner(QLearner):
    def __init__(self, mapOracle, discount, learning_rate, epsilon, featureExtractor):
        super(ApproximateQLearner, self).__init__(mapOracle, discount, learning_rate, epsilon)
        self.featureExtractor = featureExtractor
        self.weights = {"default": 1.0, "hit_pit": random.random(), "hit_wall": random.random(), "hit_goal":random.random(), "goal_closeness": random.random()}
        print "Approximate Q-Learning module initialized."
       

    def __computeQValue(self, state, action):
        weighted_sum = 0
        features = self.featureExtractor.getFeatures(state, action)
        for feature in features:
            weighted_sum += self.weights[feature]*features[feature]
        return weighted_sum

    def update(self, state, action, nextState, reward):
        ns_list = map(lambda a: self.__computeQValue(nextState, a), self.map_oracle.getPossibleActions(nextState))

        ns_qval = max(ns_list) if len(ns_list) > 0 else 0.0

        old_qval = self.__computeQValue(state, action)

        features = self.featureExtractor.getFeatures(state, action)

        delta = (reward + self.discount*ns_qval) - old_qval

        for weight in self.weights:
            self.weights[weight] = self.weights[weight] + self.alpha*delta*features[weight]

        #self.printWeights()

    def printWeights(self):
        print "WEIGHTS: {",
        for w in self.weights:
            print str(w) + " -> " + str(self.weights[w]) + ", ",
        print "}\n"

    def getOptimalAction(self, state):
        action_pairs = map(lambda a: tuple((self.__computeQValue(state, a), a)), self.map_oracle.getPossibleActions(state))

        max_action = max(action_pairs)[1] if len(action_pairs) > 0 else None
        return max_action


class ValueIteration(object):
    def __init__(self, mdp, discount, max_iters = 1000, threshold = None):
        self.mdp = mdp
        self.discount = discount
        self.qvalues = {}
        self.values = {}
        self.default_iterations = max_iters
        self.compare_threshold = threshold
        self.current_delta = None
        self.can_stop = False
        print "Value Iteration controller initialized."

    def single_iteration(self):
        # First, copy the old list so we dont have conflicts
        oldqvals = self.qvalues.copy()
        delta = 0
        # Calc for every state
        for state in self.mdp.map_oracle.getStates():
            # For every possible action from this state...
            arglist = []
            for action in self.mdp.map_oracle.getPossibleActions(state):
                # So now we have S, a....need S' ?
                q = 0
                for neighbor, prob in self.mdp.getPossibleNeighbors(state, action):
                    # S, a, S'
                    # Need: Transition prob, reward, old val
                    reward = self.mdp.map_oracle.getReward(state, action, neighbor)
                    old_val = self.values[neighbor] if self.values.has_key(neighbor) else 0

                    # Calc...
                    q += prob*(reward + (self.discount*old_val))
                arglist.append(q)
                # TODO: This line may not work lol.
                self.qvalues[ (state, action) ] = q

            # Get the max action with biggest qval (s, A) and make that the value
            # for state S
            d = 0
            if self.values.has_key(state):
                d += self.values[state]
            self.values[state] = max(arglist) if len(arglist) > 0 else 0
            d = (d - self.values[state])
            delta += abs(d)

        if self.compare_threshold is not None and (abs(delta) < self.compare_threshold):
            print "Delta at : " + str(abs(delta)) + " compared to thresh " + str(self.compare_threshold)
            self.can_stop = True

    def gatherPolicies(self):
        arr = [[0 for x in xrange(0, self.mdp.map_oracle.map.map_width)] for y in xrange(0, self.mdp.map_oracle.map.map_height)]
        for row, col in product(xrange(0, self.mdp.map_oracle.map.map_height), xrange(0, self.mdp.map_oracle.map.map_width)):
            s = State(Point(row, col))
            res = []
            for action in self.mdp.map_oracle.getPossibleActions(s):
                if self.qvalues.has_key( (s, action) ):
                    res.append( (action, self.qvalues[ (s, action) ]) )
            # Find max for this state
            max_action = max(res, key=lambda x: x[1])[0] if len(res) > 0 else -1
            arr[row][col] = max_action
        return arr
