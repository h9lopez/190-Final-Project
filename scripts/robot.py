#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from read_config import read_config
from cse_190_assi_3.msg import AStarPath, PolicyList
from astar import *
from mdp import *
import itertools

class Robot():
    def __init__(self):
        rospy.init_node("robot")
        self.config = ConfigAccess()
        self.map = Map([self.config.walls], [self.config.pits],
                       [self.config.goalState], self.config.map_width,
                       self.config.map_height)
        self.map_oracle = None
        self.mdp_oracle = None
        self.val_iter = None
        self.q_learner = None
        self.featureExtractor = None

        if self.config.isMovementUncertain == False:
            print "[ROBOT] Initializing with deterministic movement"
            self.map_oracle = MapOracle(self.map, self.config.r_wall,
                                        self.config.r_step, self.config.r_goal, self.config.r_pit)
        else:
            print "[ROBOT] Initializing with uncertain movement"
            self.map_oracle = UncertainMapOracle(self.map, self.config.r_wall,
                                        self.config.r_step, self.config.r_goal, self.config.r_pit, 
                                        self.config.p_forward, self.config.p_backward, 
                                        self.config.p_left, self.config.p_right)


        if self.config.controller == ConfigAccess.CONTROLLER_VALUE_ITERATION:
            print "[ROBOT] Value iteration controller picked."
            self.mdp_oracle = MDPOracle(self.map_oracle, self.config.p_forward, self.config.p_backward,
                                       self.config.p_left, self.config.p_right)

            self.val_iter = ValueIteration(self.mdp_oracle, self.config.discount_factor,
                                           self.config.max_iterations, self.config.threshold_diff)
        elif self.config.controller == ConfigAccess.CONTROLLER_Q_LEARNER:
            print "[ROBOT] Q Learning controller picked."
            self.q_learner = QLearner(self.map_oracle, self.config.discount_factor, self.config.alpha,
                                        self.config.epsilon)
        elif self.config.controller == ConfigAccess.CONTROLLER_APPROX_Q_LEARNER:
            print "[ROBOT] Approximate Q Learning controller picked."
            self.featureExtractor = FeatureExtractor(self.map_oracle)
            self.q_learner = ApproximateQLearner(self.map_oracle, self.config.discount_factor, 
                                                 self.config.alpha, self.config.epsilon,
                                                 self.featureExtractor)

        self.initConnections()

        rospy.sleep(5)
        print "Starting to do stuff..."

        if self.config.controller == ConfigAccess.CONTROLLER_VALUE_ITERATION:
            self.doMain()
        else:
            self.doLearningMain()

    def doLearningMain(self):
        print "Runnning learning loop instead of offline value iteration"
        print "EPISODES COMPLETED ",
        for x in xrange(0, self.config.max_iterations):
            while True:
                res = self.q_learner.single_iteration()
                if not res:
                    self.publishIterationPolicies(self.q_learner.gatherPolicies())
                    break
            print str(x) + "...",
            sys.stdout.flush()

        print "\n" + str(self.config.max_iterations) + " complete. Exiting..."
        self.doShutdown()

    def doMain(self):
        # Do A Star first
        respath = self.doAStar()
        self.publishPath(respath)

        for x in xrange(0, self.config.max_iterations):
            self.doMDPIteration()
            self.publishIterationPolicies(self.val_iter.gatherPolicies())
            print "Round " + str(x) + "/" + str(self.config.max_iterations) + " done"
            if self.val_iter.can_stop:
                print "Threshold reached! Stopping..."
                break

        self.doShutdown()

    def doShutdown(self):
        msg = Bool(True)
        self.sim_complete_pub.publish(msg)
        rospy.sleep(5)
        rospy.signal_shutdown("rip")

    def initConnections(self):
        # Setup publishers and subscribers for future computations
        self.astar_res_pub = rospy.Publisher(
            "/results/path_list",
            AStarPath,
            queue_size=20
        )

        self.mdp_path_pub = rospy.Publisher(
            "/results/policy_list",
            PolicyList,
            queue_size=20
        )

        self.sim_complete_pub = rospy.Publisher(
            "/map_node/sim_complete",
            Bool,
            queue_size=2
        )

    def doMDPIteration(self):
        self.val_iter.single_iteration()

    def publishIterationPolicies(self, policies):
        # Translate this from the number states into strings
        for row in xrange(0, len(policies)):
            for col in xrange(0, len(policies[row])):
                s = State(Point(row, col))
                if (s in self.map.wall_locs) or (s.loc.row >= self.map.map_height) \
                           or (s.loc.row < 0) or (s.loc.col >= self.map.map_width) \
                           or (s.loc.col < 0):
                    policies[row][col] = "WALL"

                elif s in self.map.pit_locs:
                    policies[row][col] = "PIT"
                elif self.map.isGoalState(s):
                    policies[row][col] = "GOAL"
                elif policies[row][col] == AStarSearch.ACTION_LEFT:
                    policies[row][col] = "W"
                elif policies[row][col] == AStarSearch.ACTION_RIGHT:
                    policies[row][col] = "E"
                elif policies[row][col] == AStarSearch.ACTION_UP:
                    policies[row][col] = "N"
                elif policies[row][col] == AStarSearch.ACTION_DOWN:
                    policies[row][col] = "S"

                if policies[row][col] == -1 or policies[row][col] == None:
                    print "Bad stuff happened at (" + str(row) + ", " + str(col) + ")"

        # Flatten the array
        flattened = itertools.chain.from_iterable(policies)
        #print "FLAT: " + str(list(flattened))
        msg = PolicyList()
        msg.data = list(flattened)
        self.mdp_path_pub.publish(msg)


    def doAStar(self):
        path = AStarSearch.astar(self.config.startState, self.map)
        return path

    def publishPath(self, path):
        for state, arrivalMethod in path.getRawList():
            msg = AStarPath()
            msg.data = [state.loc.row, state.loc.col]
            self.astar_res_pub.publish(msg)


class ConfigAccess(object):
    CONTROLLER_VALUE_ITERATION = 0
    CONTROLLER_Q_LEARNER = 1
    CONTROLLER_APPROX_Q_LEARNER = 2
    def __init__(self):
        self.config = read_config()

        self.walls = State.parseMultiple(Point.parseList(self.config["walls"]))
        self.pits = State.parseMultiple(Point.parseList(self.config["pits"]))
        self.map_height, self.map_width = self.config["map_size"]
        self.goalState = State(Point.parseSingle(self.config["goal"]))
        self.startState = State(Point.parseSingle(self.config["start"]))
        self.max_iterations = self.config["max_iterations"]
        self.threshold_diff = self.config["threshold_difference"]
        self.r_step = self.config["reward_for_each_step"]
        self.r_wall = self.config["reward_for_hitting_wall"]
        self.r_goal = self.config["reward_for_reaching_goal"]
        self.r_pit = self.config["reward_for_falling_in_pit"]
        self.discount_factor = self.config["discount_factor"]
        self.alpha = self.config["learning_rate"]
        self.epsilon = self.config["epsilon"]
        self.p_forward = self.config["prob_move_forward"]
        self.p_backward = self.config["prob_move_backward"]
        self.p_left = self.config["prob_move_left"]
        self.p_right = self.config["prob_move_right"]

        self.controller = self.CONTROLLER_VALUE_ITERATION
        controller_choice = self.config["controller"].strip()
        if controller_choice == "ValueIteration":
            self.controller = self.CONTROLLER_VALUE_ITERATION
        elif controller_choice == "QLearner":
            self.controller = self.CONTROLLER_Q_LEARNER
        elif controller_choice == "ApproxQLearner":
            self.controller = self.CONTROLLER_APPROX_Q_LEARNER

        self.isMovementUncertain = self.config["uncertain_movement"]

if __name__ == "__main__":
    Robot()
