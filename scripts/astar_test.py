#!/usr/bin/env python
from astar import *
from read_config import read_config
# File to test implementation of astar before integrating into robot

if __name__ == "__main__":
    config = read_config()
    wall_loc = State.parseMultiple(Point.parseList(config["walls"]))
    print "Wall locs:"
    for wall in wall_loc:
        print "\t" + str(wall)
    pits = State.parseMultiple(Point.parseList(config["pits"]))
    print "Pits: "
    for pit in pits:
        print "\t" + str(pit)
    height, width = config["map_size"]
    print "Height: " + str(height) + ", Width: " + str(width)
    goal = State(Point.parseSingle(config["goal"]))
    print "Goal: " + str(goal)
    start = State(Point.parseSingle(config["start"]))
    print "Start: " + str(start)

    testMap = Map([wall_loc], [pits], [goal], width, height)
    # testMap = Map([], [], [State(Point(2,3))], width, height)
    path = AStarSearch.astar(start, testMap)
    print "Found path: "
    print str(path)
