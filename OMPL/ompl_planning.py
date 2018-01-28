'''
Created on 11.04.2017

@author: mario
'''

from maps import GridMap
from ompl import base as ob
from ompl import geometric as og

mGridMap = GridMap("4_1_map.png")
startpos = (140,200)
goalpos = (725,1095)

def isStateValid(state):
    # "state" is of type SE2StateInternal, so we don't need to use the "()"
    # operator.
    #
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    global mGridMap
    posx = min(int(state[0]),mGridMap.dimx)
    posy = min(int(state[1]),mGridMap.dimy)
    pos = (posx,posy)
    valid = not mGridMap.checkcol_pos(pos)
    return valid

def plan():
    # create an discrete state space
    global startpos, goalpos, mGridMap
    xdim = mGridMap.dimx
    ydim = mGridMap.dimy
    xspace = ob.DiscreteStateSpace(0,xdim)
    yspace = ob.DiscreteStateSpace(0,ydim)
    space = ob.CompoundStateSpace()
    space.addSubspace(xspace)
    space.addSubspace(yspace)
    # set lower and upper bounds
    #bounds = ob.RealVectorBounds(2)
    #bounds.setLow(-1)
    #bounds.setHigh(1)
    #space.setBounds(bounds)
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    start = ob.State(space)
    # we can pick a random start state...
    start.setX(startpos[0])
    start.setY(startpos[1])
    goal = ob.State(space)
    # we can pick a random goal state...
    goal.setX(goalpos[0])
    goal.setY(goalpos[1])
    # ... or set specific values
    ss.setStartAndGoalStates(start, goal)
    # this will automatically choose a default planner with
    # default parameters
    solved = ss.solve(1.0)
    if solved:
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print ss.getSolutionPath()



if __name__ == '__main__':
    pass
    