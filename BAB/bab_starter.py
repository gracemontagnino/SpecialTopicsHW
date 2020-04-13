import picos as pic
from picos import RealVariable
from copy import deepcopy
from heapq import *
import heapq as hq
import numpy as np
import itertools
import math
counter = itertools.count() 

class BBTreeNode():
    def __init__(self, vars = [], constraints = [], objective='', prob=None):
        self.vars = vars
        self.constraints = constraints
        self.objective = objective
        self.prob = prob

    def __deepcopy__(self, memo):
        '''
        Deepcopies the picos problem
        This overrides the system's deepcopy method bc it doesn't work on classes by itself
        '''
        newprob = pic.Problem.clone(self.prob)
        return BBTreeNode(self.vars, newprob.constraints, self.objective, newprob)
    
    def buildProblem(self):
        '''
        Bulids the initial Picos problem
        '''
        prob=pic.Problem()
   
        prob.add_list_of_constraints(self.constraints)    
        
        prob.set_objective('max', self.objective)
        self.prob = prob
        return self.prob

    def is_integral(self):
        '''
        Checks if all variables (excluding the one we're maxing) are integers
        '''
        for v in self.vars[:-1]:
            if v.value == None or abs(round(v.value) - float(v.value)) > 1e-4 :
                return False
        return True

    def branch_floor(self, branch_var):
        '''
        Makes a child where xi <= floor(xi)
        '''
        n1 = deepcopy(self)
        n1.prob.add_constraint( branch_var <= math.floor(branch_var.value) ) # add in the new binary constraint

        return n1

    def branch_ceil(self, branch_var):
        '''
        Makes a child where xi >= ceiling(xi)
        '''
        n2 = deepcopy(self)
        n2.prob.add_constraint( branch_var >= math.ceil(branch_var.value) ) # add in the new binary constraint
        return n2


    def bbsolve(self):
        '''
        Use the branch and bound method to solve an integer program
        This function should return:
            return bestres, bestnode_vars

        where bestres = value of the maximized objective function
              bestnode_vars = the list of variables that create bestres
        '''

        # these lines build up the initial problem and adds it to a heap
        root = self
        res = root.buildProblem().solve(solver='cvxopt')
        heap = [(res, next(counter), root)]
        bestres = -1e20 # a small arbitrary initial best objective value
        bestnode_vars = root.vars # initialize bestnode_vars to the root vars

        while (len(heap)>0):
            _,_,root=hq.heappop(heap)
            for vari in root.vars:
                if vari.value == None or abs(round(vari.value) - float(vari.value)) > 1e-4:
                    ceil=root.branch_ceil(vari)
                    floor=root.branch_floor(vari)
                    ceil.prob.solve(solver='cvxopt')
                    floor.prob.solve(solver='cvxopt')
                    hq.heappush(heap,(ceil.vars[-1], next(counter), ceil))
                    hq.heappush(heap,(floor.vars[-1], next(counter), floor))
                    if ceil.is_integral()==False:
                        if ceil.vars[-1]>bestres:
                            bestres=ceil.vars[-1]
                            bestnode_vars=ceil.vars
                    if floor.is_integral()==False:
                        if floor.vars[-1]>bestres:
                            bestres=floor.vars[-1]
                            bestnode_vars=floor.vars
                    break

        return bestres, bestnode_vars
        
        return bestres, bestnode_vars
 
