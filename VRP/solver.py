'''Code file for vehicle routing problem created for Advanced Algorithms
Spring 2020 at Olin College. These functions solve the vehicle routing problem
using an integer programming and then a local search approach. This code has
been adapted from functions written by Alice Paul.'''

import picos as pic
import numpy as np
from read_files import read_file_type_A, read_file_type_C

# Integer programming approach
def cvrp_ip(C,q,K,Q,obj=True):
    '''
    Solves the capacitated vehicle routing problem using an integer programming
    approach.

    C: matrix of edge costs, that represent distances between each node
    q: list of demands associated with each client node
    K: number of vehicles
    Q: capacity of each vehicle
    obj: whether to set objective (ignore unless you are doing local search)
    returns:
        objective_value: value of the minimum travel cost
        x: matrix representing number of routes that use each arc
    '''
    q=np.append(q,0)
    new_row = [C[0][:]]
    C=np.concatenate((C,new_row))
    rowl=len(C)
    rowli=rowl-1
    newcol=np.zeros((rowl,1))
    C=np.concatenate((C,newcol),axis=1)
    new_col = C[:,0]
    C[:,rowli]=new_col

    prob = pic.Problem()
    x = []
    u=[]
    x=prob.add_variable("x",size=(rowl,rowl),vtype='binary')
    u=prob.add_variable("u",size=(rowl),vtype='continuous',upper=Q, lower=q)
    objective_value = 0
    prob.add_constraint(sum([x[0,i]for i in range (0,rowl)])<=K)
    prob.add_constraint(sum([x[i, rowl-1]for i in range (0,rowl)])<=K)
    prob.add_constraint(sum([x[0,i]for i in range (0,rowl)])==sum([x[i, rowl-1]for i in range (0,rowl)]))
    prob.add_list_of_constraints([sum([x[i,j]for i in range (0,rowl)]) ==1 for j in range (1,rowl-1)])
    prob.add_list_of_constraints([sum([x[j,i]for i in range (0,rowl)]) ==1 for j in range (1,rowl-1)])
    prob.add_constraint(sum([x[i,0]for i in range (0,rowl)])==0)
    prob.add_list_of_constraints([u[i]-u[j]+Q*x[i,j] <= Q-q[i] for i in range (0,rowl) for j in range (0,rowl)])
    prob.set_objective("min",pic.sum([C[i,j]*x[i,j] for i in range(0,rowl) for j in range(0,rowl)]))
    prob.solve(solver='cplex')
    objective_value=prob.obj_value()
    return objective_value, x

# Local search approach (OPTIONAL)
# def local_search(C,q,K,Q):
#     '''
#     Solves the capacitated vehicle routing problem using a local search
#     approach.
#
#     C: matrix of edge costs, that represent distances between each node
#     q: list of demands associated with each client node
#     K: number of vehicles
#     Q: capacity of each vehicle
#     returns:
#         bestval: value of the minimum travel cost
#         bestx: matrix representing number of routes that use each arc
#     '''
#     bestx = []
#     bestval = 0
#
#     # TODO (OPTIONAL): implement local search to solve vehicle routing problem
#
#     return bestval, bestx


if __name__ == "__main__":

    # an example call to test your integer programming implementation
    C,q,K,Q = read_file_type_A('data/A-n05-k04.xml')
    travel_cost, x = cvrp_ip(C,q,K,Q)
    print("Travel cost: " + str(travel_cost))
