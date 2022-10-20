import random
from collections import deque

import pydot

#dictonaries for parent and actions
parent, action, list_of_nodes = dict(), dict(), dict()


class Missionaries_and_Cannibals():

    def __init__(self):
        
        # Start state =  3 missionaries, 3 cannibals, left side
        self.s = (3, 3, 1)   
        # Goal state = 0 missionaries, 0 cannibals, right side
        self.goalstate = (0, 0, 0)   
        self.possiblemoves = [(1, 0), (0, 1), (1, 1), (0, 2), (2, 0)]  

        self.boat_position = ["right", "left"] # it represents the position of boat


        self.graph = pydot.Dot(graph_type='graph', bgcolor="#fff3af", label="fig: Missionaries and Cannibal State Space Tree", fontcolor="red", fontsize="24")      
        self.visited = {}   # this parameter will check if a state is visited
        self.solved = False  # to check if we reached the goal state

     
    def solve(self, solve_method="dfs"):
        self.visited = dict()
        parent[self.s] = None
        action[self.s] = None
        list_of_nodes[self.s] = None

        return self.dfs(*self.s, 0) if solve_method == "dfs" else self.bfs()


    # this function check for goal state
    def isgoal(self, n_m, n_c, side):
        return (n_m, n_c, side) == self.goalstate

    # this function check for goal state
    def check_start(self, n_m, n_c, side):
        return (n_m, n_c, side) == self.s

    # this function check constraint of no. of cannibals
    def cannibals_outnumbered(self, n_m, n_c):
        n_m_right = 3 - n_m
        
        n_c_right = 3 - n_c
        return (n_m > 0 and n_c > n_m) \
               or (n_m_right > 0 and n_c_right > n_m_right)

    # check constraint conditions for moving 
    def check_action(self, n_m, n_c):
        return (0 <= n_m <= 3) and (0 <= n_c <= 3)


    def solution(self):
        
        state = self.goalstate   # goal state
        route, steps, nodes = [] ,[], []  # route stores the path followed, steps stores the state

        while state is not None:
            route.append(state)  # store each state in the route
            steps.append(action[state])
            nodes.append(list_of_nodes[state])
        
            state = parent[state]
        
        steps, nodes = steps[::-1], nodes[::-1]

        n_m_left, n_c_left = 3, 3
        n_m_right, n_c_right = 0, 0
    
        
        print("\nSteps\n")
        for i, ((n_m, n_c, side), node) in enumerate(zip(steps[1:], nodes[1:])):
            
##            if node.get_label() != str(self.s):
##                node.set_style("filled")
##                node.set_fillcolor("yellow")
        
            print(f"{i + 1}: ({n_m},{n_c}),{n_m} missionaries  and {n_c} cannibals moved from {self.boat_position[side]} to {self.boat_position[int(not side)]}.")

            op = -1 if side == 1 else 1
            
            n_m_left = n_m_left + op * n_m
            n_c_left = n_c_left + op * n_c

            n_m_right = n_m_right - op * n_m
            n_c_right = n_c_right - op * n_c
            
        print("Finished")
        print("*" * 60)

    def draw_edge(self, n_m, n_c, side, depth):
        a, b = None, None
        if parent[(n_m, n_c, side)] is not None:
            a = pydot.Node(str(parent[(n_m, n_c, side)] + (depth - 1, )), label=str(parent[((n_m, n_c, side))]))
            self.graph.add_node(a)

            b = pydot.Node(str((n_m, n_c, side, depth)), label=str((n_m, n_c, side)))
            self.graph.add_node(b)

            edge = pydot.Edge(str(parent[(n_m, n_c, side)] + (depth - 1, )), str((n_m, n_c, side, depth) ), dir='forward')
            self.graph.add_edge(edge)
        else:
            
            b = pydot.Node(str((n_m, n_c, side, depth)), label=str((n_m, n_c, side)))
            self.graph.add_node(b)        
        return a, b


# Implementation using BFS approach
    def bfs(self):
        q = deque()  # Intialising queue
        q.append(self.s + (0, ))
        self.visited[self.s] = True

        # go from a to b where a is the parent[b] and b = (n_m, n_c, side, depth)
        while q:
            n_m, n_c, side, depth = q.popleft()
            
            a, b = self.draw_edge(n_m, n_c, side, depth)    

            
            op = -1 if side == 1 else 1

            can_expand = False

            for x, y in self.possiblemoves:
                next_m, next_c, next_s = n_m + op * x, n_c + op * y, int(not side)
                if (next_m, next_c, next_s) not in self.visited:
                    if self.check_action(next_m, next_c):
                        can_expand = True
                        self.visited[(next_m, next_c, next_s)] = True
                        q.append((next_m, next_c, next_s, depth + 1))
                        
                        # Keep track of parent and corresponding action
                        parent[(next_m, next_c, next_s)] = (n_m, n_c, side)
                        action[(next_m, next_c, next_s)] = (x, y, side)
                        list_of_nodes[(next_m, next_c, next_s)] = b
                
            if not can_expand:
                b.set_style("filled")
                b.set_fillcolor("gray")
        return False

    def dfs(self, n_m, n_c, side, depth):
        self.visited[(n_m, n_c, side)] = True

        a, b = self.draw_edge(n_m, n_c, side, depth)    

        
        if self.check_start(n_m, n_c, side):
            b.set_style("filled")
            b.set_fillcolor("blue")
        elif self.isgoal(n_m, n_c, side):
            b.set_style("filled")
            b.set_fillcolor("green")    
            return True
        elif self.cannibals_outnumbered(n_m, n_c):
            b.set_style("filled")
            b.set_fillcolor("red")
            return False
        else:
            b.set_style("filled")
            b.set_fillcolor("orange")

        solution_found = False
        operation = -1 if side == 1 else 1
        
        can_expand = False

        for x, y in self.possiblemoves:
            next_m, next_c, next_s = n_m + operation * x, n_c + operation * y, int(not side)

            if (next_m, next_c, next_s) not in self.visited:
                if self.check_action(next_m, next_c):
                    can_expand = True
                    # Keep track of parent state and corresponding action
                    parent[(next_m, next_c, next_s)] = (n_m, n_c, side)
                    action[(next_m, next_c, next_s)] = (x, y, side)
                    list_of_nodes[(next_m, next_c, next_s)] = b

                    solution_found = (solution_found or self.dfs(next_m, next_c, next_s, depth + 1))
                
                    if solution_found:
                        return True


        self.solved = solution_found
        return solution_found


def main():
    #creating object
    mc_problem = Missionaries_and_Cannibals()  

    # BFS APPROACH solution
    print("Using BFS approach\n")
    if(mc_problem.solve(solve_method="bfs")):
        mc_problem.solution()

    else:
        print("Solution Not Found")

    # DFS APPROACH
    print("\n\n\nUsing DFS approach\n")
    if(mc_problem.solve(solve_method="dfs")):
        mc_problem.solution()

    else:
        print("Solution Not Found")

class MCGraph:
  def __init__(self):
    self.vertex_list = []
    self.open_list = []
    self.closed_list = []
    self.maxm = 0
    self.maxc = 0
    self.boatcarry = 0

  def astarSearch(self, m, c, bc):
    siddu_steps = 0
    self.maxm = m
    self.maxc = c
    self.boatcarry = bc
    self.open_list = PriorityQueue()
    self.closed_list = set()
    start = MCNode(m, c, 'L')
    start.g = 0
    self.open_list.put((start.h + start.g, start))
    transfer_node = None
    while ( not self.open_list.empty()) and (transfer_node is None or transfer_node.missionaries != 0 or transfer_node.cannibals != 0 or transfer_node.boatpos != 'R'):
      siddu_steps += 1
      transfer_node = self.open_list.get()
      self.closed_list.add(transfer_node)
      adj_nodes = transfer_node.getAdjacentNodes(m, c, self.boatcarry)
      for adj_node in adj_nodes:
        inCL = False
        for closed_node in self.closed_list:
          if adj_node == closed_node:
            if (transfer_node.g + 1 < closed_node.g):
              closed_node.g = transfer_node.g + 1
              closed_node.predecessor = transfer_node
              self.parentRedirection(closed_node)
            inCL = True
            break
        if (inCL):
          continue
        inOL = False
        for open_node in self.open_list:
          if (adj_node == open_node):
            if (transfer_node.g + 1 < open_node.g):
              open_node.g = transfer_node.g + 1
              open_node.predecessor = transfer_node
            inOL = True
            break
        if (inOL):
          continue
        adj_node.g = transfer_node.g + 1
        adj_node.predecessor = transfer_node
        self.open_list.put((adj_node.h + adj_node.g, adj_node))

    print(siddu_steps)
    if transfer_node.missionaries == 0 and transfer_node.cannibals == 0 and transfer_node.boatpos == 'R':
      print("Path:")
      transfer_node.printPath()
  def parentRedirection(self, node):
    adj_nodes = node.getAdjacentNodes(self.maxm, self.maxc, self.boatcarry)
    for adj_node in adj_nodes:
      for closed_node in self.closed_list:
        if adj_node == closed_node:
          if node.g + 1 < closed_node.g:
            closed_node.g = node.g + 1
            closed_node.predecessor = node
            self.parentRedirection(closed_node)
          break

    

if __name__ == "__main__":
    main()
