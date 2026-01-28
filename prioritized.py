import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any
class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        # Question 1 testing
        # constraints.append({'agent': 0, 'loc': [(1,5)], 'timestep': 4})
        # constraints.append({'agent': 0, 'loc': [(1,4)], 'timestep': 4})
        # constraints.append({'agent': 1, 'loc': [(1,2), (1,3)], 'timestep': 1})
        @dataclass
        class VertexConstraint:
            loc: Tuple[int, int]
            timestep: int            

            def getConstraintsList(self, agent_start: int, agent_end: int) -> List[Dict[str, Any]]:
                result = []
                for i in range(agent_start, agent_end):
                    result.append({'agent': i, 'loc': [self.loc], 'timestep': self.timestep})
                # print("getCL: " + str(result))
                return result
        
        @dataclass
        class GoalVertexConstraint:
            loc: Tuple[int, int]
            timestep_start: int

            def getConstraintsList(self, agent_start: int, agent_end: int, time_end: int) -> List[Dict[str, Any]]:
                result = []
                for i in range(agent_start, agent_end):
                    for t in range(self.timestep_start, time_end):
                        result.append({'agent': i, 'loc': [self.loc], 'timestep': t})
                # print("getCL: " + str(result))
                return result
        
        @dataclass
        class EdgeConstraint:
            loc1: Tuple[int, int]
            loc2: Tuple[int, int]
            timestep: int

            def getConstraintsList(self, agent_start: int, agent_end: int) -> List[Dict[str, Any]]:
                result = []
                for i in range(agent_start, agent_end):
                    result.append({'agent': i, 'loc': [self.loc1, self.loc2], 'timestep': self.timestep})
                # print("getCL: " + str(result))
                return result

        num_V = sum([1 for row in self.my_map for cell in row if not cell])
        longest_path_so_far = 0
        agent_goal_constraints: List[GoalVertexConstraint] = []

        for i in range(self.num_of_agents):  # Find path for each agent
            T = longest_path_so_far + num_V
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints, T)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)
            longest_path_so_far = max(longest_path_so_far, len(path))
            
            agent_goal_constraints.append(GoalVertexConstraint(loc=path[-1], timestep_start=len(path)))

            ##############################
            # Task 1.3/1.4/2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################
            _constraints_vertex: List[VertexConstraint] = []
            _constraints_edge: List[EdgeConstraint] = []
            for pathidx in range(len(path)):
                _constraints_vertex.append(VertexConstraint(loc=path[pathidx], timestep=pathidx)) #((x,y), t)
            for pathidx in range(len(path)-1):
                _constraints_edge.append(EdgeConstraint(loc1=path[pathidx], loc2=path[pathidx+1], timestep=pathidx+1)) #((x1,y1),(x2,y2), t)
            for cv in _constraints_vertex:
                constraints.extend(cv.getConstraintsList(agent_start=i+1, agent_end=self.num_of_agents))
            for ce in _constraints_edge:
                constraints.extend(ce.getConstraintsList(agent_start=i+1, agent_end=self.num_of_agents))
            for cg in agent_goal_constraints:
                constraints.extend(cg.getConstraintsList(agent_start=i+1, agent_end=self.num_of_agents, time_end=T+1))
            print(constraints)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
