import numpy as np
import math
# class MDP that takes in number of state, action and allow users to update
# transition functions through gradually inputting logic

class MDP():
    """
    The following class defines a MDP
    Running example MDP with 3 state variables: a, b, c
        with  a = [1, 2, 3]
              b = [4, 5, 6, 7]
              c = [10, 12, 13, 15]
    Three different actions ["forward", "left", "right"]
    """

    def __init__ (self, states=None, actions=None, r=-1.0, method="replace"):
        """
        Define the value list of states and actions
        State should be a list of list, with sub-lists are lists of state values

          input: states=[a, b, c]

        Action should be a list

          input: actions = ["forward", "left", "right"]
        
          
        method: replace or add: replacing the current element in P matrix if value is > 0 or add into. Default to replace.
        """
        self.s = states
        self.a = actions
        self.num_s_vars = len(self.s)
        self.num_s = 1
        self.s_range = []
        for i in range(self.num_s_vars):
            self.s_range.append(len(self.s[i]))
            self.num_s = self.num_s * len(self.s[i])
        self.num_a = len(self.a)
        # transition function
        # P[new_state, current_state, action] = probability
        self.P = np.zeros((self.num_s, self.num_s, self.num_a))

        # reward function
        self.R = r * np.ones((self.num_s, self.num_a))
        
        self.method = method
  
    def add_route(self, current_state, action, new_state, p=1.0):
        """
        Add new transition route to MDP
          Default probability is 1.0
          current_state (list): [a_i, b_i, c_i]. E.g: [1, 4, 15]
          new_state (list): [a_ii, b_ii, c_ii]. E.g: [1, 5, 10]
          action: "forward"
        """
        # get correct index of MDP action from input action
        action_index = self.a.index(action)

        if self.P[self.get_index(new_state), self.get_index(current_state), action_index] > 0.0:
            # print("Warning: Already have prop value of {}. Use {} to deal with".format(self.P[self.get_index(new_state), self.get_index(current_state), action_index], self.method))
            if self.method == "replace":
                self.P[self.get_index(new_state), self.get_index(current_state), action_index] = p
            else:
                self.P[self.get_index(new_state), self.get_index(current_state), action_index] += p
        else:
            self.P[self.get_index(new_state), self.get_index(current_state), action_index] = p

    def add_reward(self, state, action, reward):
        self.R[self.get_index(state), self.a.index(action)] = reward
  
    def get_state(self, index):
        """
        Get state index tuple from index
        E.g. 
          Input: 0
          Output: [0, 0, 0]
        """
        state_index = []
        divisor = self.num_s
        dividend = index
        for i in range(self.num_s_vars-1, -1, -1):
            divisor = divisor / self.s_range[i]
            quotient = math.floor(dividend / divisor)
            remainder = dividend % divisor
            state_index.append(quotient)
            dividend = remainder
        return np.flip(state_index)

    def get_real_state_value(self, index):
        """
        Get state tuple from index
        E.g.
          Input: 0
          Output: [1, 4, 10]
        """
        index = self.get_state(index)
        return [self.s[i][v] for i, v in enumerate(index)]
  
    def get_index(self, state):
        """
        Get index from state tuple
        E.g.
          Input: [1, 4, 10]
          Output: 0
        """
        # get state index tuple from state tuple
        state_index = [self.s[i].index(v) for i, v in enumerate(state)]
        cur_mul = self.num_s
        index = 0
        for i in range(self.num_s_vars-1, -1, -1):
            cur_mul = cur_mul / self.s_range[i]
            index =  index + state_index[i] * cur_mul
        return int(index)
  
    def get_mdp(self):
        return self.num_a, self.num_s, self.R, self.P