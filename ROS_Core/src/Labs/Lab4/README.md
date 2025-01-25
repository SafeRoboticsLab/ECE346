# Lab 4: MDP and POMDP
**[Due 11:59PM Thursday, April 3]**

This lab will focus on using Markov Decision Process (MDP) and Partially Observable Markov Decision Process (POMDP) in solving robot planning problems. We will first look at how MDP works through an example of T-intersection negotiation and methods to solve MDP (value iteration, policy iteration). We will then move on to POMDP, specifically problem formulation, and solution through approximation, with a focus on QMDP. You will get a brief refresher on Bayesian Inference, and how it is used in updating the belief in approximating solution of POMDP.

**Note**: Make sure you have **pulled the code from upstream** into your repository and **updated all submodules**, i.e.,
```bash
git pull upstream 2025 --recurse-submodules
```

# Getting Started
MDP and POMDP can help solve problems in probabilistic planning and control. Specifically, MDP is a framework that can be used to solve robot planning problems when there is uncertainty in robot motion. This means that the states are assumed to be **fully observable**, and only the action effects are uncertain. On the other hand, POMDP helps solve problems with both uncertainty in action effects and perception, by applying iterative calculation process on belief state representation. Solving POMDP can be easily intractable, and thus approximations can be used to calculate control policies more efficiently instead.

This lab will first work you through the construction of a fully observable MDP. You will then attempt to write your own *value iteration* and *policy iteration* on a given MDP of the T-intersection problem, with the task to negotiate to turn right when the other car wants to turn left. There will be **{3 tasks} for this first half of the lab.

The second half of the lab will walk you through POMDP problem formulation. You will then extend the previous MDP formulation to POMDP one. Instead of solving it directly, we will approximate the solution using QMDP. You will first write a function to update our belief using observations gathered throughout negotiation process using Bayesian Inference. You will then construct QMDP to calculate the next best actions to take, given the updated belief. By attempting to change reward and transition function of the underlying fully observable MDP, you can further see how these changes can affect the final rollout of the negotiation process. There will be **3 tasks** for this second half of the lab.

Refer to the Jupyter Notebook [`pomdp.ipynb`](https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/pomdp.ipynb) and docstring to get started.

# T-intersection negotiation MDP
### Task 1.1: MDP formulation
### Use the MDP class
In this task you will first formulate the MDP used to represent the T-intersection problem. To simplify the process, we have created the **{MDP} class for you. To initialize the MDP, create an MDP object from the respective class with the following information:

```python
    mdp_object = MDP(
        states=[[state_variable_1],[state_variable_2],[...]],
        actions=[action_1, action_2, ...],
        r=-1 # default reward,
        method=method # default to "replace"
    )
```

With `method` $\in \{\text{`replace}, `add}}\}$ determining how the routing probability will solve conflicts between two similar `MDP.add_route()} prompts.  MDP will then used your input states and actions to create an empty matrix of P and R. The matrix has the following structure:
```python
    # P[new_state, current_state, action] = probability
    mdp_object.P.shape = (num_state, num_state, num_action)
    # R[state, action] = reward, default to -1
    mdp_object.R.shape = (num_state, num_action)
```

The following is a list of internal variables of this class that you can access:
```python
    MDP().a             # list of actions
    MDP().s             # list of state variables
    MDP().num_s         # number of state
    MDP().num_s_vars    # number of state variables
    MDP().num_a         # number of action
    MDP().P             # transition matrix
    MDP().R             # reward matrix
```    

The following functions are supported in this class:

```python
    # Add new route to current MDP object
    # If method is set to "replace", p = p_new
    # Else if method is "add", p = p + p_new
    add_route(current_state, action, new_state, p=1.0)
    
    # Add new reward to current MDP routes
    add_reward(state, action, reward)
    
    # Get state indexing in MDP().P and MDP().R
    get_index(state)
    
    # Get state in index form from index
    get_state(index)
    
    # Get state in real state variable value from from index
    get_real_state_value(index)
    
    # Get basic MDP information
    # Output: num_a, num_s, R, P
    get_mdp()
```

The class MDP will help you to index and manage routes and rewards within the MDP easier. Assuming that you have an MDP with 2 state variables $s_1=\{$`restaurant}, `supermarket`$\}$, $s_2 = \{$`vacant}, `full`$\}$ and 4 different actions to take $a=\{$`left`, `right`, `forward`, `backward`$\}$, simply create and index your MDP as follows:

```python
    mdp_object = MDP(
        states=[
            ["restaurant","supermarket"], # range of state var s_1
            ["vacant","full"] # range of state var s_2
        ],
        actions=["left","right","forward","backward"]
    )
    # to say that the probability to go to 
    # s_new=[supermarket,vacant] from s_cur=[restaurant,full] 
    # when taking action a=forward is 0.8, simply call
    mdp_object.add_route(
        ["restaurant","full"], 
        "forward", 
        ["supermarket","vacant"],
        p=0.8
    )
```

Let's have a simple example to show you how this class works. Assuming that we want to describe the MDP in figure \ref{fig:two-state-mdp}, the following code describes the respective MDP using our MDP class:
\begin{figure}[htp]
    \centering
    \includegraphics[width=12cm]{lab3/figures/2-state-mdp.png}
    \caption{Simple two-state MDP}
    \label{fig:two-state-mdp}
\end{figure}

```python
    class TwoStateMDP(MDP):
        def __init__(self):
            self.states = [["s1", "s2"]]
            self.actions = ["a0", "a1"]
            self.gam = 0.9
            
            # call the parent class
            # notice that the state is a list of state variables
            super().__init__(
                states=self.states, actions=self.actions)
            self.populate_data()
        
        def populate_data(self):
            # add all routes from s1
            self.add_route(["s1"],"a0",["s1"])
            self.add_route(["s1"],"a1",["s2"])
            # add all routes from s2
            self.add_route(["s2"],"a0",["s2"])
            self.add_route(["s2"],"a1",["s2"])
            
            # let's populate the reward, assuming r>0 is 0.5
            for a in self.a:
                self.add_reward(["s1"],a,0.5)
                self.add_reward(["s2"],a,1.5)
```

Let's test some internal functions of class MDP:
```python
    twoStateMDP = TwoStateMDP()
    print(twoStateMDP.get_index(["s1"]))
    print(twoStateMDP.get_state(0))
    print(twoStateMDP.get_real_state_value(0))
    
    # Output
    >>> 0
    >>> [0]
    >>> ['s1']
```

### Task 1.1: T-intersection MDP formulation}
Your task is to use our MDP class to create the MDP based on figure \ref{fig:t-intersection-mdp}.
\begin{figure}[htp]
    \centering
    \includegraphics[height=8cm]{lab3/figures/t-intersection-mdp.jpg}
    \caption{Passenger pick up MDP diagram for Minicity}
    \label{fig:t-intersection-mdp} 
\end{figure}

The T-intersection problem has 2 cars: our car, called `ego}, and the other car, called `other}. The state $s = \{\text{ego}_i, \text{other}_j\} \; \text{with} \; i,j \in \{1 \dots 5\}$ includes where we are our trajectory (in yellow) and where the other car is on its trajectory (in blue). The action space has 2 actions: `forward} and `stop}. The following information describes our MDP:

\begin{itemize}
    \item Our choice of action affects the other car's action as follow:
        \begin{align*}
            P(a_{\text{other}} | a_{\text{ego}}) = \begin{vmatrix}
                & \text{forward} & \text{stop} 
                \text{forward} & 0.2 & 0.8 
                \text{stop} & 0.4 & 0.6
            \end{vmatrix}
        \end{align*}
        Meaning that if we take the action `forward}, there is a **{0.8} chance that the other car will `stop}, and **{0.2} chance that both cars will move forward.
    \item When moving forward with action `forward}, for each car there is a **{0.8} chance of moving 1 step ahead, and **{0.2} chance of moving 2 steps ahead.
    \item When choosing action `stop}, the car will stop with probability **{1.0}
    \item The system terminates when either car reaches the goal, i.e. $\text{ego}_i = \text{ego}_5$ or $\text{other}_i = \text{other}_5$, or when collision happens, i.e. $s = \{\text{ego}_i, \text{other}_j\}$ with $i, j \in \{4, 5\}$.
\end{itemize}

The reward function is as follows:
\begin{itemize}
    \item `forward} action has reward **{-1}.
    \item `stop} action has reward **{-5}.
    \item Reaching the goal gives reward **{+5}.
    \item Collision gives reward **{-10}.
\end{itemize}

In the provided Jupyter Notebook, you will see the docstring for task 1.1. Compute the state transition probability and fill in the missing `p} values for each `MDP.add_route()} commands.

### Task 1.2: Value iteration and policy iteration}
Your next task is to write value iteration and policy iteration for the MDP that you have just created. Refer to Section \ref{mdp-background} for more information.

Refer to the docstring in your Jupyter Notebook for task 1.2. To check if your implementation of value iteration and policy iteration is correct, we provide you with a closed-form solution for the simple two-state MDP. Once finish implementing, run the test cases provided to check if your calculated $V^*(x), \pi^*(x)$ match the closed-form solution.

### Task 1.3: Simulate your computed $\pi^*(x)$}
We provide you with a visualizer class for the T-intersection problem located in https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/visualizer.py}{`./Lab3/visualizer.py}}. The class `TIntersectionVisualizer} has a function `TIntersectionVisualizer.plot()} that takes parameter `state} and gives you the visualization of the system at that state.

From the $V^*, \pi^*$ in task 1.2, using the provided visualizer, do the following tasks:
\begin{enumerate}
    \item Choose an initial state.
    \item Iterate from the initial state until you reach a terminal condition (reach goal or in collision).
    \item Maintain a list of all states that you have visited.
    \item Use the provided visualizer and function to plot all figures of each recorded state and create a GIF. Show this to your AIs.
\end{enumerate}

# T-intesection QMDP}
Let us now turn this problem into a POMDP one. Assume that we have the same T-intersection negotiation problem as our underlying MDP, but this time the state is not fully observable, in the sense that we always know where we are, but \emph{we do not know where the other car is}. The following modifications are added to our old MDP:

\begin{itemize}
    \item Add in a new action `look}
    \item When action is `look}, we will receive reward **{-1}
    \item When action is `look}, the new state is the current state with probability **{1.0}
    \item After apply action `look}, we receive an observation $z$ corresponding to where the other car is, with probability
    \begin{align*}
        & P(z = \text{other}_i | \text{other}_i) = 0.8  
        & P(z = \text{other}_{\max\{1, i-1\}} | \text{other}_i) = 0.1  
        & P(z = \text{other}_{\min\{5, i+1\}} | \text{other}_i) = 0.1
    \end{align*}
\end{itemize}
We can approximate the POMDP solution with QMDP by computing offline the value function of the underlying MDP before doing online QMDP. Following Section \ref{qmdp-background}, the state value $\hat{V}$ of this MDP can be used in QMDP later.

As our belief is the probability of where the other car is, we can use Dirichlet-mulinomial model to gradually update our belief, by treating this similar to the 5-face dice toss problem.

### Task 2.1: Defining POMDP problem}
Follow the docstring in the Jupyter Notebook to create a new MDP for T-intersection negotiation problem with extra action `look}, additional state transition information and reward.

### Task 2.2: QMDP}
For this task you will attempt to calculate $\hat{V}$ using the same value iteration that you have written in Task 2.

Next, you will implement QMDP as described in Section \ref{qmdp-background}. Your QMDP should return a single action every time. Use the docstring in Jupyter Notebook to finish this task.

### Task 2.3: Observation and belief space}
For this task you will create a Diriclet-multinomial model that uses Bayes' Theorem to recompute your predictive posterior. Refer to Section \ref{beta-binomial-background} and docstring in your Notebook to build this.

Once you have built your Dirichlet-multinomial model, run the last block code in your Notebook to see how everything is connected. Does the result make sense?

# Background
This section attempts to go through the basic concepts of MDP and POMDP that might be useful for this lab.

## Markov Decision Process
MDP is a discrete-time control system whose transitions are probabilistic. In MDP, the states are fully observable, and only the control effects are uncertain. We can represent this uncertainty through state transition function. In MDP, we will have states $\mathcal{S}$, action $\mathcal{A}$, transition probability $P(s'| s, a)$ and reward function $r(s, a): \mathcal{S} \times \mathcal{A} \rightarrow \mathbb{R}$.

A policy $\pi : \mathcal{S} \times \mathcal{A}$ is a mapping from states to actions, determining what action to take from each possible state. Our goal in solving MDP is to either maximizing the cumulative reward, or minimizing the cost, and thus the optimal policy can be written as:
$
\begin{equation}
    \pi^*=\arg \max_{\pi} J(s_0, (\pi(s_0), \pi(s_1),\dots)) = \mathop{\mathbb{E}}_{s_0, s_1,\dots} \sum_{t_0}^\infty\gamma^t r(s_t, \pi(s_t))
\end{equation}
$

In MDP, value function is a function of state, $V(s)$ that defines the expected value of a state given a defined policy. And if we apply the optimal policy $\pi^*$, we will receive the maximum value of a state, denoted $V^*(s)$. The value function can be written as:
$
\begin{equation}
    V(s_0):= \max_{\pi} \mathop{\mathbb{E}}_{s_1, s_2, \dots} \sum_{t_0}^\infty \gamma^t r(s_t, \pi(s_t))
\end{equation}
$

From this, we can rewrite the equation for value function as:
$
\begin{equation}
    V(s_0) = \max_{a_0} \mathop{\mathbb{E}}_{s_1} [r(s_0, a_0) + \gamma V(s_1)]
    % eq:bellman_v_function
\end{equation}
$
Equation \eqref{eq:bellman_v_function} is the Bellman Equation for MDP that can be solved using Dynamic Programming. 

### Value iteration
Value iteration is a process in which we apply the Bellman Equation to iteratively update the value of all states. We modify \eqref{eq:bellman_v_function} into:

$
\begin{equation}
    V^{k+1} = \max_{\pi} [r_\pi + \gamma P_\pi V^k]
    % \label{eq:value_iteration}
\end{equation}
$

Using \eqref{eq:value_iteration}, we have the following pseudo-code for Value Iteration:

\begin{figure}[H]
    \centering
    \begin{minipage}{0.8\textwidth}
    \begin{algorithm}[H]
    \caption{Value Iteration}
    \begin{algorithmic}[1]
        \small
        \Require $P(\cdot), r(\cdot)$ (state transition function/reward function), $\gamma, \epsilon$.
        \State $V^0 \gets \mathbb{R}^N $ \Comment{Start with any guess}
    	\While {not $||V^{k+1} - V^k||_\infty < \epsilon$}
    	\For{s in $\mathcal{S}$} \Comment{Update new value for all states}
    	    \State $V^{k+1}(s) \gets \max_{\pi} [r(s, \pi(s)) + \gamma P_\pi V^k(s)]$ 
    	\EndFor \vspace{2pt}
    	\EndWhile
    	\Return $V$
    	\end{algorithmic}
    \label{algo:fwd_pass}
    \end{algorithm}
    \end{minipage}
\end{figure}

Once we got the converged value from *{Value Iteration}, the optimal policy $\pi^*$ can be computed as:
\begin{equation}
    a^*_s := \pi^*(s) = \arg \max_{a} r(s, a) + \gamma P_a V \label{eq:optimal_policy_from_value}
\end{equation}

Equation \eqref{eq:optimal_policy_from_value} says that the optimal action to take at each state is the action that maximizes the immediate reward and discounted cumulative reward, among all the actions that can be taken from the respective state.

### Policy iteration}
In policy iteration, you start off with a candidate/guess policy $\pi$. Then, using that policy, we will calculate the value for all states, and use the state values to derive a new policy. The iterative process then follows this pseudo-code:


$\begin{figure}[H]
    \centering
    \begin{minipage}{0.8\textwidth}
    \begin{algorithm}[H]
    \caption{Policy Evaluation}
    \begin{algorithmic}[1]
        \small
        \Require $\pi$ (current policy), $P(\cdot), r(\cdot)$, $\gamma, \epsilon$.
    	\While{not $||V^{k+1} - V^k||_\infty < \epsilon$}
    	\For{s in $\mathcal{S}$}
    	    \State $V^{k+1}(s) \gets \sum P_{\pi} [r(s, \pi(s)) + \gamma V^k(s)]$ 
    	\EndFor \vspace{2pt}
    	\EndWhile
    	\Return $V$
    	\end{algorithmic} $
    \end{algorithm} $
    \end{minipage} $
\end{figure}$


$\begin{figure}[H]
    \centering
    \begin{minipage}{0.8\textwidth}
    \begin{algorithm}[H]
    \caption{Policy Iteration}
    \begin{algorithmic}[1]
        \small
        \Require $P(\cdot), r(\cdot)$ (state transition function/reward function), $\gamma, \epsilon$.
        \State $\pi(s) \gets \mathbb{R}^N $ \Comment{Start with any candidate/guess policy}
        \State $*{\mbox{policy-stable}} \gets True$
        \State $V \gets \mbox{policy-evaluation($\cdot$)}$ \Comment{Get the current state value with initialized policy}
    	\While{$True$}
    	\For{s in $\mathcal{S}$}
    	    \State $*{\mbox{old-action}} \gets \pi(s)$ 
    	    \State $\pi(s) \gets \arg \max_{a} \sum P_{\pi} [r(s, \pi(s)) + \gamma V(s)]$
    	    \If{$*{\mbox{old-action}} \neq \pi(s)$}
    	        \State $*{\mbox{policy-stable}} \gets False$
    	    \EndIf
    	\EndFor \vspace{2pt}
    	\If{not $*{\mbox{policy-stable}}$}
    	    \State $V \gets \mbox{policy-evaluation($\cdot$)}$ \Comment{Calculate new state value using new policy}
    	    \State $*{\mbox{policy-stable}} \gets True$
    	\Else
    	    \State break
    	\EndIf
    	\EndWhile
    	\Return $V$, $\pi$
    	\end{algorithmic}
    \label{algo:fwd_pass}
    \end{algorithm}
    \end{minipage}
\end{figure}$


Policy iteration and value iteration, if implemented correctly, will result in similar optimal policy.

## Partially Observable Markov Decision Process
In POMDP, we can apply similar idea in MDP. However, the state $s$ is not observable. The robot has to make its decision in the belief state, which is the space of posterior distributions over states.

POMDP computes a value function over belief space:
\begin{equation}
    V_T(b) = \max_{u}[r(b, a) + \int \gamma V_{T-1}(b')p(b' | a, b)db']
\end{equation}
With $V_0(b) = \gamma \max_{a} \mathbb{E}_x[r(s, a)]$. The induced control policy is as follows:
\begin{equation}
    \pi_T(b) = \arg \max_{a} [r(b, a) + \int \gamma V_{T-1}(b')p(b' | a, b)db']
\end{equation}

A belief is a probability distribution; thus, each value in a POMDP is a function of an entire probability distribution. This is problematic. If the state space is finite, the belief space is continuous, since it is the space of all distributions over the state space. Thus, there is a continuum of different values, whereas there was only a finite number of different values in the MDP case. The situation is even more intractable when we are dealing with continuous state spaces, as the belief space will be an infinitely-dimensional continuum. 

However, when the world we are considering is finite, that is, the state space, the action space, the space observations and the planning horizon are all finite, an exact solution exists. Though solution can be calculated in this case, the computational expense of exact planning makes it inapplicable to use with almost any practical problems in robotics.

## {QMDP} 
% \label{qmdp-background}
Instead of computing the exact solution, we can approximate it. One of the methods to do so is *{QMDP}. *{QMDP} is a hybrid between MDP and POMDP. This algorithm generalizes the MDP-optimal value function defined over states, into a POMDP-style value function over beliefs. QMDP works under the assumption that after one step of control, the state becomes fully observable. 

The following pseudo-code shows how QMDP works:

\begin{figure}[H]
    \centering
    \begin{minipage}{0.8\textwidth}
    \begin{algorithm}[H]
    \caption{QMDP} \label{alg:qmdp}
    \begin{algorithmic}[1]
        \small
        \Require $b=(p_1, p_2, \dots, p_N)$ (beliefs), $P(\cdot), r(\cdot)$ (state transition function/reward function of MDP), $\gamma, \epsilon$.
        \State $\hat{V} \gets \mbox{MDP-value-iteration($\cdot$)}$
    	\For{a in $\mathcal{A}$}
    	    \State $Q(s_i, a) \gets r(s_i, a) + \sum_{j=1}^N \hat{V}(s_j)P(s_j | a, s_i)$
    	\EndFor \vspace{2pt}
    	\State **{return} $\arg \max_{a} \sum_{i=1}^N p_i Q(s_i, a)$
    	\end{algorithmic}
    \label{algo:fwd_pass}
    \end{algorithm}
    \end{minipage}
\end{figure}

The mathematical trick of *{QMDP} is relatively straightforward. The MDP provides us with s state-based value function that is optimal under the assumption that state is fully observable., The resulting value function $\hat{V}$ is defined over world states. The *{QMDP} generalizes this value to the belief space through the mathematical expectation:

\begin{equation}
    \hat{V}(b) = \mathbb{E}_s[\hat{V}(s)] = \sum_{i=1}^N p_i \hat{V}(s_i)
\end{equation}

With $p_i = b(s_i)$. It can also be observed from Algorithm \ref{alg:qmdp} that the computation on line 3 of the algorithm is computing the MDP-value of taking an action $a$ at a certain state $s_i$. We then generalize the value to the belief states, by taking the expectation over the belief state. We finally maximize over all actions, and return the control action with the highest expected value.

### Bayesian Inference}
Bayesian Inference can be used to update the belief space in QMDP when new observations are received. Let's revise this a little. Combining the definition of conditional probability with the product and sum rules yields **{Bayes rule}, also called **{Bayes Theorem}:
\begin{equation}
    p(X=x|Y=y) = \frac{p(X=x, Y=y)}{p(Y=y)}=\frac{p(X=x)p(Y=y|X=x)}{\sum_{x'}p(X=x')p(Y=y|X=x')}
\end{equation}

Let's look at an example of this. Consider the following medical diagnosis problem: Suppose a person is testing for cancer. This person is told that the test has a **{sensitivity} of $80\%$. This means that if this person has cancer, the test will be positive with probability 0.8 (true positive). In order words,

\begin{equation}
    p(x=1|y=1) = 0.8
\end{equation}

where $x=1$ is the event that test is positive, and $y=1$ is the event this person actually has cancer. Many people will conclude that this person has $80\%$ chance to have cancer. But this is not the case. Doing so will ignore the prior probability of having this type of cancer, which, assuming and fortunately, is low:

\begin{equation}
    p(y=1) = 0.004
\end{equation}

Ignoring this prior is called the **{base rate fallacy}. We also need to take in account that the test may be a **{false positive} or **{false alarm}. Unfortunately, such false positives are quite likely (with current technology):

\begin{equation}
    p(x=1|y=0)=0.1
\end{equation}

Combining these three terms using Bayes rule, we can compute the correct answer as follows:

\begin{align*}
    p(y=1|x=1) &= \frac{p(x=1|y=1)p(y=1)}{p(x=1|y=1)p(y=1) + p(x=1|y=0)p(y=0)} 
                &= \frac{0.8 \times 0.004}{0.8 \times 0.004 + 0.1 \times 0.996} = 0.031
\end{align*}

In order word, if this person test comes out as positive, it would means that there is only $3\%$ chance of actually having cancer (which is now less scary in comparison to the original $80\%$).

### Dirichlet-multinomial model} \label{beta-binomial-background}
The Dirichlet-multinomial model uses Bayes Theorem to gradually update our belief for a multinomial distribution. Using Dirichlet distribution as prior for multinomial distribution in Bayesian Inference is helpful in computation, as Dirichlet is a **{conjugate prior} of multinoulli, meaning that the calculated posterior distribution is in the same probability distribution family as the prior probability distribution (Dirichlet, in this case).

Consider a dice with K sides, we want to infer the probability of this dice giving is the face k. Suppose we observe $N$ dice rolls, $\mathcal{D} = \{x_1, \dots, x_N\}$ where $x_i \in \{1, \dots, K\}$. If we assume that the data is iid, the likelihood has the form:

\begin{align*}
    p(\mathcal{D}|\theta) = \Pi_{k=1}^K \theta_k^{N_k}
\end{align*}

Where $N_k$ is the number of times event k occurred.

We will use Dirichlet distribution as our conjugate prior. Define the Dirichlet prior as:

\begin{align*}
    \text{Dir}(\theta | \alpha) = \frac{1}{B(\alpha)}\Pi_{k=1}^K \theta_k^{\alpha_k - 1}
\end{align*}

Multiplying the likelihood by the prior, we find that the posterior is also Dirichlet:

\begin{align*}
    p(\theta|\mathcal{D}) &\propto p(\mathcal{D}|\theta)p(\theta) 
    & \propto \Pi_{k=1}^K \theta_k^{N_k}\theta_k^{\alpha_k - 1} = \Pi_{k=1}^K \theta_k^{\alpha_k + N_k - 1} 
    & \propto \text{Dir}(\theta | \alpha_1 + N_1, \dots, \alpha_K + N_K)
\end{align*}

We can see that the posterior is obtained by adding the prior hyper-parameters (pseudo-counts) $\alpha_k$ to the empirical counts $N_k$.

From here, we can derive the posterior predictive distribution for a single multinoulli trial, given by the following expression:

\begin{align*}
    p(X=j | \mathcal{D}) = \frac{\alpha_j + N_j}{\alpha_0 + N}
\end{align*}

With $\alpha_0 \equiv \sum_{k=1}^K \alpha_k$, the sample size of the prior.

Let's walk through an example to understand this easier. Assuming that we want to know the probability of each face of a 6-face dice. We will model the distribution with Dirichlet-multinomial. Let us have an unbiased prior belief on the distribution of the faces, that is, every face has the same initial probability of $\theta_j = \frac{1}{6}$ ($\alpha = \{1, 1, 1, 1, 1, 1\}$).

We start throwing the dice 10 times, and record the values, $\mathcal{D} = \{1, 2, 1, 1, 2, 5, 4, 2, 1, 2\}$. Using the above posterior predictive equation, we now have the following updated probability for each face:

\begin{align*}
    p(X=j|\mathcal{D}) &= \frac{\alpha_j + N_j}{\sum_{i}\alpha_i + N_i} = \frac{1 + N_j}{6 + 10} 
    &= (\frac{5}{16}, \frac{5}{16}, \frac{1}{16}, \frac{2}{16}, \frac{2}{16}, \frac{1}{16})
\end{align*}