# Lab 3 - Collision Avoidance and Navigation in Dynamic Environment (Forward Reachable Set)
**[Due 11:59PM Thursday, March 6]**


In this lab, we will dive deeper into our ILQR trajectory planner. Specifically, we will introduce its new capability to avoid static and dynamic obstacles. First, we will build upon your Lab 2's result and allow your robot to navigate around static obstacles. Then, we will integrate forward-reachable sets to enable your robot to interact with other robots through a traffic simulator, with other cars joining the traffic with your robot.

There are **3 tasks** in this lab, and you will need to submit (push) your results and demonstrate them to a lab TA before **11:59PM March 6, 2025**.

**Note**: Make sure you have **pulled the code from upstream** into your repository and **updated all submodules**, i.e.,
```bash
git pull upstream 2025 --recurse-submodules
```

# Getting Started #

In this lab, you will use your ILQR algorithm developed in the last lab to plan collision-free trajectories. A new node called `/traffic_simulation_node` is introduced in your workspace. This node (**Figure 1**) simulates static and dynamic obstacles and publishes them under topics `/Obstacles/Static` and `/Obstacles/Dynamic`. Your trajectory planner will leverage these messages and pass them into ILQR. To see the full changes of ROS nodes from Lab 2 to Lab 3, please refer to **Figure 8** and **Figure 9** in the Appendix.

![`/traffic_simulation_node` is added to ROS workspace in Lab 3 and it publishes `/Obstacles/Static` and `/Obstacles/Dynamic` topics](/assets/traffic_simulation_node_graph.png)

***Figure 1**: `/traffic_simulation_node` is added to the ROS workspace in Lab 3 and it publishes the `/Obstacles/Static` and `/Obstacles/Dynamic` topics*


\begin{figure}[h]
    \centering
    \includegraphics[width=0.6\textwidth]{lab3/figures/traffic_simulation_node_graph.png}
    \caption{}
    \label{fig: lab3_new_nodes}
\end{figure}

\section{Static Obstacles}
In the first part of this Lab, we will build collision avoidance functionality based on your ILQR. After activating ROS environment, rebuilding (`catkin_make}), and sourcing the workspace, we can launch the ROS nodes by
\begin{lstlisting}[language=bash]
roslaunch racecar_planner lab3_task1.launch num_static_obs:=2
\end{lstlisting}
This will launch a simulation environment (\autoref{fig: rviz_static}) with two static obstacles (Blue squares). 
\begin{figure}[h]
    \centering
    \includegraphics[width=0.6\textwidth]{lab3/figures/rviz_static.png}
    \caption{Simulated environment with two static obstacles}
    \label{fig: rviz_static}
\end{figure}

\subsection*{Task 1: Collision Avoidance with Static Obstacles} \label{task: task1}
\addcontentsline{toc}{subsection}{\textbf{Task 1: Collision Avoidance with Static Obstacles}}
Recall that in Lab 2, we have implemented a receding horizon planner inside \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23}{\textcolor{cyan}{\texttt{TrajectoryPlanner}}} class, we will add following features for this class:
\subsubsection*{Adding the subscriber for static obstacles}
\begin{enumerate}
    \item Within your \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23}{\textcolor{cyan}{\texttt{TrajectoryPlanner}}} class, read the \textbf{topic name} of static obstacles from the ROS parameter \texttt{"$\sim$static\_obs\_topic"} using the helper function \texttt{get\_ros\_param} and setting the default parameter as \texttt{"/Obstacles/Static"}.
    
    \item Subscribe to the topic from step 1, with message type \href{http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html}{\texttt{MarkerArray}}. This message contained a list of obstacles represented by a Marker.\\
    
    Hint: You can use \texttt{rosmsg show visualization\_msgs/MarkerArray} to inspect the data structure of \texttt{MarkerArray} message.
    
    \item Initialize an empty \textbf{dictionary} (let's call it `static_obstacle_dict}) as a \href{https://www.tutorialspoint.com/python/python_classes_objects.htm}{class variable}. %i.e. a variable that is shared by all instances of a class (in this case, it is your \texttt{TrajectoryPlanner}).
    
    \item Create a callback function for the subscriber. Inside this callback function, we retrieve \textbf{id} and \textbf{vertices} for each obstacle using \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/utils/static_obstacle.py#L5}{\textcolor{orange}{\texttt{get\_obstacle\_vertices}}} helper function. Then, \textbf{add vertices to `static_obstacle_dict} whose key is the id of the obstacle}.
    
    \item \textbf{(Optional)} Feel free to implement any reset strategies for the dictionary inside your callback function. For example, you can clear the dictionary every time the callback function is called or clear it every few seconds.
\end{enumerate}
\subsubsection*{Passing static obstacles into ILQR}
Inside the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L409}{\textcolor{orange}{\texttt{receding\_horizon\_planning\_thread}}} function, 
\begin{enumerate}
    \item At each time before replanning, initialize an empty List (let's call it `obstacles_list}).
    \item Append all \textbf{values} from `static_obstacle_dict} into `obstacles_list}.
    \item Pass `obstacles_list} into ILQR planner using \texttt{update\_obstacles} function.
\end{enumerate}
\subsubsection*{Testing obstacle avoidance}
Now re-launch ROS nodes and select a goal point on the map. 
\begin{lstlisting}[language=bash]
roslaunch racecar_planner lab3_task1.launch num_static_obs:=2
\end{lstlisting}
The default parameter should be able to handle most static obstacles. If the robot is running off the corner, you will need to restart the simulation. If your robot is stuck and you have implemented a reset strategy in the optional Step 5, you can reset static obstacles using RQT (\autoref{fig: rqt_reset}). 
\begin{figure}[h]
    \centering
    \includegraphics[width=0.5\textwidth]{lab3/figures/rqt_reset.png}
    \caption{Reset static obstacles by 1) selecting \textcolor{red}{\texttt{/simulation/reset\_static\_obstacle}} from drop-down menu; 2) entering numbers of static obstacles into the \textcolor{orange}{service expression}; 3) clicking the \textcolor{blue}{\texttt{Call}} button to send the service.}
    \label{fig: rqt_reset}
\end{figure}


\newpage
\section{Dynamic Obstacles}

In addition to static obstacles, we must consider other agents as dynamic obstacles and avoid collision with them. While we are unsure where other agents can be in the future, we can use forward reachable sets $\reach{}{\tdisc}$ to model all possible future states and avoid them at each time step. 

% For simplicity, we consider other agents identical to our robot truck. Forward reachability analysis enables us to consider all possible states that the other agent will be in the future. Then, we can treat the forward reachable set (FRS) $\reach{}{\tdisc}$ at each time instant as a static obstacle. We can use the same method in the previous section to incorporate the FRS information into the ILQR planner.

\paragraph{Worst-Case Analysis.}
We can compute the worst-case FRS concerning any possible controls. By avoiding FRSs at every time step within our planning horizon, your robot can avoid collision for any actions taken by other agents. However, this can make our planned trajectory very conservative and inefficient. For example, Fig.~\ref{fig:frs_worst} shows the evolution of worst-case FRS. We can observe that worst-case FRS grows rapidly and occupies the entire road.

\begin{figure}[!ht]
    \centering
    \includegraphics[width=15cm]{lab3/figures/frs_ol.pdf}
    \caption{The evolution of worse-case forward reachable set.}
    \label{fig:frs_worst}
\end{figure}


\paragraph{FRS with Predicted Policy.}
Worst-case reachability analysis often leads to overly conservative planning. Thodeus, if we can acquire information about other agents' behavior, it is useful to incorporate it into our planning algorithm.
Suppose we have computed an estimate of another agent's control policy\footnote{For example, we may have learned an estimate of the agent's preferences, expressed as a cost function and then computed an ILQR policy for this cost.} $\policy^\other \colon \xSet \to \cSet^\other$.
We assume the uncertainty in other agent's behavior is well represented by an additive disturbance term $\dstb^\other_\tdisc$, i.e.,
\begin{equation}
    \state^\other_{\tdisc+1} = \dyn \big(\state^\other_\tdisc, \policy^\other(\state^\other_\tdisc) \big) + \dstb^\other_\tdisc. \label{eq:dstb_dyn_other}
\end{equation}
In this case, by avoiding FRSs at every time step within our planning horizon, the robot can safeguard against all possible disturbances.

\subsection{Linear System Approximation}
We can use a simplified dynamical model to describe the motion of other agents. Assuming the agent follows a reference path and maintains a constant velocity, its continuous state-space model is:
\begin{equation}\dot{X} = AX+Bu=
    \begin{bmatrix}
    \dot{\hat{x}}\\ \dot{\hat{y}} \\\dot{v}_x\\ \dot{v}_y \\ \dot{v}_{ref}
    \end{bmatrix} = \begin{bmatrix}
        0 & 0 & 1 & 0 & 0\\
        0 & 0 & 0 & 1 & 0\\
        0 & 0 & 0 & 0 & 0\\
        0 & 0 & 0 & 0 & 0
    \end{bmatrix}
    \begin{bmatrix}
    {\hat{x}}\\ {\hat{y}} \\{v}_x\\ {v}_y \\v_{ref}
    \end{bmatrix} + \begin{bmatrix}
        0 & 0 \\
        0 & 0 \\
        1 & 0\\
        0 & 1 \\
        0 & 0
    \end{bmatrix}\begin{bmatrix}
        a_x\\a_y
    \end{bmatrix},  \label{eq: dyn_sys}
\end{equation}
where $\hat{x}$ and $\hat{y}$ are longitudinal and lateral position along the reference path, $v_x$ and $v_y$ are longitudinal and lateral velocity, $a_x$ and $a_y$ are longitudinal and lateral acceleration, and $v_{ref}$ is the reference longitudinal velocity. The agent applies a simple feedback control policy:
\begin{equation}
    u = \begin{bmatrix}
        a_x\\a_y
    \end{bmatrix}=\begin{bmatrix}
        -K_{vx}(v_x-v_{ref})+d_x\\-K_y \hat{y}-K_{vy}v_y+d_y
    \end{bmatrix} = -\begin{bmatrix}
        0 & 0 & K_{vx} & 0 & -K_{vx}\\
        0 & K_y & 0 & K_y & 0 
    \end{bmatrix}X + \begin{bmatrix}
        d_x\\d_y
    \end{bmatrix}= -KX+d, \label{eq: control}
\end{equation}

 Putting \autoref{eq: dyn_sys} and \autoref{eq: control} together, we have a new feedback control system as: 
 \begin{equation}
     \dot{X} = (A-BK)X+Bd
 \end{equation}
 Using this formulation, we can obtain the FRS of other agents in \href{https://fjp.at/posts/optimal-frenet/#:~:text=to%20the%20controller.-,Frenet%20Coordinates,road%20or%20a%20reference%20path.}{Frenet coordinate}, which can be transformed into Cartesian coordinate easily. For example, the FRS with predicted policy can be seen in \autoref{fig:frs}.
This forward reachable set does not over-grow as timestep increases because our feedback policy can stabilize the system despite the disturbance.
 \begin{figure}
     \centering
     \includegraphics[width=0.7\textwidth]{lab3/figures/FRS.png}
     \caption{20 Steps forward reachable sets with predictive policy projected to $\hat{x}-\hat{y}$ plane}
     \label{fig:frs}
 \end{figure}

 \subsection*{Task 2: Multi-step Forward Reachable Set} \label{task: task2}
\addcontentsline{toc}{subsection}{\textbf{Task 2: Multi-step Forward Reachable Set}}
 Inside the file \texttt{ROS\_Core/src/Labs/Lab3/scripts/frs.py}, we have implemented the majority of functionalities to compute FRS in \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/scripts/frs.py}{\textcolor{cyan}{\texttt{FRS}}} class. For example, given a set, $A$ and $B$ matrices to represent dynamics, bounds of control/disturbance, and time step $d_t$, \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/scripts/quickzonoreach/zono.py#L17}{\textcolor{orange}{onestep\_zonotope\_reachset}} function will calculate the FRS after $d_t$ seconds.\\

 You task is to finish \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/scripts/frs.py#L10}{\textcolor{orange}{multistep\_zonotope\_reachset}} function in the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/scripts/frs.py}{\textcolor{cyan}{\texttt{FRS}}} class following instructions. This function will calculate multiple-step reachable sets given an initial set.\\

 Finally, you can use \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/scripts/task2.ipynb}{\texttt{ROS\_Core/src/Labs/Lab3/scripts/task2.ipynb}} to reproduce the \autoref{fig:frs}.


\newpage
\subsection*{Task 3: Collision Avoidance with Dynamic Obstacles} \label{task: task2}
\addcontentsline{toc}{subsection}{\textbf{Task 3: Collision Avoidance with Dynamic Obstacles}}
In Task 3, we will first create a new ROS node to host ROS Service Server that calculates the FRS. We will implement this node in  \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab3/scripts/dyn_obstacle_node.py}{\texttt{ROS\_Core/src/Labs/Lab3/scripts/dyn\_obstacle\_node.py}} file. Specifically, we will:
\begin{enumerate}
    \item Create a subscriber to get poses of other agents;
    \item Create a Dynamic Reconfigure Server to allow you to adjust FRS parameters on the fly; 
    \item Create a ROS service Server with the name \texttt{`/obstacles/get\_frs'} to obtain other agents' FRSs on demand.
    \item Initialize the ROS node to start the service server you just created.
\end{enumerate}
Detailed instructions can be found in doc-strings. You will find ROS's official tutorials helpful for this task.
\begin{itemize}
    \item \href{http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile}{How to Write Your First cfg File}
    \item \href{http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29}{Setting Up Dynamic Reconfigure For A Node in Python}
    \item \href{http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29}{Writing a Simple Service and Client in Python}
\end{itemize}

Next, you \textbf{must also write a ROS Service Client} inside your trajectory planner. The general workflow is:
\begin{enumerate}
    \item Create a \href{http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29#rospy_tutorials.2FTutorials.2FWritingServiceClient.Writing_the_Client_Node}{client} for \texttt{`/obstacles/get\_frs'} service when the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23}{\textcolor{cyan}{\texttt{TrajectoryPlanner}}} class is initialized.
    \item Create a publisher (let's call it `frs_pub}) to publish FRS information for visualization when the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L23}{\textcolor{cyan}{\texttt{TrajectoryPlanner}}} class is initialized. This publisher publishes \texttt{MarkerArray} messages to the \texttt{`/vis/FRS'} topic.
    \item Inside the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/traj_planner.py#L409}{\textcolor{orange}{\texttt{receding\_horizon\_planning\_thread}}} function, call the service client you created in Step 1. For example, you can do 

    \begin{lstlisting}[language=python]
    request = t_cur + np.arange(self.planner.T)*self.planner.dt
    response = Your_Service_Client(request)
    \end{lstlisting}
    
    \item Then process the response of your service call using the helper function \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/utils/dyn_obstacle.py#L7}{\textcolor{orange}{\texttt{frs\_to\_obstacle}}}. The output of this helper function need to be \textbf{extended} into the `obstacles_list} (the same list you are using for Task 1) before sending it to the ILQR planner. \\

    \textbf{Hint}: See \href{https://www.geeksforgeeks.org/append-extend-python/}{append() and extend() in Python} to learn more about their difference.
    \item Use the the helper function \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/Lab2/scripts/utils/dyn_obstacle.py#L26}{\textcolor{orange}{\texttt{frs\_to\_msg}}} to generate visualization messages of FRSs. Publish the message with `frs_pub} that you created in step 2.
\end{enumerate}

Finally, you can test your collision avoidance by launching ROS nodes:
\begin{lstlisting}[language=bash]
roslaunch racecar_planner lab3_task2.launch
\end{lstlisting}
If everything works properly, you will see your robot moving around the track and avoid collisions with other agents.

\begin{figure}[h]
    \centering
    \includegraphics[width=0.8\textwidth]{lab3/figures/task2.png}
    \caption{Example result of Task 2}
    \label{fig:task2}
\end{figure}

You can also use RQT (\autoref{fig:task2_rqt} to adjust FRS parameters, as described in the previous sections. What will happen if you increase $d_x$ and $d_y$ and set all $K$ terms to 0? Please discuss with your TA about your observations.
\begin{figure}[h]
    \centering
    \includegraphics[width=0.6\textwidth]{lab3/figures/rqt_dyn_obs.png}
    \caption{You can use RQT to setup Dynamic Reconfigure Parameters for FRS}
    \label{fig:task2_rqt}
\end{figure}

\appendix \section*{Appendix}\label{appendix}
% The figure of full node graph showing changes from lab 2

\begin{sidewaysfigure}[ht]
    \includegraphics[width=\textwidth]{lab3/figures/lab3_new_nodes.png}
    \caption{New nodes and topics in lab 3}
    \label{fig: lab3_new_nodes_full}
\end{sidewaysfigure}

\begin{sidewaysfigure}[ht]
    \includegraphics[width=\textwidth]{lab3/figures/lab3_new_nodes_highlight.png}
    \caption{New nodes and topics in lab 3}
    \label{fig: lab3_new_nodes_full_highlight}
\end{sidewaysfigure}