# TODO: PUT EVERYTHING IN HERE!

# HERE IS INTRO TO ROS
\documentclass[12pt]{article}
\input{style}
\input{notation}

% \rhead{Due 12:00 PM\\ Feb. 10, 2023}
\chead{\textbf{Pre-Lab Tutorial: Intro to ROS}}

\begin{document}
\looseness=-1
Welcome to the Robotics Assignment (``Lab'') component of Intelligent Robotic Systems!\\

Over the semester, we will implement various methods for robot decision-making, both in simulated environments and on physical robotic hardware. In the upcoming ``zeroth'' lab, we will learn the key concepts of ROS (the Robot Operating System) to control the Mini Trucks that will accompany us throughout the class.\\
% create workspaces, packages, nodes, publishers and subscribers.\\

This is a Pre-Lab Tutorial to help you get ready for our labs. Although this material is not graded, we strongly encourage you to walk through it and try out all commands on your own computer before your first lab section. It will make things much smoother! The following are the objectives of this Pre-Lab Tutorial:
\begin{itemize}
    \item Install ROS on your own computer.
    \item Get familiar with basic ROS concepts.
    \item Be able to build and run a provided ROS package. 
\end{itemize}
\tableofcontents

\newpage

\input{lab0/intro_to_ros_1}


%\bibliography{reference.bib}
\end{document}

\section{Setting Up ROS}
Before we get started, you need to set up the Git Repository and configure your computer for the lab. Please read through the detailed instructions \href{https://github.com/SafeRoboticsLab/ECE346}{here}.   


\section{Intro to ROS}

Here, we will go over some basic concepts of ROS (which stands for ``Robot Operating System''). ROS is an open-sourced framework for controlling robotic components from a computer. You can generally think of ROS as a graph or network of independent \textbf{ROS nodes}. Each ROS node communicates via \textbf{ROS messages} by \textbf{publishing} and \textbf{subscribing} to \textbf{ROS topics}. Published messages will be received by any node in the graph subscribed to the corresponding \textbf{ROS topic}.\\

Most roboticists today create ROS software using either C++ (\href{http://wiki.ros.org/roscpp}{roscpp}) or Python (\href{http://wiki.ros.org/rospy}{rospy}), as both languages are well-supported in the ROS community. In this class, our default working language is Python. However, you will find that it is simple to adapt ROS in the other language once you master one of them. 

\subsection{Key Concepts in ROS}
\begin{itemize}

    \item\textbf{\href{http://wiki.ros.org/Master}{ROS Master}}\\
    The ROS Master serves as the central coordinator of the ROS system. It provides naming and registration services to the rest of the nodes in the ROS system. It tracks publishers and subscribers to topics and services. 
    \item\textbf{\href{http://wiki.ros.org/Nodes}{ROS Nodes}}\\
    A node is a process that performs some computation. For example, a robot with a control system will usually be comprised of many nodes. A robot can have many nodes, each meant to operate at a fine-grained scale. For example, a node for processing camera images, a node that controls the robot's motors, a node that performs localization, a node that performs path planning, etc.
    \item\textbf{\href{http://wiki.ros.org/Messages}{ROS Message}}\\
    Communications between ROS nodes are through ROS messages. A ROS message is a simple data structure, comprising integers, floating-point numbers, booleans, as well as arrays of those data types defined by a \texttt{.msg} file. Messages can include arbitrarily nested structures and arrays as defined by the user.
    
    \item\textbf{\href{http://wiki.ros.org/Topics}{ROS Topics}}\\
    ROS nodes communicate with one another by publishing and subscribing to topics that contain messages. The ROS topics provide ID to the correct channel of communication.
    \item\textbf{\href{http://wiki.ros.org/Services}{ROS Services}}\\
    A ROS service is a type of message that allows two-way communication.  This may be necessary for robotics applications where you need to change the robot's mode and receive acknowledgment of receiving the request.  This process can be accomplished by using ROS services which depends on \texttt{.srv} files.
    
    \item\textbf{\href{http://wiki.ros.org/catkin/conceptual_overview}{Catkin Build System}}\\
    A build system compiles source code and creates executable programs.  ROS has a custom build system called catkin.  Catkin was created to alleviate the complexity of using existing build tools.
    \item\textbf{\href{http://wiki.ros.org/catkin/workspaces}{ROS Catkin Workspace}}\\
    A catkin workspace is a directory used to modify, build, and install multiple catkin packages.  Anything we do with ROS will be inside of a catkin workspace.
    \item\textbf{\href{https://wiki.ros.org/Packages}{ROS Catkin Package}}\\
    A catkin package is a directory that contains source code for your ROS nodes, descriptions for your custom ROS messages and services, or other libraries used in ROS. 

    \item \textbf{\href{http://wiki.ros.org/Parameter_Server}{Parameter Server}}\\
A parameter server is a shared, multi-variate dictionary, which is accessible to all nodes to store and retrieve data by keys from at runtime.
    
\end{itemize}

\begin{figure}[h]
    \centering
    \includegraphics[width=0.7\textwidth]{lab0/figures/ROS_graph.JPG}
    \caption{A ROS graph example containing two nodes}
    \label{fig:ros_graph}
\end{figure}

Putting those terms together, let us consider the example in \autoref{fig:ros_graph}. We have a ROS master with two registered nodes: the camera node and the image processing node. The camera node talks to the camera through an API that is independent of ROS. It publishes an image message to the\texttt{/image} topic. The image processing node is subscribed to the \texttt{/image} topic and receives the image message for further processing. Additionally, let's assume the camera costs a lot of energy to run and should only be used when needed. We can use the ROS service \texttt{/Turn\_On\_Camera} to ask the camera node to turn on the camera. The image processing node can send this service request, and it begins computation once the camera sends back the response. 
\\

The main advantage of ROS is that it is a modular system.  What if we want to have another process to do localization based on camera images? Using ROS, we can simply set up a new localization node that subscribes to the same \texttt{/image} topic, without the need to change code in the other two modules. Similarly, if we deploy the system to another robotic platform with a different camera, we just need to make sure the new camera node publishes the same type of image messages, and keeps using other downstream packages.  

\section{ROS Catkin Workspace}
A catkin workspace is a directory (folder) that contains all catkin packages.  You can think of this as the main folder that contains everything you need for a specific project or lab.  A catkin workspace can store multiple catkin packages and allows us to build all of the catkin packages at the same time. We will learn more about the build process later in this reading, but we must build a workspace to convert source code into executable files.  If you don't use a catkin workspace, you can build catkin packages independently, however, this can be very tedious when there are several packages that need to be built. \\

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Checkpoint 1 START%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
First, open a terminal and go into to your desired directory.  Then, let's activate our ROS environment by  
\begin{lstlisting}[language=bash]
    # Activate ros_base environment. Note: the name can vary
    conda activate ros_base
\end{lstlisting}

\subsection{Create a Catkin Workspace}
Begin by creating a directory labeled $\texttt{first\_catkin\_ws}$ with a source sub-directory labeled \texttt{src}:

\begin{lstlisting}[language=bash]
    # Create a workspace labeled catkin_ws with a src sub-directory
    mkdir -p first_catkin_ws/src 
\end{lstlisting}

While using the command \texttt{mkdir}, the \texttt{-p} flag allows us to create sub-directories simultaneously with the main directory.\\

\textit{Note}: Alternatively, we can do the same thing in three separate steps:

\begin{lstlisting}[language=bash]
    # Create a directory labeled first_catkin_ws 
    mkdir first_catkin_ws
    # Navigate indside the first_catkin_ws 
    cd first_catkin_ws
    # Create a directory labeled src
    mkdir src
\end{lstlisting}

The directory labeled $\texttt{first\_catkin\_ws}$ is the catkin workspace.  Within the catkin workspace, we have a directory labeled \texttt{src} which represents the source space and will be discussed later in the lab.  The \texttt{src} directory is \textbf{necessary} to build code in the catkin workspace. \\

\section{ROS Catkin Package}
As previously mentioned, a catkin package refers to a directory that contains source code for ROS nodes, services, messages, etc.  Catkin packages are located inside the catkin workspace.   under the \texttt{src} sub-directory. A given project workspace will likely contain many packages inside \texttt{src}. \\

While ROS provides a tool to create a catkin package using the command \href{http://wiki.ros.org/ROS/Tutorials/CreatingPackage}{\texttt{catkin\_create\_pkg}}, we have provided a template package called \texttt{first\_pkg} \textbf{in our Git Repo} under the catkin workspace within the \texttt{src} directory (\texttt{catkin\_ws/src}) for this tutorial and also for your future labs.\\

% To create a $\texttt{catkin\_package}$ we will use the command $\texttt{catkin\_create\_pkg}$. The basic format for creating a catkin package follows the form:

% \begin{lstlisting}[language=bash]
%     catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
% \end{lstlisting}

% For our catkin package, we will start by navigating to the \texttt{src} directory of our catkin workspace (catkin packages are always stored in the \texttt{src} directory of the catkin workspace).  We will create a package called $\texttt{catkin\_pkg}$ with the Python dependency \texttt{rospy}. 

% \begin{lstlisting}[language=bash]
%     # Navigate to the catkin workspace source folder
%     cd ~/catkin_ws/src
%     # Create a catkin package labeled catkin_pkg w/ a rospy depend.
%     catkin_create_pkg catkin_pkg rospy
% \end{lstlisting}

% The command $\texttt{catkin\_pkg}$ creates an essential foundation for a catkin package including the \texttt{package.xml} and \texttt{CMakeLists.txt} files.  Additionally, a \texttt{src} directory is produced within the catkin package directory for the purposes of containing Python packages and C++ code.  

Let's first inspect what's inside the $\texttt{first\_pkg}$ catkin package. A typical file tree of a catkin package can be seen in Figure \ref{fig:first_pkg}.

\begin{figure}[h]
    \centering
    \includegraphics[width=0.4\textwidth]{lab0/figures/first_pkg.png}
    \caption{File tree for the $\texttt{first\_pkg}$ catkin package}
    \label{fig:first_pkg}
\end{figure}

The \texttt{launch} directory has launch files for the purpose of launching one or multiple nodes within the package.\\

The \texttt{scripts} directory holds executable python scripts that contain nodes and their dependency packages.\\

The \texttt{src} directory is for users who prefer to use C++. 
 This is where C++ users will place their source code.\\

The \texttt{package.xml} is the manifest file containing metadata about a package. It includes the package's name, version, description, license information, dependencies, and other meta information like exported packages. The detailed requirements of a \texttt{package.xml} is listed in this \href{http://docs.ros.org/en/melodic/api/catkin/html/howto/format2/index.html}{documentation}.\\

The \texttt{CMakeLists.txt} file is the input to the CMake build system for building software packages. In ROS, the \texttt{CMakeLists.txt} file tells ROS which files need to be built and what packages need to be linked. You are still required to have a \texttt{CMakeLists.txt} file, even if the entire ROS package is written in Python. Luckily, ROS has a very detailed template for the \texttt{CMakeLists.txt} file, and we will provide the proper files for you during this class. To learn more about this, please check out this \href{http://wiki.ros.org/catkin/CMakeLists.txt}{documentation}.\\

For now, we do not need to worry about \texttt{package.xml} and \texttt{CMakeLists.txt} as they will be provided for most of the packages in this class. Later in the semester, we will dive into those concepts while building more open-ended software for our final projects.

\subsection{Building a Catkin Package}

From Section 1 (Setup ROS), you should have downloaded the class GitHub repository.  Using the terminal you can to navigate to the ECE346 directory ($\texttt{ECE346}$) and then navigate to the catkin workspace ($\texttt{catkin\_ws}$).\\

Now, let's build the code in the catkin workspace using the ROS command $\texttt{catkin\_make}$. The command $\texttt{catkin\_make}$ should be executed at the top level of your catkin workspace.


\begin{lstlisting}[language=bash]
    # Navigate to the top level of your catkin workspace
    cd ECE346/catkin_ws
    # Build catkin workspace
    catkin_make
\end{lstlisting}

After using $\texttt{catkin\_make}$, a lot of messages are printed in your console. If everything went well, you should see something similar to:
\begin{lstlisting}[language=bash]
        .
        .
        .
        .
    Configuring done
    -- Generating done
    -- Build files have been written to: XXXXX/catkin_ws/build
    ####
    #### Running command: "make -j12 -l12" in "XXXXX/catkin_ws/build"
    ####
\end{lstlisting}

After using $\texttt{catkin\_make}$, you should notice that your catkin workspace contains two new sub-directories labeled $\texttt{build}$ and $\texttt{devel}$ and a $\texttt{CMakeLists.txt}$ file in the \texttt{src} directory.  Because \texttt{build} and \texttt{devel} are generated by \texttt{catkin\_make}, and you do not typically have to worry about them. \\

A catkin workspace can be broken down into three separate parts: source (\texttt{src}) directory, build (\texttt{build}) directory, and development (\texttt{devel}) directory.  A typical file tree for a catkin workspace with a catkin package can be seen in Figure \ref{fig:catkin_ws}.  Let's break down the purpose of the source directory, build directory, and development directory. 

\begin{figure}[h]
    \centering
    \includegraphics[width=0.5\textwidth]{lab0/figures/catkin_ws.png}
    \caption{File tree for a catkin workspace with a catkin package after using $\texttt{catkin\_make}$}
    \label{fig:catkin_ws}
\end{figure}

 
 \begin{itemize}
\item\textbf{Source (\texttt{src}) Directory}\\
    The \texttt{src} directory is the home for all catkin packages.  It contains all source code, and it is where we will soon create nodes using Python.
\item\textbf{Build (\texttt{build}) Directory}\\
    The \texttt{build} directory is the location where CMake builds all of the code from the \texttt{src} directory. It keeps catkin and CMake's cache information and other intermediate files there.
\item\textbf{Development (\texttt{devel}) Directory}\\
    The \texttt{devel} directory is the location for the built source code. 

\end{itemize}


\section{ROS Master and Node}
Simply put, a ROS node is a process that performs computation. Nodes are combined together into a graph and communicate with one another using streaming messages through topics, sending requests through services, and setting values through Parameter Server.  ROS nodes can be written in Python (rospy) or C++ (roscpp) using the ROS client libraries.  

\subsection{Making Node Executable}

In the last section, we discussed how to build catkin packages using \texttt{catkin\_make}. In order to add the workspace to the ROS environment you need to navigate to the top level of your catkin workspace \texttt{catkin\_ws} and \texttt{source} the \texttt{setup.bash} file. For users of \textit{zsh} (default for Mac OS), you should use file \texttt{devel/setup.zsh} instead.



\begin{lstlisting}[language=bash]
    # Navigate to the top level of your catkin workspace (eg: catkin_ws)
    cd ECE346/catkin_ws
    # Add workspace to ROS environment
    source devel/setup.bash
\end{lstlisting}

\textbf{Important}: You need to use the $\texttt{catkin\_make}$ command every time you add a new catkin package to your catkin workspace, define a new message type or build a new service for Python users. However, it is good practice to use $\texttt{catkin\_make}$ whenever you make any changes to elements in the catkin workspace. You will also need to do \texttt{source devel/setup.bash} every time you open a new terminal window to use your packages under the workspace.\\

\subsection{Running ROS Nodes with \texttt{rosrun}}
In $\texttt{first\_pkg}$, the \texttt{scripts} directory has a Python script for the ROS node ($\texttt{first\_node.py}$). To run this node, let us first open a ROS Master by
\begin{lstlisting}[language=bash]
    # Start a ROS Master
    roscore
\end{lstlisting}
Figure \ref{fig:ros_core} is an exemplary output you will see from your terminal.\\
\begin{figure}[h]
    \centering
    \includegraphics[width=0.5\textwidth]{lab0/figures/ros_core.png}
    \caption{Example of \texttt{roscore} output}
    \label{fig:ros_core}
\end{figure}

Then, we open a new terminal, activate \texttt{ros\_base} environment, navigate to your catkin workspace, and add environment variables.
\begin{lstlisting}[language=bash]
    # Activate ros_base
    conda activate ros_base
    # Navigate to the top level of your catkin workspace
    cd catkin_ws
    # Add ROS env variables
    source devel/setup.bash
\end{lstlisting}
Now, in this terminal, we can run our nodes by 
\begin{lstlisting}[language=bash]
    # run the node
    # rosrun <pkg_name> <node_file>
    rosrun first_pkg first_node.py
\end{lstlisting}
You should see 'Hello World' printed out continuously. To stop the ROS process, you need to first terminate your node (\texttt{ctrl-C}), then turn off your ROS Master (\texttt{ctrl-C}).

\subsection{Running ROS Nodes with \texttt{roslaunch}}
This is so cumbersome! However, recall that the $\texttt{first\_pkg}$ (\autoref{fig:first_pkg}) also contains a \texttt{launch} directory with the file ($\texttt{first\_launch.launch}$). The key idea here is to start the ROS Master, run one or multiple nodes, and handle parameters using one single command. We will learn how to create our own \texttt{.launch} files in future labs. Using \texttt{roslaunch} let's run our nodes without opening hundreds of terminals.

% Using \texttt{roslaunch}, greatly simplifies the process of running nodes by automatically starting up the ROS master (centeral coordinator of the ROS system). It also allows you to easily define parameters and start multiple nodes simultaneously with a singular \texttt{.launch} file. \\
\begin{lstlisting}[language=bash]
    # Launch our first node 
    # rosrun <pkg_name> <launch_file>
    roslaunch first_pkg first_launch.launch
\end{lstlisting}

You should see exact same results you saw with \texttt{rosrun}. When finished, press \texttt{ctrl-C}, and all processes will be terminated. A detailed guide of the launch file can be found \href{http://wiki.ros.org/roslaunch}{here}.


\section{References and Additional Materials}
Over the previous sections, we have covered a tiny portion of what ROS offers. Throughout the semester, we will learn more topics while implementing exciting algorithms on robots. We also encourage you to go over some of the excellent ROS tutorials and examples available online. You may find these materials very useful for gaining a deeper and more advanced understanding of ROS. Below are a few pointers to get you started on your ROS journey.
\begin{itemize}
        \item \href{http://wiki.ros.org/Documentation}{Official ROS documentation}
        \item \href{http://www.clearpathrobotics.com/assets/guides/melodic/ros/}{ROS tutorial from Clearpath Robotics}
        \item \href{https://nu-msr.github.io/me495_site/}{ROS lecture notes from ME495 at Northwestern University}
        \item \href{https://www.cse.sc.edu/~jokane/agitr/agitr-letter.pdf}{A Gentle Introduction to ROS by Jason M. O’Kane}
\end{itemize}
\looseness=-1
If you encounter issues, bugs, or unsolvable puzzles, your best helper is always Google (much wiser than any one of us). If your questions are ROS-related or you are unsure how to achieve some advanced features, you can check \href{https://answers.ros.org/questions/}{ROS Answers}, where you are most likely to find solutions to your problems.



# Lab 0 - Introduction to ROS and Mini Truck

## Overview & Goals
1. Get familar with the hardware of the truck:
    - Turn the truck on and off.
    - Remote control the truck.
    - Replace the battery.
2. Get familar with the software interface of the truck:
    - Connect to the truck via SSH.
    - Run the SLAM and controller nodes.
    - Play around with RViz visualization and the QRT control panel.
4. Learn basic ROS sub/pub:
    - Subscribe to Odometry topic and publish control command.
5. Learn ROS parameter server:
    - Set the parameters from launch file and command line.
    - Set the parameters from yaml file.
6. Write a simple P controller to reach a goal.

## Get Started
1. Build the workspace from [`ROS_Core`](../..).
```
catkin_make
```
2. Source the workspace. If your terminal uses bash as the shell, use
```
source devel/setup.bash
```
Otherwise, if your terminal uses zsh, use
```
source devel/setup.zsh
```
3. Launch the ros packages and rviz visualization.
```
roslaunch lab0 lab0_simulation.launch
```
Click the **2D Nav Goal** button on the top panel of RViz, and click on the map to set the goal. You should see a green triangle on the map, representing the chosen goal. At the same time, in the terminal you should also see:
```
[INFO] [xxxx.xxx]: Received a new goal [nan, nan]
```

4. Write your code in [`scripts/controller/pure_pursuit.py`](scripts/controller/pure_pursuit.py) and finish all TODOs. Repeat step 3 to see if your controller works.
![](assets/example.png)

## Common Issues
This lab depends on several external packages, such as [pySpline](https://github.com/mdolab/pyspline) and [networkx](https://networkx.org/). If you encounter **Cannot find module** errors, try to use install those packages with this [script](/Host_Setup/RoboStack/install_dependency.sh). 
```
cd <Path of your repo>/Host_Setup/RoboStack
source install_dependency.sh
```


# HERE IS OLD LAB 0

\documentclass[12pt]{article}
\input{style}
\input{notation}

\rhead{Due 11:59 PM\\ Feb. 16, 2024}
\chead{\textbf{Lab 0\\ Intro to ROS}}

\begin{document}
\looseness=-1

Welcome to the ``zeroth'' lab of ECE 346!
This is a preliminary Robotics Assignment (or just Lab for short) in which we will learn the key concepts of ROS (the Robot Operating System) to control the Mini Trucks that will accompany us throughout the class.\\

In Lab 0, you will:
\begin{itemize}
    \item Get familiar with the visualization and simulation tools for this class.
    \item Get familiar with the mini-truck platform.
    \item Learn how to interface with ROS subscribers, publishers, and parameter servers.
    \item Learn how to run your own software on the Mini Truck.
    \item Develop and test a goal-reaching controller for your robot.
\end{itemize}

Prerequisites (from Pre-Lab 0). Before you get started with Lab 0, make sure you have:
\begin{itemize}
    \item Successfully installed ROS on your computer.
    \item Created a fork of the ECE346 GitHub repository. 
    \item Become familiar with the basic ROS concepts.
    % \item Become familiar with the Mini Truck hardware.
\end{itemize}
\tableofcontents

\newpage

\input{lab0/intro_to_ros_2}

%\bibliography{reference.bib}
\end{document}

\section{Getting Started}
In this Lab, you will implement a simple goal-reaching controller and apply your knowledge of ROS to make it run on both the simulation environment and your Mini Truck robot. We will use a proportional controller for the throttle, and a pure pursuit controller for steering.  You need to finish \textbf{6} tasks by filling in missing codes of file \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/lab0/scripts/controller/pure_pursuit.py}{\texttt{pure\_pursuit.py}} (file path: \texttt{<Path to your repo>ECE346/ROS\_Core/src/Labs/lab0/scripts/controller/pure\_pursuit.py}). \\

Before we dive into technical details, let's take a look at what is provided for this Lab. First, please make sure you have the latest version of the ECE346 code in your own private fork. If you need a refresher on how to do this, you can check \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/Docs/private_fork.md#update-your-fork-from-public-repo}{here}. Under our catkin workspace (\texttt{ROS\_Core}), we can build all packages, set up the environment, and launch our nodes, all from our terminal, as follows:
\begin{lstlisting}[language=bash]
    # Activate ROS environment
    conda activate ros_base
    # Navigate to the workspace
    cd <Path to your repo>/ECE346/ROS_Core
    # Build ROS packages
    catkin_make
    # Setup environment
    source devel/setup.bash # .zsh for Mac users
    # Launch Nodes
    roslaunch lab0 lab0_simulation.launch
\end{lstlisting}

\begin{figure}[h]
    \centering
    \begin{subfigure}[c]{0.49\textwidth}
         \centering
         \includegraphics[height=2.5in]{lab0/figures/lab0_rviz.png}
         \caption{Rviz visualization tool.}
         \label{fig: lab0_rviz}
     \end{subfigure}
     \hfill
     \begin{subfigure}[c]{0.49\textwidth}
         \centering
         \includegraphics[height=2.5in]{lab0/figures/rqt_sim.png}
         \caption{RQT GUI.}
         \label{fig:lab0_rqt_sim}
     \end{subfigure}
     \caption{Diagrams of the remote controller}
     \label{fig:lab0_interface}
\end{figure}

Two windows should pop up when you run the above roslaunch command. 
The first one (\autoref{fig: lab0_rviz}), is managed by an \href{http://wiki.ros.org/rviz}{\textbf{Rviz}} node.  In the RViz window, you should see an orange rectangle which represents your robot. Rviz will serve as the main visualization tool in our class. It is highly configurable, and we will introduce more functionalities (such as visualizing the map and planned routes) in future Labs. \\

The second window (\autoref{fig:lab0_rqt_sim}), is the \href{http://wiki.ros.org/rqt}{\textbf{RQT}} GUI. It is a versatile tool that allows you to inspect your ongoing ROS processes, send ROS messages and call ROS services, visualize data, etc. RQT is highly configurable.
You can \href{https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Creating%20RQT%20Dashboard.html}{adjust the layout and panels} and even \href{https://wiki.ros.org/rqt/Tutorials/}{create your own plugins}. \\

\begin{figure}[h]
    \centering
    \includegraphics[width=0.8\textwidth]{lab0/figures/lab0_rqt.png}
    \caption{Node graph of Lab 0 from RQT GUI}
    \label{fig:lab0_rqt}
\end{figure}

From the RQT GUI, let's first take a look at the node graph page. If the node graph is not shown on your GUI, you can add one from \textbf{Plugins} menu on the top of the panel. \autoref{fig:lab0_rqt} shows a node graph of Lab 0 of 6 nodes. The \texttt{/rosout} node starts automatically with ROS Master, and it logs messages to your console. The \texttt{/rviz}, \texttt{/visualization\_node} and \texttt{/rqt\_gui} nodes handle visualization and process monitoring.  The \texttt{/simulation\_node} simulates the dynamics of our robot after executing control commands from the \texttt{/lab0} node. All those nodes are started with a single \texttt{roslaunch} command. In the next section, we will take a look at the basic functionality of roslaunch.

\section{\texttt{roslaunch} Basics}
\texttt{roslaunch} is a tool for easily launching multiple ROS nodes, as well as setting parameters on the Parameter Server. \texttt{roslaunch} takes in one or more XML configuration files (with the .launch extension) that specify the parameters to set and nodes to launch. In Lab 0, you just need to know how to interpret a launch file and pass arguments during \texttt{roslaunch}.

\subsection{Reading a Launch File}
let's first \textbf{take a look} at the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/lab0/launch/lab0_simulation.launch}{launch file} you just used. You can click the link or find it locally from \texttt{<Path to your repo>ECE346//ROS\_Core/src/Labs/lab0/launch/lab0\_simulation.launch}.\\

The first section, with syntax \href{http://wiki.ros.org/roslaunch/XML/arg}{\lstinline{<arg name="AA"  default="BB"  doc="CC"/>}}, defines a list of arguments that you can pass into this launch file. \\

The second section \href{http://wiki.ros.org/roslaunch/XML/rosparam}{\lstinline{<rosparam command="load" file="$(find lab0)/configs/config.yaml"/>}} loads a list of parameters defined in a YAML file to the ROS parameter server. \lstinline{$(find lab0)} will ask ROS to find the path to the \texttt{lab0} package so that you do not need to type the absolute path. \\

The third section, with blocks enclosed by \href{http://wiki.ros.org/roslaunch/XML/include}{\lstinline{<include> xxxx </include>}}, specifies other launch files to include during this launch, effectively allowing nesting of launch files. In addition, the syntax \lstinline{<arg name="XX" value="$(arg AA)"/>} passes argument AA in this launch file into argument XX of the included launch file.\\

The last section, with blocks encolosed by \href{http://wiki.ros.org/roslaunch/XML/node}{\lstinline{<node> xxxx </node>}}, starts nodes defined in \texttt{lab0\_node.py}. Similar to the above section, the syntax \href{http://wiki.ros.org/roslaunch/XML/param}{\lstinline{<param name="YY" value="$(arg AA)"/>}} loads argument \textit{AA} from the first section into the ROS parameter server with name \textit{YY}.\\

We will not be writing our own launch file for this Lab. That said, you can find a complete guide to launch file syntax \href{http://wiki.ros.org/roslaunch/XML}{here} if you are interested in learning more.
\subsection{Passing Arguments During \texttt{roslaunch}}

To pass an argument, we can simply append \lstinline{Argument_Name:=Argument_Value} to your \texttt{roslaunch} command. For example, let's close our previously launched ROS nodes and try
\begin{lstlisting}[language=bash]
    roslaunch lab0 lab0_simulation.launch init_x:=1 init_y:=1
\end{lstlisting}
You will see the car is now starting at a different location.

\subsection{Getting ROS Parameters}
The argument you set in during \texttt{roslaunch} are read by your ROS nodes through \href{http://wiki.ros.org/rospy/Overview/Parameter%20Server}{ROS Parameter Server API}. In ECE346, we provided a wrapper around this API to search and load a parameter. This can be achieved by using the function
\begin{lstlisting}[language=python]
    get_ros_param(param_name, default_value)
\end{lstlisting}
You can find this function in \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/ROS_Core/src/Labs/lab0/scripts/controller/utils/ros_utility.py}{here}, and it will be included for all Labs.

\section{ROS Messages, Topics, Publishers and Subscribers}
One primary function of ROS is the communication between nodes using messages. The publisher sends out the message to ROS topics, and the subscribers receive the messages. This section will use two examples borrowed from the \href{http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)}{ROS official tutorial} to understand how publishers and subscribers work in ROS. For visualization, the node graph in  \autoref{fig:lab0_rqt} indicates which nodes are publishers or subscribers to a certain topic. For example, the \texttt{/lab0} node publishes to the \texttt{/Control} topic, and the \texttt{/simulation\_node} is subscribed to the \texttt{/Control} topic. 

\subsection{ROS Publisher}
First, let us look at the simple ROS publisher code below. In this Python code, we have a node named \texttt{talker} that sends messages to a topic named \texttt{chatter} at a rate of 10 HZ (every 0.1 seconds).

\begin{lstlisting}[style=python_env, xleftmargin=0.2in, caption=Code for publisher]
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher("chatter", String, queue_size=10)
    rospy.init_node("talker", anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world {}".format(rospy.get_time())
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
\end{lstlisting}

Now, let's break down the code.  

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
#!/usr/bin/env python
\end{lstlisting}

Every Python ROS Node will have this declaration at the top of the Python file. This line ensures your script is executed as a Python script.\\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
import rospy
from std_msgs.msg import String
\end{lstlisting}

You need to import \texttt{rospy} if you are writing a ROS node in Python. We also need to import our desired data type for ROS messages.  Here, we will import \texttt{String} from the standard \texttt{std\_msgs.msg} data type package.\\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
def talker():
\end{lstlisting}

Over the next few lines of code, we will define a \texttt{talker} node, and it's publishing capabilities.\\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
pub = rospy.Publisher("chatter", String, queue_size=10)
\end{lstlisting}

Here, we define a publisher node to publish messages to a desired topic.  It declares that we want to publish to the \texttt{chatter} topic, using a \texttt{String} message, with a $\texttt{queue\_size}$ argument of 10.  The $\texttt{queue\_size}$ limits the amount of queued messages for the case where a subscriber is not receiving the messages fast enough.  Here we are queuing 10 messages.\\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
rospy.init_node("talker", anonymous=True)
\end{lstlisting}

Now, we will initialize a ROS node in Python.  The function $\texttt{init\_node}$ tells the ROS master the name of your node.  The node will be Labeled as \texttt{talker}.  We will also set \texttt{anonymous = true}.  This allows the compiler to append numbers to the end of the node's name to ensure a unique node name (this is useful for more complex simulations). \\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
rate = rospy.Rate(10) # 10hz
\end{lstlisting}

The ROS rate function,  \href{http://wiki.ros.org/rospy/Overview/Time}{\texttt{rate()}}, defines the speed at which we want to perform some task.  Here we define a \texttt{rate} handle to maintain a desired speed of 10 Hz (0.1 seconds). Later, we will see that \texttt{rate} is used to define the amount of time that the system should sleep in the \texttt{while} loop.\\


\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
while not rospy.is_shutdown():
    hello_str = "hello world {}".format(rospy.get_time())
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
\end{lstlisting}

This \texttt{while} loop structure is fairly standard in \texttt{rospy}.  The \texttt{while} loop will begin to iterate after checking the $\texttt{is\_shutdown}$ flag. Generally, when we launch nodes, the $\texttt{is\_shutdown}$ flag is not activated, but when we terminate a node using \texttt{Ctrl-C} the $\texttt{is\_shutdown}$ flag is activated and the loop terminates. \\

 Inside the loop, we first define a string message called $\texttt{hello\_str}$ that will contain the text "hello world". (Note: $\texttt{\%s}$ concatenates string messages, and $\texttt{\%}$ $\texttt{rospy.get\_time}$ prints the current time).  
 Next, using the \texttt{loginfo()} function, the string message ($\texttt{hello\_str}$) will be printed in the terminal, written to the node's log file, and written to rosout (used for debugging). 
 Using  \href{http://wiki.ros.org/rospy/Overview/Publishers and Subscribers}{\texttt{pub.publish()}}, the string message is published to the \texttt{chatter} topic. Lastly, \texttt{rate.sleep()} is used to maintain a desired loop rate (we previously defined the loop rate as 10 Hz).\\


\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
\end{lstlisting}

This snippet of code is where we make everything happen.  It is the main block of the code which calls upon our previously set up definitions.  The main block includes the \texttt{talker()} definition and also includes a \texttt{ROSInterruptException} to prevent the code from continuing to execute during \texttt{sleep()}.

\subsection*{Task 1: Set up a publisher for the ServoMsg message}
\addcontentsline{toc}{subsection}{\textbf{Task 1: Set up a subscriber for the ServoMsg message}}
Now you know how to publish a ROS message. Let's write our first ROS code!  Open your \texttt{pure\_pursuit.py} file in the text editor of your choice (file path: \texttt{<Path of your
repo>ECE346/ROS Core/src/Labs/Lab0/scripts/controller/pure\_pursuit.py)} Your first task is to set up a missing publisher in the function \texttt{setup\_publisher} following instructions under \textbf{TODO}.  Make sure you read through the code to get an understanding of variable names (e.g topic name). Once you are finished, show your code to the TA and proceed.

\subsection{ROS Subscriber}
The code for the subscriber is very similar to the publisher and can be seen below.  Now, instead of publishing to the \texttt{chatter} topic, we are subscribing to it.\\

\begin{lstlisting}[style=python_env, xleftmargin=0.2in, caption=Code for subscriber]
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)
    
def listener():
    # Define listener node
    rospy.init_node("listener", anonymous=True)
    # Subscribe the listener node to chatter topic
    rospy.Subscriber("chatter", String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    listener()
\end{lstlisting}

Let us now break down the subscriber code.
\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
\end{lstlisting}

Exactly like the publisher code, we need to setup our Python script and import necessary packages.\\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
def callback(data):
    rospy.loginfo("I heard %s", data.data)
\end{lstlisting}

Now, we will define a \texttt{callback(data)} function.  This function is used to process the received message data. Here, \texttt{loginfo()} is used to print to the terminal.  The printed messages will begin with the text \texttt{"I heard"}  and will then print out the message data received from the chatter topic (\texttt{data.data}).  This function should make more sense when we discuss the subscriber. \\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()
\end{lstlisting}
Here, we define the \texttt{listener} node, also known as the node that is subscribed to the \texttt{chatter} topic.  First, the node is initialized so that the master knows the name our new node.\\  

Next, the \texttt{listener} is defined as a subscriber.  There are three important categories we need to specify when using the \texttt{Subscriber()} function.  First, we need to declare the topic that we want to subscribe to.  Here we are subscribing to the \texttt{chatter} topic.  Second, we need to identify the data type of the ROS message.  The data type for the message of the publisher and subscriber needs to be the same. Therefore the data type of the message will be a \texttt{String}.  Third, we need to identify the name of the function where the message data will be sent.  In our case, we are sending the message data to the \texttt{callback()} function.\\

The \texttt{spin()} function keeps the node active until it is manually shut down (\texttt{Ctrl-C}).\\

\begin{lstlisting}[style=python_env2, xleftmargin=0.2in]
if __name__ == "__main__":
    listener()
\end{lstlisting}

This last snippet of code is where we actually run the listener code. 


\subsection*{Task 2: Set up a subscriber for the Odometry message}
\addcontentsline{toc}{subsection}{\textbf{Task 2: Set up a subscriber for the Odometry message}}
Open your \texttt{pure\_pursuit.py} file.  Your second task is to set up a missing subscriber in the function \texttt{setup\_subscriber} following instructions under \textbf{TODO}. Once you are finished, show your code to the TA and proceed.

\subsection{Inspecting ROS Messages using \href{http://wiki.ros.org/rostopic}{\texttt{rostopic}} and \href{http://wiki.ros.org/rosmsg}{\texttt{rosmsg}}}
Now you are an expert in setting up ROS publisher and subscriber. However, you may be wondering how to decode those ROS messages or figure out what's inside of each datatype in order to write a callback function. The command line tool \href{http://wiki.ros.org/rostopic}{\texttt{rostopic}} and \href{http://wiki.ros.org/rosmsg}{\texttt{rosmsg}} are designed for this usage.\\

Let's try this out! First, make sure your Rviz is still running. Now, open a new terminal, activate our ROS environment (\texttt{conda activate ros\_base}), setup the ROS environment (\texttt{source devel/setup.bash}), and  try:
\begin{itemize}
    \item \lstinline{rostopic list}\\
    This will print the names of active topics
    \item \lstinline{rostopic info <Topic Name>}\\
    This will information about a desired topic, including its datatype, publisher, and active subscribers.
    \item \lstinline{rostopic echo <Topic Name>}\\
    This will print out ROS messages from a desired topic in your terminal
    \item \lstinline{rostopic type <Topic Name> | rosmsg show}\\
    This will first look up the datatype of the topic, then print out its data structure.    
\end{itemize}

A full list of \href{http://wiki.ros.org/rostopic}{\texttt{rostopic}} and \href{http://wiki.ros.org/rosmsg}{\texttt{rosmsg}} functionalities can be found in their documentations. 

\subsection*{Task 3: Fill in the subscriber callback function}
\addcontentsline{toc}{subsection}
{\textbf{Task 3: Fill in the subscriber callback function}}
Open your \texttt{pure\_pursuit.py} file. Your third task is to fill in the missing code of the function \texttt{goal\_callback} following instructions under \textbf{TODO}.\\
Once you are finished, \textbf{restart} \texttt{lab0\_simulation.launch}.  From the RViz simulator, you can add a desired goal location by selecting \textbf{2D Nav Goal} from the top panel and then clicking a point on the map. You will see that the position of your clicked point is printed on your terminal.

\subsection*{Task 4: Construct and publish a ROS message}
\addcontentsline{toc}{subsection}{\textbf{Task 4: Construct and publish a ROS message}}
Open your \texttt{pure\_pursuit.py} file. Your fourth task is to fill in the missing code of the function \texttt{publish\_control} following instructions under \textbf{TODO}. Once you are finished, show your code to your TA.

\section{Goal Reaching Controller}

In this Lab, you will implement a simple goal-reaching controller. We will use a proportional controller for the throttle, and a pure pursuit controller for steering. 

\subsection{Throttle Control}
Our robot can control its acceleration through the motor's throttle input. In this Lab, we will implement a proportional controller to track reference speed $V_{ref}$. 
\begin{equation}
    a = K_p(V_{ref}-V_{robot})
\end{equation}


\subsection{Steering Control}
The pure pursuit method is a geometry-based algorithm to determine desired steering angle for a car to follow a path. As shown in \autoref{fig: pure-pursuit}, pure pursuit calculates the steering angle $\delta$ to ensure the vehicle reaches the target point (\textbf{TP}) according to the kinematic bicycle model. This \href{https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html}{tutorial} provides an excellent interactive explanation of the pure pursuit algorithm. 

\begin{figure}[h]
    \centering
    \includegraphics[width=0.7\textwidth]{lab0/figures/pure-pursuit.png}
    \caption{Geometric Interpretation of Pure-Pursuit Algorithm [\href{https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html}{source}]}
    \label{fig: pure-pursuit}
\end{figure}

In short, you can obtain the steering angle $\delta$ by \autoref{eq: pure-pursuit}, where $L$ is the wheelbase of the robot, $\alpha$ is the relative angle of the look-ahead point w.r.t the robot, and $l_d$ is the distance between the robot and the look-ahead point. 

\begin{equation}
    \delta = \arctan \left(\frac{2 L \sin(\alpha)}{l_d}\right) 
    \label{eq: pure-pursuit}
\end{equation}

In this Lab, we assume the reference path is the straight line connecting your robot and goal point. Therefore, the \textbf{TP} is a point on this line segment defined by user parameters. \\

\subsection*{Task 5: Implement the goal reaching controller}
\addcontentsline{toc}{subsection}{\textbf{Task 5: Implementing Goal Reaching Controller}}
Open your \texttt{pure\_pursuit.py} file. You will finish the function \texttt{planning\_thread} following the implementation details under the \textbf{TODO} block. This task concludes all coding parts of Lab 0. Re-launch the simulation, set \textbf{2D Nav Goal} as any points on Rviz, and drive your robot towards the goal point. The default parameter should work well in the simulation if your implementation is correct. \textbf{Show your simulation results to your TAs. }

\section{Let's Get Real}
The modularity of ROS allows us to quickly deploy our algorithms from the simulated environment into the real robot with minimal changes to your code. 

\subsection*{Task 6: Try Out On Mini Truck}
\addcontentsline{toc}{subsection}{\textbf{Task 6: Try Out On Mini Truck}}

Follow the instructions in the \textit{Intro to Mini Truck} tutorial and test your goal-reaching controller on the Mini Truck with the provided \texttt{lab0\_truck.launch}. \textbf{Demo your robot to your TAs.}



# HERE IS INTRO TO RC CAR

\section{Intro to the \emph{Mini Truck} Robotic Platform}

Autonomous driving has sparked much public interest in the last few years. In this lab, we will work with a 1/14-scale autonomous \emph{mini truck} as our mobile robot platform (\autoref{fig:robot}).

\begin{figure}[h]
    \centering
    \includegraphics[width=0.7\textwidth]{lab0/figures/robot.jpg}
    \caption{The 1/14-scale autonomous \emph{mini truck} used in Intelligent Robotic Systems.}
    \label{fig:robot}
\end{figure}

 \begin{figure}[h]
    \centering
    \includegraphics[width=0.8\textwidth]{lab0/figures/schematic.pdf}
    \caption{A hardware schematic of the robotic platform.}
    \label{fig:schematic}
\end{figure}

\autoref{fig:schematic} overviews the robot's key physical components.
The robot's \textbf{body} carries an \href{https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-xavier-nx/}{NVIDIA Jetson Xavier NX} onboard computer, a \href{https://www.stereolabs.com/zed-2/}{ZED 2 stereo camera}, and a battery that powers them. These components form the core of the robot's perception and decision-making hardware.\\
 
The robot's \textbf{chassis} consists of drivetrain components that steer and drive the truck. We use a \href{https://www.tamiyausa.com/shop/110-4wd-shaft-drive-road-tt/rc-team-hahn-racing-man-tgs-2/}{TAMIYA TT-01 Type-E} RC chassis (\autoref{fig:TT01}), which is a shaft-driven 4WD platform powered by a single 25 turn 540 brushed DC motor. The Jetson is connected with a \href{https://www.pololu.com/product/1350}{Mastro Servo Controller}, which translates the command from the Jetson and sends a Pulse Width Modulation (PWM) signal to the steering servo and motor ESC. In order to allow the robot to be driven through a remote controller, we added an additional \href{https://www.pololu.com/product/2806}{servo multiplexer} to switch signals.

\begin{figure}[h!]
    \centering
    \includegraphics[width=0.7\textwidth]{lab0/figures/TT01E_chassis.jpg}
    \caption{TT-01 Type-E Chassis that is used as the base of our robot platform}
    \label{fig:TT01}
\end{figure}
\newpage
\subsection{Safety First}
\vspace{-1mm}
Read the following instructions \textbf{carefully} before operating the robot, and keep them handy.
\newmdenv[linecolor=red,linewidth=3pt, backgroundcolor=white]{callout}
\begin{callout}[frametitle=Robot Rules \& Responsibilities]
Please keep the robot and its accessories stored when you are not working with them. You can keep the robot in F111 or take it with you if it is convenient. Either way, please note that \textbf{you are responsible for your assigned robot for the duration of the semester}.
\begin{itemize}
    \item \textbf{Use caution and common sense when operating your robot.} Avoid driving it around potential hazards---including stairs---or unsuspecting civilians! Beyond basic ``bench tests'', we strongly recommend you only drive the robot inside the F111 lab space.
    
    \item \textbf{Let us know immediately if any component is damaged or goes missing,}
    so that we can address the issue as quickly as possible.
    If you are experiencing hardware issues with the robot, please consult with a course AI \textbf{before} you try to fix it by yourself.
    
    \item {\textbf{Practice with the remote control and drive slowly!}} The robot can achieve a maximum speed of over 15 mph (25 km/h). \textcolor{red}{\mbox{\textbf{Injuries and bone fractures are likely}}} if a person is hit at that level of speed. 
    
    \item Be \textbf{extremely careful} when handling and connecting batteries. {\color{red}\textbf{Shorted batteries will cause a fire}}.
    
    \item Motors can become very {\color{red}\textbf{hot}} while driving the robot \textbf{and} shortly after!
    
    \item The motor ESC will automatically cut off power if the Ni-MH battery voltage is too low. However, if the robot has been running for over 30 minutes or you notice a significant power drop, you should \textbf{immediately} place the battery in the \mbox{\textbf{Uncharged Battery Bin}} and replace it with a charged battery. 
    
    \item Please be aware that our ability to repair the robots is limited, so take care of your robot accordingly. \textcolor{red}{\textbf{If your robot suffers severe damage, you may not be able to complete the Lab and Final Project components of the class.}}

\end{itemize}
\end{callout}


\subsection{Connecting to the Robot}
\subsubsection{Turning On the Jetson}
The NVIDIA Jetson Xavier NX takes 12 V -- 20 V DC power provided by the power bank mounted right below the Jetson. The power bank can output various DC voltages ranging from \mbox{5 V} to \mbox{20 V}. In order to avoid damage to the Jetson, {\color{red}\textbf{make sure you unplug the power cord before turning on the battery}.} First, hold the power button on the battery until it lights up. Then, cycle through the voltage by double-clicking the power button until it shows {\color{red}\textbf{20 V}}. Underpower will lead to hardware malfunctions. Finally, plug in the power cord, and the Jetson will turn on automatically.\\

\looseness=-1
You will see the remaining battery life on the screen. Please note that the power will last less than two hours. To program the Jetson, use the external power supply, labeled \textbf{NX PWR}, instead of using the battery. 

\subsubsection{Working on the Jetson Directly}
The Jetson NX runs Ubuntu 20.04 desktop OS and can be used as a regular PC by connecting with a keyboard, a mouse, and a monitor through the HDMI port. The password for login is \textbf{nvidia}.
The F110 Lab space provides Windows workstations, and you can borrow a mouse, keyboard, and monitor for the Jetson.

\subsubsection{Working on the Jetson Through SSH Connection}
When the robot cannot be connected to a monitor and keyboard, for example when it is running on the ground, SSH becomes a useful tool to log into your robot and run programs through the command-line interface. SSH stands for \textbf{S}ecure \textbf{Sh}ell, a protocol that allows you to securely and remotely connect to your robot using a wired/wireless network connection.\\

All robots are connected to the local Wi-Fi network \textbf{ECE346} at startup. Each will have a reserved IP address \textbf{192.168.1.1XX}, where XX is the ID of the robot. For example, if the Jetson on your robot has the label NX-7, the IP address is \emph{192.168.1.107}. Similarly, if your robot is NX-11, the IP address is \emph{192.168.1.111}. Before running SSH, first, connect your computer to the \textbf{ECE346} Wi-Fi with password \textbf{ece346sp2023}. Once you have connected to the network,  open a terminal/power shell and type
    \begin{lstlisting}[language=bash]
    ssh nvidia@192.168.1.1XX
    \end{lstlisting}
    It will ask you for the login password for the Jetson: enter \textbf{nvidia} and you are all set.\\
    % \item If you are using a \textbf{Mac} computer, first install \href{https://www.xquartz.org/}{XQuarts}, then you can follow the same procedure with a terminal.
    % \item If you are using a \textbf{Windows} computer, first install \href{https://www.x.org/wiki/}{Xming}, then you can follow the same procedure from PowerShell. On the other hand, you can install the \href{https://www.putty.org/}{PuTTY} SSH client for convenient interface. If PuTTY is used, you might need to enable X11 forwarding:
    % \begin{itemize}
    %     \item In Putty, click on the plus sign to the left of "SSH" in the left hand panel \item Click "X11" and check the box labelled "Enable X11 Forwarding".
    % \end{itemize}
    % \begin{figure}[H]
    %      \centering
    %      \includegraphics[width=0.6\textwidth]{B7r4t.png}
    %      \caption{Enable X11 Forwarding in PuTTY}
    %      \label{fig: tx}
    % \end{figure}

SSH is a feature that can be found in any modern OS (Mac/Linux/Windows) machine. You can either use one of the Windows  workstations in the F111 lab or bring your own laptop to connect to the robot. The process to get SSH working may differ across operating systems. If you do not have previous experience with SSH, we provide a detailed set of instructions in the \href{https://github.com/SafeRoboticsLab/ECE346/blob/SP2023/Docs/ssh.md}{class GitHub repo}.%\\

% \subsubsection{Coding Remotely with VS Code}
% If you want to code directly on your laptop but have the code saved in Jetson directly, one solution is to SSH into the Jetson and use vim or nano through the terminal. On the other hand, if you enjoy using modern IDEs,  \href{https://code.visualstudio.com/docs/remote/ssh-tutorial}{VS Code allows you to code remotely through SSH}.  
% \subsection*{Checkpoint 3: Create your repository}
% \addcontentsline{toc}{subsection}{\textbf{Checkpoint 3}}
% Each team is required to create a repository that forks the \href{https://github.com/SafeRoboticsLab/ECE346}{class GitHub repo}. We recommend you to follow the repo's \texttt{README} file and create a \emph{private} fork. In addition, please add all course AIs (zzx9636, kaichiehhsu, buzi-princeton) to your forked GitHub repo by the due date.

\subsection{Driving the Robot with the Remote Controller}
% https://manuals.plus/spektrum/2-4ghz-digital-radio-system-transmitter-manual
\looseness=-1
The remote controller (\autoref{fig: tx}) allows you to drive the robot manually---as you would a regular RC car---and also serves as a \emph{dead man's switch} for the robot. We list each element's function below.
\begin{enumerate}[label=\Alph*.]
\item \textbf{Throttle Trim}: Adjusts the throttle neutral point
\item \textbf{Steering Trim}: Adjusts the steering centerpoint. Normally, the steering trim is adjusted until the vehicle tracks straight.
\item \textbf{LED}: Indicates the power is ON 
\item \textbf{Steering Wheel}: Controls the steering angle of the front wheels.
\item \textbf{Throttle/Brake}: Controls the vehicle's acceleration.
\item \textbf{Steering Rate}: Adjusts the sensitivity (gain) of the steering wheel.
\item \textbf{Channel 3}: Three-position momentary switch (not used here).
\item \textbf{Throttle Limit}: Limits throttle output to 50/75/100\%. Note: you should keep it at 50\%. 
\item \textbf{Throttle Reversing}: Flip the switch to reverse the throttle channel.
\item \textbf{Steering Reversing}: Flip the switch to reverse the steering channel.
\item \textbf{Power Button}: Turns the controller on and off.
\end{enumerate}

\begin{figure}[H]
    \centering
    \begin{subfigure}[b]{0.49\textwidth}
         \centering
         \includegraphics[width=0.5\textwidth]{lab0/figures/TX_front.png}
         \caption{Right diagram of the remote controller.}
     \end{subfigure}
     \hfill
     \begin{subfigure}[b]{0.49\textwidth}
         \centering
         \includegraphics[width=0.5\textwidth]{lab0/figures/TX_back.png}
         \caption{Left diagram of the remote controller.}
     \end{subfigure}
     \caption{Diagrams of the remote controller}
     \label{fig: tx}
\end{figure}

The robot's drivetrain uses a separate power source that is connected directly to the motor ESC. To power up the robot, \emph{first} turn on the remote controller by pressing the Power Button (K), \emph{then} turn the switch on the bottom of the chassis to the ON position.\\

By default, pull the throttle towards you to go forward, and push the throttle away to brake and reverse. In order to steer the truck, you need to rotate the steering knob by the desired amount. You can invert the throttle and steering using the corresponding (I, J) switches.

\subsection{Driving the Robot with the onboard Jetson computer}
\label{sec: drive_robot}
We use a \href{https://www.pololu.com/product/1350}{Maestro 6-Channel USB Servo Controller} to control the motor ESC and steering servo. 

% The running documentation of this servo controller can be found \href{https://www.pololu.com/docs/0J40}{here}. You can access the GUI interface of the controller from the command line:

% \begin{lstlisting}[language=bash]
%     cd YOUR_REPO/asset/maestro-linux/
%     ./MaestroControlCenter
% \end{lstlisting}

We have provided you with a ROS wrapper of the Mastero Servo Controller API. It subscribes to a ROS topic and sends inputs to the controller. Due to safety concerns, the multiplexer switch automatically disables the control signal from the Mastero Servo Controller. \textbf{In order to drive the robot with Jetson, you need to press the Down button of Channel 3 (G) all the time}. The system will immediately switch to the remote controller mode if you release this button. 

\newpage
\section{Moving Mini Truck Autonomously}
Before we run any decision-making algorithms, we need to ensure our mini-truck knows where it is and can execute our control command. These functionalities have been built on your robot as ROS nodes.\\

In addition, we want to easily visualize the state and future plan of our robot on your own computers. Luckily, ROS has made it easy for us since it is naturally a distributed computing environment. A running ROS system can comprise dozens, even hundreds of nodes, spread across multiple machines with \href{http://wiki.ros.org/ROS/NetworkSetup}{network setups}.\\

In general, to pass ROS messages between your laptop and the robot, your computer must be connected to the same network. We provide two useful scripts for network setups, which you will use later on.
\begin{lstlisting}[language=bash]
    # If the computer hosts ROS Master
    source network_ros_host.sh <HOST_IP>

    # If the computer is a client of the ROS Master
    source network_ros_client.sh <HOST_IP> <CLIENT_IP>
\end{lstlisting}

In ECE346, \texttt{<HOST\_IP>} is the IP address of your Robot, and \texttt{<PC\_IP>} is the IP address of your computer under ECE346 WIFI. You can look this up on your computer's network settings.

\subsection*{Step 1: Launch Perception and Control Nodes On Robot}
\addcontentsline{toc}{subsection}{\textbf{Step 1: Launch Perception and Control Nodes On Robot}}
 To launch perception and control nodes, \textbf{open a new terminal} and \textbf{ssh into your robot}. Then, 
\begin{lstlisting}[language=bash]
    cd ~/StartUp
    ./start_ros.sh <HOST_IP>
\end{lstlisting}
The \texttt{./start\_ros.sh <HOST\_IP>} command will automatically set your robot as the host of ROS Master using the previously mentioned script and start ROS. Please make sure your robot is static on the track in the F111 lab, because the localization algorithm requires accurate gravitational direction for initialization.  

% If you encounter the error stating this file is not executable, you can change the permission by the following command and then retry.
% \begin{lstlisting}[language=bash]
%     chmod +x start_ros.sh
% \end{lstlisting}

\subsection*{Step 2: Launch Visualization On Your PC}
\addcontentsline{toc}{subsection}{\textbf{Step 2: Launch Visualization On Your PC}}
Next, we open a new terminal on your PC and navigate to the \texttt{ROS\_Core} under your Git repository. Then, we activate the conda ROS environment, and (optionally) rebuild the workspace by:
\begin{lstlisting}[language=bash]
    # Navigate to ROS_Core
    cd <Path of your repo>/ECE346/ROS_Core
    # Start virtual environment
    conda activate ros_base
    # Optional: if you have new packages
    catkin_make
\end{lstlisting}
Moreover, let's configure the network setting using the script
\begin{lstlisting}[language=bash]
    source network_ros_client.sh <HOST_IP> <PC_IP>
\end{lstlisting}
Finally, we can launch visualization nodes
\begin{lstlisting}[language=bash]
    source devel/setup.bash # .zsh for Mac user
    roslaunch racecar_interface visualization.launch
\end{lstlisting}
After around 30s, Rviz (\autoref{fig: rviz_truck}) and RQT (\autoref{fig: rqt_truck}) windows will show up. 

\begin{figure}[H]
    \centering
     \includegraphics[width=0.6\textwidth]{lab0/figures/rviz_truck.png}
     \caption{Rviz visualization tool. The orange box indicates the current pose of the robot and the yellow arrows indicate the past poses.}
     \label{fig: rviz_truck}
\end{figure}

\begin{figure}[H]
     \centering
     \includegraphics[width=0.65\textwidth]{lab0/figures/rqt_truck.png}
     \caption{RQT GUI.}
     \label{fig: rqt_truck}
\end{figure}

\subsection*{Step 3: Start Localization}
\addcontentsline{toc}{subsection}{\textbf{Step 3: Start Localization}}
On your RQT console (\autoref{fig: rqt_truck}), first, go to the \textbf{Service Caller} page. Then, click the \textbf{refresh button} \faRefresh~ and choose \textbf{/SLAM/start\_slam} from the drop-down menu. Finally, click the \textbf{call} button to start localization. \\

An orange box will appear on your Rviz, which indicates the pose of your robot. Drive around the track and try to verify if the state estimation is accurate.\\

\textbf{Important}: Localization results will be significantly compromised if fiducial markers are occluded. Please do not stay or place your items inside the room.

\subsection*{Step 4: Launch Your Program On Truck}
\addcontentsline{toc}{subsection}{\textbf{Step 4: Launch Your Program On Robot}}
To launch your own decision-making algorithms, \textbf{open a new terminal} and \textbf{ssh into your robot}. Then, let's navigate to your workspace, activate the conda ROS environment and configure network setups by:
\begin{lstlisting}[language=bash]
    cd <Your Workspace>
    conda activate ros_base
    source network_ros_host.sh <HOST_IP>
    source devel/setup.bash # .zsh for Mac user
\end{lstlisting}
Finally, launch your node and remember to press the Down button as described in Section \ref{sec: drive_robot} to test your algorithm.
\begin{lstlisting}[language=bash]
    roslaunch <ROS Package> <Launch File>
\end{lstlisting}


