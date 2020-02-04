## MRGA Strategy repository                                       

This repository contains the ROS-MRGA code, the domain and problems used in:

"Task Allocation Strategy for Heterogeneous Robot Teams in Offshore Missions"

The implementation is devided in two parts:


### 1. GOALS ALLOCATION

1.1 Compilation:

   Select or create a catkin workspace:

         mkdir -p mrga_ws/src
         cd mrga_ws/

   Get the code:

         cd src/
         git clone https://github.com/YanielCarreno/MRGA.git

   Compile everything:

         catkin build

1.2 To run an example:

         roslaunch mrga mrga_strategy.launch

1.3 The results:

The allocation_solution.txt file located in the mrga package shows the allocation's results. This results can be added to the PDDL problem files to generate plans using the steps in the TASK PLANNING section.


### 2. TASK PLANNING


2.1 First folder (constrained_domain) contains the domain and problems that consider 
   the predicate "robot_can_act" to implement the plan which were generated using the 
   Multi-Role Goal Assignment (MRGA) strategy presented in the paper.

2.2 Second folder (non-constrained_domain) constains a version of the domain and problems
   in folder one without using the predicates generate by  the MRGA strategy.
   
   How to run it?
   
       --> Clone the repository
       --> Compile the planners:
       
                1) TemporAl, TFLAP & OPTIC IPC-2018 (https://ipc2018-temporal.bitbucket.io/)
       
                2) LPG (http://zeus.ing.unibs.it/lpg/) 
                
                3) TFD (http://gki.informatik.uni-freiburg.de/tools/tfd/)
                
                4) POPF (https://nms.kcl.ac.uk/planning/software/popf.html)
                
                
         --> Run the planners using the domain and problem required
       
## Publications

If you are using this work for your research, please cite:
```
@InProceedings{carreno2020task,
  Title  = {Task Allocation Strategy for Heterogeneous Robot Teams in Offshore Missions},
  Author = {Carreno, Yaniel and Pairet, {\`E}ric and Petillot, Yvan and Petrick, Ronald PA},
  Booktitle = {Proceedings of the 2020 International Conference on Autonomous Agents and Multiagent Systems},
  Year = {2020}
}
}
```
                
