## DHRTA Strategy repository                                       

This repository contains the ROS-DHRTA code, the domain and problems used to prove our task allocation strategy:

The implementation is devided in two parts:


### 1. Goals Distribution

1.1 Compilation:

   Select or create a catkin workspace:

         mkdir -p dhrta_ws/src
         cd dhrta_ws/

   Get the code:

         cd src/
         git clone https://github.com/YanielCarreno/DHRTA.git

   Compile everything:

         catkin build

1.2 To run an example:

         roslaunch dhrta dhrta_mission_execution.launch

1.3 The results:

The allocation_solution.txt file located in the dhrta package shows the allocation's results. The file contains all the goalas and the robot in charge of executing them. For instance, (poi_rock_analysis wp42)[rexrov0] defines the goal (written in PDDL format) is executed by robot rexrov0. These results can be added to the PDDL problem files to generate plans using the steps in the Task Planning section.

### 2. Task Planning


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
@InProceedings{carreno2020a,
  Title  = {A Decentralised Strategy for Heterogeneous AUV Missions via Goal Distribution and Temporal Planning},
  Author = {Carreno, Yaniel and Pairet, {\`E}ric and Petillot, Yvan and Petrick, Ronald PA},
  Booktitle = {Thirteenth International Conference on Automated Planning and Scheduling},
  Year = {2020}
}
```
                
