# FSM Project

For the detailed architecture and behavior of the project, see the README.pdf (sorry for the terrible naming convention). 
The goal of this README is to describe how to run the project as the way we implemented it needs a bit of explanation. 

# How to run the code (PLEASE READ!!)

Run these command-line prompts in the folder. Each node has to be activated in a separate terminal. The program will need all the nodes to function as intended.

```ros2 run master_node controller```

```ros2 run wall_follow_pkg wall_follow```

```ros2 run person_follow_pkg person_follow```

```ros2 run driving_square driving_square```

```ros2 run collision_avoidance_pkg collision_avoidance```

The master_node has a file controller.py that tracks the topics /neato_behavior and /fsm_events, both of which have publishers and subscribers in the other nodes.
If the other nodes are not run with the master_node, then Neato will not work or will experience buggy behavior.
