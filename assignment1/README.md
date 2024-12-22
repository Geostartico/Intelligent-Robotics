## GROUP 02 

Artico Giovanni, [giovanni.artico@studenti.unipd.it](mailto:giovanni.artico@studenti.unipd.it)

Colla Francesco, [francesco.colla.2@studenti.unipd.it](mailto:francesco.colla.2@studenti.unipd.it)

Toffolon Mattia, [mattia.toffolon@studenti.unipd.it](mailto:mattia.toffolon@studenti.unipd.it)

---

## Code Running Instructions

For this project we provide some launchfiles to ease the execution of the program. 
Launch them on different terminals in the order later explained to correctly start the system.

Firstly start the simulation environment:

```bash
	roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment1
```

Once the arm tucking procedure is completed, use the following launchfile to start the nodes needed for robot movement and apriltags id generation which were already provided for this assignment:

```bash
	roslaunch assignment1 after_gazebo.launch
```

Now that the environment is ready, the nodes developed by us can be started. Launch the following files in the given order to start the simulation:

```bash
	roslaunch assignment1 independent_services.launch 
	roslaunch assignment1 last_nodes.launch