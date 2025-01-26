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
	roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment2
```

Once the arm tucking procedure is completed, use the following launchfile to start the nodes already provided for this assignment along with Node B and Node C:

```bash
	roslaunch assignment2 after_gazebo.launch
```

Lastly, launch Node A using the following command:

```bash
	roslaunch assignment2 main_services.launch
```

Note that all prints on screen have been suppressed besides the Node A ones.

---

## Additional points

Both additional points for this assignment (tables position and object color detections) have been implemented.
Both are enabled by default and it is not possible to deactivate them.
Therefore, the sole available version of the system is the most complete one.