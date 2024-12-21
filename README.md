# README #

### GROUP 02 

Artico Giovanni, [giovanni.artico@studenti.unipd.it](mailto:giovanni.artico@studenti.unipd.it)

Colla Francesco, [francesco.colla.2@studenti.unipd.it](mailto:francesco.colla.2@studenti.unipd.it)

Toffolon Mattia, [mattia.toffolon@studenti.unipd.it](mailto:mattia.toffolon@studenti.unipd.it)

# Instructions to run the code

## To toggle the motion control law open the file /assignment1/lanch/launch_independent_services.launch and in line 3 set args to 0

For this assignment we provided some launchfiles to ease the execution of the program, to do so send this commands in different terminals. 
Firtly start the simulation environment

```bash
	roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=iaslab_assignment1
```
And wait for the  arm tucking procedure, once that is completed start the nodes needed for movements and apriltags with the commands provided in the assignment paper by using a second launch file provided for ease of use

```bash
	roslaunch assignment1 launch_after_gazebo.launch
```

Now the enviorment is ready and we can start launching the nodes created by us, launching in the following order

```bash
	roslaunch assignment1 launch_independent_services.launch 
	
	roslaunch assignment1 launch_last_nodes.launch
```

Now the simulation started and the robot will navigate. To toggle

