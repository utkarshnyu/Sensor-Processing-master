# Sensor processing
Code for processors on the hyperloop pod that deal with sensor data. There are three processors that handle specific sensor groups (halleffect, pneumatics and PED) and another central processor that handles processing of smaller sensors that are not as critical as well as sensor fusion. Data is sent between the processors over SPI. Data is also sent and recieved by the central arduino with a raspberry pi for saving data on the server as well as recieving commands.


# Halleffect
The halleffect processor handles rpm, distance traveled and linear velocity gathered from each of the halleffects on each of the four wheels.


# Pneumatics
Pneumatics deals with pressure data on the pneumatics system as well as the states of the cylinders (retracted or extended)


# PED
Collects data from the four PED sensors to count the number of tape strips that have been passed by the pod. The number of tape strips passed gives us our distance traveled in increments of 100ft.

# Central Arduino
Collects data from each of the processors and fuses the data together to get an accurate reading on the health and state of the pod. Also sends data to pi for viewing data on the GUI and for recieving remote commands such as braking. Central arduino determines when the pod needs to begin braking depending on the position of the pod in the tube and the pods speed.

