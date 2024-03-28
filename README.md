# Instructions & Documentation : Robot-assisted Subdermal Foreign Object Detection and Removal For Franka Emika Panda & Similar Robots


## **Introduction and Quick Start Guide**
This Project is for use with the Franka Emika Panda cobot and the Healson C30L ultrasound probe; and similar probes and cobots.

The project uses the Franka robot alongside a ultrasound probe to scan a section of ultrasound gel or water; the medium will mimic human flesh. Within the medium there will be one or more metallic bolts or other objects which simulate shrapenel subdermally embedded. The projects consists of three parts:

*(0): Setup*<br />
Connect to the Franka control interface and ensure everything is properly initialized. 
First set the z component of the end effector to 150mm in Franka Desk
Second, enable the [FCI](https://frankaemika.github.io/docs/getting_started.html)

Install dependancies and hardware; see further sections for instructions.

Then start the docker container included in the "docker" directory within "csc496":
    run build_docker_container.sh
    run start_docker_container.sh
    run open_docker_terminal.sh
    to close the running terminal run stop_docker_container.sh

Inside the docker container goto `/root/git/scratchpad/csc496/build` and run `rm CMakeCache.txt && cmake ../ && make` in the bash terminal.

If you are in a new envrionment you must determine the origin (the bottom left point; facing away and furthest from the robot).
You must run ./TODOTODO  to find the origin point, 

TODO TODO

*(1): Automated scanning of the medium in a rectangular pattern*<br />
     - when the operator sees a foreign object on the ultrasound they press the space bar<br />
     - the location is saved in a text file for further insepction and adjustment<br />
     - this simulates scanning a large portion of the body to locate multiple peices of shrapenal<br />

    Run:
        ./   *YOUR_FRANKA_IP*

    Inputs:


    Controls:
        Space - saves the position

*(2): Fine position finding* <br />
     - Now that you know the approximate position of the shrapenel use the keyboard to locate it <br />
     - Once the operator has found the center of the object they will press 'space' and save the coordinates <br />
     - The saved coordinats will also be saved to at text file; both to handle nultiple peices of shrapenel, or multiple grasping attmpts of the same object <br />
    
    Run:
        ./   *YOUR_FRANKA_IP*  

     Inputs:
        s -

     Controls:
        W - moves forward (away from the robots base)
        A - moves left (towards the side of the robots ethernet port)
        S - moves backward (towards the robots base)
        D - moves right (towards the side of the robots power connectors)
        Space - saves the position
        R - moves upwards
        F - move downwards
        T - relases/opens the gripper
        G - closes gripper to grasp
        Escape - stops and closes the program

*(3): Object extraction*<br />
     - The user will be promted for a raw x,y position<br />
     - The operator will enter the x,y positons determined from the previous step<br />
     - The robot will then automatically go to its home position, open gripper, go to the object, close gripper, and extract it<br />
     - If the robot fails to grasp and extract the object repeat step 2, or use alternative object positions already collcted in the previous step<br />
     - Press space bar to open the gripper and drop the extracted object (or simply open the gripper for another iteration)<br />
  
    Run:
        ./   *YOUR_FRANKA_IP*  
        
    Inputs:
        x position - raw x positon of ultrasound detected object
        y position - raw y positon of ultrasound detected object

     Controls:
        Enter - On prompt, continue exectuion of the next task
        Space - On prompt, open the gripper



## **Hardware**
For hardware used in the project go to the "Models" directory and see "Hardware_Instructions.pdf" for printing and assembly information.

Once assembled, there is distance of 150mm from the flange of the robot to the ends of the grippers; set these values in Franka Desk.
It should also be noted that the center of the ultrasound probe is 60mm away from the center of the end-effector; 
with any tray or working area there are mechanical limitations which result in spaces where the probe cannot scan / the grippers cannot reach.

For running in custom enrionments, you will also have to measure deimensions and the coordinate of your working space;
by default the top left of the board is placed 32.5cm away from the base plate of the robot (13 M6 1" screw holes away)

## **Dependancies**
All files are to be run on a linux real-time kernel.
Must have at minimum [ncurses-6.0](https://lists.gnu.org/archive/html/info-gnu/2015-08/msg00002.html) installed; this should be done automatcially
inside the provided docker container. 

## **Citation**
This codebase is modifed from the one provided in CSC496H5 at the University of Toronto Missisauga, taught by Proffesor Lueder Kahrs, and provided by Ruthrash Hari.

This codebase can be cited as follows:










