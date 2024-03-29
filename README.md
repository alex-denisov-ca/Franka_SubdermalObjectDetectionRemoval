# Robot-assisted Subdermal Foreign Object Detection and Removal For Franka Panda

## **Quick Start Guide**
This Project is for use with the Franka Emika Panda cobot and the Healson C30L ultrasound probe; and similar probes and cobots.

The project uses the Franka robot alongside an ultrasound probe to scan a section of ultrasound gel or water; the medium will mimic human flesh. Within the medium, there will be one or more metallic bolts or other objects that simulate shrapnel subdermally embedded. The project consists of three parts:

*(0): Startup*<br />
Connect to the Franka control interface and ensure everything is properly initialized; such as the joints are unlocked, robot is not reflex-locked, and is not user-stopped.
First, set the z component of the end effector to 150mm in Franka Desk, and measure the weights of the probe, grippers and holders (print and user depedant).
Second, enable the [FCI](https://frankaemika.github.io/docs/getting_started.html)


Install dependencies and hardware; see further sections for instructions.

Then start the docker container included in the "docker" directory within "csc496":
    run build_docker_container.sh
    run start_docker_container.sh
    run open_docker_terminal.sh
    to close the running terminal, run stop_docker_container.sh

*(1): Running the code*<br />
Inside the docker container go to `/root/git/scratchpad/csc496/build` and run `rm CMakeCache.txt && cmake ../ && make` in the bash terminal.
then run `run.sh` in the terminal; this script links all the steps together onto a single terminal, follow instructions and prompts that show up on the terminal.
    There will be four distinct steps, with an "operator: sitting at the terminal and ultrasound screen, and a "guide" watching the robot and manning the enabling device.

    (0): Setting the origin of the scanner
        - The guide will move the probe to the bottom left of the tray farthest away from the robot
           and to the side that it has its power ports and follow instruction on screen.

    (1): Rough scanning mode
        - the probe will begin scanning the working area, the operator should press space 
          when an anomally is detected.

    (2): Keyboard guidance mode
        - The operator will have to navigate to the rough position and center the object on the ultrasound, 
          this should be done with the help of the guide to avoid collisions.

    (2.5): (Suggested) Remove the ultrasound probe from the holder.      

    (3): Object Retreival
        - after the operator has identifed the object then the robot will attemp to grasp the object, upon completin it will home and drop the object for the guide to catch upon operator approval. 


## **Running Details**
if for some reason you dont use `run.sh` then you can run each step individually after following the inital completing the instructions from Startup, as shown below:<br />

*(0): Set origin of scan*<br />
     - sets and records origin of scanning sequence<br />
     - use for manually running scan from origin<br />

    Run manually:
        ./set_origin *YOUR_FRANKA_IP* 

    Controls:
        Enter - saves the position
        Any other key - quiot the program

*(1): Automated scanning of the medium in a rectangular pattern*<br />
     - when the operator sees a foreign object on the ultrasound they press the space bar<br />
     - the location is saved in a text file for further inspection and adjustment<br />
     - this simulates scanning a large portion of the body to locate multiple peices of shrapenal<br />

    Run manually:
        ./run_autoscan_rough_capture *YOUR_FRANKA_IP* *<Origin X>* *<Origin Y>* *<Origin Z>*

    Controls:
        Space - saves the position

*(2): Fine position finding* <br />
     - Now that you know the approximate position of the shrapnel use the keyboard to locate it <br />
     - Once the operator has found the center of the object they will press 'space' and save the coordinates <br />
     - The saved coordinates will also be saved to at text file; both to handle multiple pieces of shrapnel, or multiple grasping attmpts of the same object <br />
    
    Run:
        ./run_keyboard_input_capture *YOUR_FRANKA_IP*  *<X>* *<Y>* *<Z>*

     Controls:
        W - moves forward (away from the robots base)
        A - moves left (towards the side of the robots ethernet port)
        S - moves backward (towards the robots base)
        D - moves right (towards the side of the robotspower connectors)
        Space - saves the position
        R - moves upwards
        F - move downwards
        T - releases/opens the gripper
        G - closes gripper to grasp
        Escape - stops and closes the program

*(3): Object extraction*<br />
     - The user will be prompted for a raw x,y position<br />
     - The operator will enter the x,y positions determined from the previous step<br />
     - The robot will then automatically go to its home position, open the gripper, go to the object, close the gripper, and extract it<br />
     - If the robot fails to grasp and extract the object repeat step 2 or use alternative object positions already collected in the previous step<br />
     - Press the space bar to open the gripper and drop the extracted object (or simply open the gripper for another iteration)<br />
  
    Run:
        ./run_keyboard_input_capture *YOUR_FRANKA_IP*  *<X>* *<Y>* *<Z>* 
        
    Inputs:
        x position - raw x position of ultrasound detected object
        y position - raw y position of ultrasound detected object

     Controls:
        Enter - On prompt, continue execution of the next task
        Space - On prompt, open the gripper

## **Hardware and Envrionment**
For hardware used in the project, go to the "Models" directory and see [Hardware_Instructions.pdf](https://github.com/alex-denisov-ca/Franka_SubdermalObjectDetectionRemoval/blob/main/Models/Hardware_Instructions.pdf) for printing and assembly information.

Once assembled, there is a distance of 150mm from the flange of the robot to the ends of the grippers; set these values in Franka Desk.
It should also be noted that the center of the ultrasound probe is 60mm away from the center of the end-effector; 
with any tray or working area, there are mechanical limitations which result in spaces where the probe cannot scan / the grippers cannot reach.

It is suggstedted to use an ultrasound gel for running this project; for best results smooth the gel before every run (this can be done with something unimportant, like a student ID).

For running in custom enrionments, you will also have to measure dimensions and the coordinate of your working space;
this should be done in `set_origin` but it may be necessary to manually tweak code if your tray is larger or smaller than the one provided to work with the auto scan feature.

## **Dependencies**
All files are to be run on a Linux real-time kernel.
If you are NOT running the provided docker container, you will have to have at minimum [ncurses-6.0](https://lists.gnu.org/archive/html/info-gnu/2015-08/msg00002.html) installed; 

## **Citation**
This codebase is modified from the one provided in CSC496H5 at the University of Toronto Missisauga, taught by Professor Lueder Kahrs and provided by Ruthrash Hari; based on the Franka Emika [libfranka repo.](https://github.com/frankaemika/libfranka)

Citation is available in the about section of the github project, or the CITATION.cff file.


