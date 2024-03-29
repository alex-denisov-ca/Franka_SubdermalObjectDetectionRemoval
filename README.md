# Robot-assisted Subdermal Foreign Object Detection and Removal For Franka Panda

## **Quick Start Guide**
This project is for use with the Franka Emika Panda cobot and the Healson C30L ultrasound probe, as well as similar probes and cobots.

The project uses the Franka robot alongside an ultrasound probe to scan a section of ultrasound gel or water; the medium will mimic human flesh. Within the medium, one or more metallic bolts or other objects will simulate shrapnel subdermally embedded. 

*(0): Startup*<br />
Git clone and build the project, specificslly the csc496 directory of the project in the same directory that contains the Franka Panda's [libfranka](https://github.com/frankaemika/libfranka) directory.

Connect to the Franka control interface and ensure everything is properly initialized; joints are unlocked, and the robot is not reflex-locked or user-stopped.
First, set the z component of the end effector to 150mm in Franka Desk, also measure and set the weight of the Franka hand including the probe, grippers and holders (print and user depedant).
Second, enable the [FCI](https://frankaemika.github.io/docs/getting_started.html)


Install printed hardware to the Franka hand, seal important electrical components on and around the robot, set your tray/working area, then fill the tray with the preferred medium and insert foreign bodies.

Then start the docker container included in the "docker" directory within "csc496":
    run build_docker_container.sh
    run start_docker_container.sh
    run open_docker_terminal.sh
    to close the running terminal, run stop_docker_container.sh

*(1): Running the code*<br />
Inside the docker container, go to `/root/git/scratchpad/csc496/build` and run `rm CMakeCache.txt && cmake ../ && make` in the bash terminal.
then run `run.sh` in the terminal; this script links all the steps together onto a single terminal, follow instructions and prompts that show up on the terminal.
    There will be four distinct steps, with an "operator" sitting at the terminal and the ultrasound screen and a "guide" watching the robot and manning the enabling device.

    (0): Setting the origin of the scanner
        - The guide will move the probe to the bottom left of the tray farthest away from the robot
           and to the side that it has its power ports.

    (1): Rough scanning mode
        - the probe will begin scanning the working area, the operator should press space 
          when an anomaly is detected.

    (2): Keyboard guidance mode
        - The operator will have to navigate to the rough position and center the object on the ultrasound, 
          this should be done with the help of the guide to avoid collisions.

    (2.5): (Suggested) Remove the ultrasound probe from the holder.      

    (3): Object Retrieval
        - after the operator has identified the object then, the robot will attempt to grasp the object, 
          upon completion, it will home and drop the object for the guide to catch upon operator approval. 


## **Running Details**
if, for some reason, you don't use `run.sh`, then you can run each step individually after following the initial instructions from Startup, as shown below:<br />

*(0): Set origin of scan*<br />
     - sets and records the origin of scanning sequence<br />
     - use for manually running scan from origin<br />

    Run manually:
        ./set_origin <YOUR_FRANKA_IP>

    Controls:
        Enter - saves the position
        Any other key - quit the program

*(1): Automated scanning of the medium in a rectangular pattern*<br />
     - when the operator sees a foreign object on the ultrasound screen they press the space bar<br />
     - the location is saved in a text file for further inspection and adjustment<br />
     - this simulates scanning a large portion of the body to locate multiple pieces of shrapenal<br />

    Run manually:
        ./run_autoscan_rough_capture <YOUR_FRANKA_IP> <Origin X> <Origin Y> <Origin Z>

    Controls:
        Space - saves the position

*(2): Fine position finding* <br />
     - Now that you know the approximate position of the shrapnel, use the keyboard to locate it <br />
     - Once the operator has found the center of the object, they will press 'space' and save the coordinates <br />
     - The saved coordinates will also be saved to a text file; both to handle multiple pieces of shrapnel, or multiple grasping attempts of the same object <br />
    
    Run:
        ./run_keyboard_input_capture <YOUR_FRANKA_IP>  <X> <Y> <Z>

     Controls:
        W - moves forward (towards the robots base)
        A - moves left (towards the side of the robots power connectors)
        S - moves backward (away from the robots base)
        D - moves right (towards the side of the robots ethernet port)
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
     - If the robot fails to grasp and extract the object, repeat step 2 or use alternative object positions already collected in the previous step<br />
     - Press the space bar to open the gripper and drop the extracted object (or simply open the gripper for another iteration)<br />
  
    Run manually:
        ./run_keyboard_input_capture <YOUR_FRANKA_IP>  <X> <Y> <Z> 
        
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
with any tray or working area, there are mechanical limitations that result in spaces where the probe cannot scan / the grippers cannot reach.

It is suggested to use an ultrasound gel for running this project; for best results, smooth the gel before every run (this can be done with something unimportant, like a student ID).

For running in custom environments, you will also have to measure dimensions and the coordinate of your working space; `set_origin` allows you to move the board anywhere in the working space (without rotations), but it may be necessary to manually tweak code if your tray is larger or smaller than the one provided to work with the auto scan feature.

## **Dependencies**
All files are to be run on a Linux real-time kernel.
The github repo OR the csc496 directory included in the repo must be in the same directory as [libfranka](https://github.com/frankaemika/libfranka); i.e in some directory X, there will X/libfranka, X/csc496, X/Models, etc.

If you are NOT running the provided docker container, you will have to have at minimum [ncurses-6.0](https://lists.gnu.org/archive/html/info-gnu/2015-08/msg00002.html) installed; 

## **Citation**
This codebase is modified from the one provided in CSC496H5 at the University of Toronto Mississauga, taught by Professor Lueder Kahrs and provided by Ruthrash Hari; based on the Franka Emika [libfranka repo.](https://github.com/frankaemika/libfranka)

Citation is available in the About section of the GitHub project or the CITATION.cff file.

## **Testing**
We have tested this repo using the Franka Panda robot, the Healson C30L ultrasound probe, and all stl files provided in this repository. Tests have been running on multiple robots; different robots have different vertical offsets from their base due to mounting variances, and thus, some board parameters need to be tweaked between robots.

When testing, running for the first time, or tweaking the parameters, it is important to always have one person manning the enabling device and developing incrementally with one change at a time. For example, changing the speed; start at a known working speed, then increment it to the desired value, etc.

In testing the code, it is best to perform the tests without a medium inside the tray. We have tested each of the four sections above individually and together with `run.sh`. 

We have run through all steps of the project multiple times using a 15mm M6 screw (shrapnel) embedded in ultrasound gel (medium), resulting in successful objection detection at all stages and removal from the medium. 

If the program is unsuccessful, there may be times that the terminal running the program will freeze to an orphan thread run by one of the members. It is also important to move the robot between runs to ensure there is no reflex-lock and also to not put excessive force on the robot while it's moving and running.