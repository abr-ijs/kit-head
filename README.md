KIT HEAD INFORMATION
====================

The main documentation can be found at https://i61wiki.itec.uka.de/doc/karlsruhe_head_doc

More info: 
Carlos Rosales: carlos.rosales@for.unipi.it

MCA framework
-------------

git clone https://i61wiki.itec.uka.de/git/mca.git mca2.4

webpage: http://mca2.org/

Armar3Head repo
---------------

git clone https://i61wiki.itec.uka.de/git/karlsruhe_head.git

IVT repo
--------

Computer vision functionalities, extension of OpenCV.

git clone https://i61wiki.itec.uka.de/git/ivt.git IVT

webpage: http://ivt.sourceforge.net/


Before using the Head
---------------------

0. Login into the Head PC and DISconnect the cameras!

1. `cd /home/master/Projects/mca2.4`

2. `source script/mcasetenv -p armar3`

3. `startcans`

4. `startHead`

5. Load the command GUI `mcagui projects/armar3/etc/gui/HEAD_COMMAND.mcagui`

6. Click only the "RESET Head" button 

7. Click "Execution Toggle Head" button

8. Connect the cameras again and you are ready to go.

9. !! REMEMBER to stop the head after using it (stopcans, stopHead) !!

Network Configuration
---------------------

If you want to use the MCA framework with the GUI scenarios, you need to:

1. Download the MCA framework in your local machine

2. Login into the Head PC and modify /home/master/Projects/mca2.4/etc/hosts.allowed and add the IP address of your machine to be allowed to connect

3. (set the environment) Start the reset procedure of the head (startcans, startHead) in remote

4. (set the environment) Run your local GUI scenario, and connect to the Head PC by putting the IP:PORT, and leaving the default password (mca)

5. !! REMEMBER to stop the head (stopcans, stopHead) !!

Useful shortcuts
----------------

Head coordinate systems
https://i61wiki.itec.uka.de/doc/karlsruhe_head_doc/static/head_coords.pdf

Calibration after changing the focus length of any camera
https://i61wiki.itec.uka.de/doc/karlsruhe_head_doc/2-calibration.html

Pattern: https://i61wiki.itec.uka.de/doc/karlsruhe_head_doc/static/calibration_pattern.pdf
Print in A3, the squares must be 35mm.
