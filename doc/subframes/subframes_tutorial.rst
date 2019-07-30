Subframes
============================

Subframes are frames that are defined on CollisionObjects.
They can be used to define points of interest on objects that you place in the scene, such as
the opening of a bottle, the the tip of a screwdriver, or the head of a screw.
They can be used for planning and to write robot instructions such as "*pick up the bottle, then 
move the opening under the spout of the tap*", or "*pick up the screwdriver, then place it above 
the head of the screw*". 

Writing code that focuses on the object that the robot manipulates is not only
more readable, but also more robust and portable between robots. This tutorial shows you how to 
set subframes on collision objects, publish them to the planning scene and use them to plan motions. 

This video shows the output of this tutorial. The robot moves the tip of the cylinder to different positions on the box.
In the tutorial, you can control these motions interactively:

.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>

Running The Demo
----------------
After having completed the steps in `Getting Started <../getting_started/getting_started.html>`_, open two terminals. In the first terminal, execute this command to load up a panda, and wait for everything to finish loading: ::

    roslaunch panda_moveit_config demo.launch

In the second terminal run the tutorial: ::

    rosrun moveit_tutorials subframes_tutorial


The Code
---------------
The code for this example can be seen :codedir:`here <subframes>` in the moveit_tutorials GitHub project and is explained in detail below.

The code spawns a box and a cylinder in the planning scene, attaches the cylinder to the 
robot, and then lets you send motion commands via the command line. It also defines two 
convenience functions for sending a motion command, and for publishing the objects.

.. |br| raw:: html

   <br />

.. tutorial-formatter:: ./src/subframes_tutorial.cpp
