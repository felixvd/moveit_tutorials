Subframes Tutorial
============================

This tutorial shows you how to set subframes on collision objects, publish them to the planning scene and use them to plan motions. This allows you to write, for example, "move the tip of the screwdriver to the head of the screw".

This video shows the output of this tutorial:

.. raw:: html

    <div style="position: relative; padding-bottom: 5%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe width="700px" height="400px" src="https://www.youtube.com/embed/QBJPxx_63Bs?rel=0" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
    </div>

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

Running The Demo
----------------
Open two terminals. In the first terminal, execute this command to load up a panda, and wait for everything to finish loading: ::

    roslaunch panda_moveit_config demo.launch

In the second terminal run the tutorial: ::

    rosrun moveit_tutorials subframes_tutorial


The Code
---------------
The code for this example can be seen :codedir:`here <subframes>` in the moveit_tutorials GitHub project and is explained in detail below.

.. |br| raw:: html

   <br />

.. tutorial-formatter:: ./src/subframes_tutorial.cpp
