<style TYPE="text/css">
code.has-jax {font: inherit; font-size: 100%; background: inherit; border: inherit;}
</style>
<script type="text/x-mathjax-config">
MathJax.Hub.Config({
    tex2jax: {
        inlineMath: [['$','$'], ['\\(','\\)']],
        skipTags: ['script', 'noscript', 'style', 'textarea', 'pre'] // removed 'code' entry
    }
});
MathJax.Hub.Queue(function() {
    var all = MathJax.Hub.getAllJax(), i;
    for(i = 0; i < all.length; i += 1) {
        all[i].SourceElement().parentNode.className += ' has-jax';
    }
});
</script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.4/MathJax.js?config=TeX-AMS_HTML-full"></script>

# The C++ Project Readme #

This is the readme for the C++ project.

For easy navigation throughout this document, here is an outline:

 - [Development environment setup](#development-environment-setup)
 - [Simulator walkthrough](#simulator-walkthrough)
 - [The tasks](#the-tasks)
 - [Evaluation](#evaluation)


## Development Environment Setup ##

Regardless of your development platform, the first step is to download or clone this repository.

Once you have the code for the simulator, you will need to install the necessary compiler and IDE necessary for running the simulator.

Here are the setup and install instructions for each of the recommended IDEs for each different OS options:

### Windows ###

For Windows, the recommended IDE is Visual Studio.  Here are the steps required for getting the project up and running using Visual Studio.

1. Download and install [Visual Studio](https://www.visualstudio.com/vs/community/)
2. Select *Open Project / Solution* and open `<simulator>/project/Simulator.sln`
3. From the *Project* menu, select the *Retarget solution* option and select the Windows SDK that is installed on your computer (this should have been installed when installing Visual Studio or upon opening of the project).
4. Make sure platform matches the flavor of Windows you are using (x86 or x64). The platform is visible next to the green play button in the Visual Studio toolbar:

![x64](x64.png)

5. To compile and run the project / simulator, simply click on the green play button at the top of the screen.  When you run the simulator, you should see a single quadcopter, falling down.


### OS X ###

For Mac OS X, the recommended IDE is XCode, which you can get via the App Store.

1. Download and install XCode from the App Store if you don't already have it installed.
2. Open the project from the `<simulator>/project` directory.
3. After opening project, you need to set the working directory:
  1. Go to *(Project Name)* | *Edit Scheme*
  2. In new window, under *Run/Debug* on left side, under the *Options* tab, set Working Directory to `$PROJECT_DIR` and check ‘use custom working directory’.
  3. Compile and run the project. You should see a single quadcopter, falling down.


### Linux ###

For Linux, the recommended IDE is QtCreator.

1. Download and install QtCreator.
2. Open the `.pro` file from the `<simulator>/project` directory.
3. Compile and run the project (using the tab `Build` select the `qmake` option.  You should see a single quadcopter, falling down.

**NOTE:** You may need to install the GLUT libs using `sudo apt-get install freeglut3-dev`


### Advanced Versions ###

These are some more advanced setup instructions for those of you who prefer to use a different IDE or build the code manually.  Note that these instructions do assume a certain level of familiarity with the approach and are not as detailed as the instructions above.

#### CLion IDE ####

For those of you who are using the CLion IDE for developement on your platform, we have included the necessary `CMakeLists.txt` file needed to build the simulation.

#### CMake on Linux ####

For those of you interested in doing manual builds using `cmake`, we have provided a `CMakeLists.txt` file with the necessary configuration.

**NOTE: This has only been tested on Ubuntu 16.04, however, these instructions should work for most linux versions.  Also note that these instructions assume knowledge of `cmake` and the required `cmake` dependencies are installed.**

1. Create a new directory for the build files:

```sh
cd FCND-Controls-CPP
mkdir build
```

2. Navigate to the build directory and run `cmake` and then compile and build the code:

```sh
cd build
cmake ..
make
```

3. You should now be able to run the simulator with `./CPPSim` and you should see a single quadcopter, falling down.

## Simulator Walkthrough ##

Now that you have all the code on your computer and the simulator running, let's walk through some of the elements of the code and the simulator itself.

### The Code ###

For the project, the majority of your code will be written in `src/QuadControl.cpp`.  This file contains all of the code for the controller that you will be developing.

All the configuration files for your controller and the vehicle are in the `config` directory.  For example, for all your control gains and other desired tuning parameters, there is a config file called `QuadControlParams.txt` set up for you.  An import note is that while the simulator is running, you can edit this file in real time and see the affects your changes have on the quad!

The syntax of the config files is as follows:

 - `[Quad]` begins a parameter namespace.  Any variable written afterwards becomes `Quad.<variablename>` in the source code.
 - If not in a namespace, you can also write `Quad.<variablename>` directly.
 - `[Quad1 : Quad]` means that the `Quad1` namespace is created with a copy of all the variables of `Quad`.  You can then overwrite those variables by specifying new values (e.g. `Quad1.Mass` to override the copied `Quad.Mass`).  This is convenient for having default values.

You will also be using the simulator to fly some difference trajectories to test out the performance of your C++ implementation of your controller. These trajectories, along with supporting code, are found in the `traj` directory of the repo.


### The Simulator ###

In the simulator window itself, you can right click the window to select between a set of different scenarios that are designed to test the different parts of your controller.

The simulation (including visualization) is implemented in a single thread.  This is so that you can safely breakpoint code at any point and debug, without affecting any part of the simulation.

Due to deterministic timing and careful control over how the pseudo-random number generators are initialized and used, the simulation should be exactly repeatable. This means that any simulation with the same configuration should be exactly identical when run repeatedly or on different machines.

Vehicles are created and graphs are reset whenever a scenario is loaded. When a scenario is reset (due to an end condition such as time or user pressing the ‘R’ key), the config files are all re-read and state of the simulation/vehicles/graphs is reset -- however the number/name of vehicles and displayed graphs are left untouched.

When the simulation is running, you can use the arrow keys on your keyboard to impact forces on your drone to see how your controller reacts to outside forces being applied.

#### Keyboard / Mouse Controls ####

There are a handful of keyboard / mouse commands to help with the simulator itself, including applying external forces on your drone to see how your controllers reacts!

 - Left drag - rotate
 - X + left drag - pan
 - Z + left drag - zoom
 - arrow keys - apply external force
 - C - clear all graphs
 - R - reset simulation
 - Space - pause simulation




### Testing it Out ###

When you run the simulator, you'll notice your quad is falling straight down.  This is due to the fact that the thrusts are simply being set to:

```
QuadControlParams.Mass * 9.81 / 4
```

Therefore, if the mass doesn't match the actual mass of the quad, it'll fall down.  Take a moment to tune the `Mass` parameter in `QuadControlParams.txt` to make the vehicle more or less stay in the same spot.

Note: if you want to come back to this later, this scenario is "1_Intro".

With the proper mass, your simulation should look a little like this:

<p align="center">
<img src="animations/scenario1.gif" width="500"/>
</p>


## The Tasks ##

For this project, you will be building a controller in C++.  You will be implementing and tuning this controller in several steps.

You may find it helpful to consult the [Python controller code](https://github.com/udacity/FCND-Controls/blob/solution/controller.py) as a reference when you build out this controller in C++.

#### Notes on Parameter Tuning
1. **Comparison to Python**: Note that the vehicle you'll be controlling in this portion of the project has different parameters than the vehicle that's controlled by the Python code linked to above. **The tuning parameters that work for the Python controller will not work for this controller**

2. **Parameter Ranges**: You can find the vehicle's control parameters in a file called `QuadControlParams.txt`. The default values for these parameters are all too small by a factor of somewhere between about 2X and 4X. So if a parameter has a starting value of 12, it will likely have a value somewhere between 24 and 48 once it's properly tuned.

3. **Parameter Ratios**: In this [one-page document](https://www.overleaf.com/read/bgrkghpggnyc#/61023787/) you can find a derivation of the ratio of velocity proportional gain to position proportional gain for a critically damped double integrator system. The ratio of `kpV / kpP` should be 4.

### Body rate and roll/pitch control (scenario 2) ###

First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, you will:

0. Drone representation
  ![](images/drone1_1.png) ![](images/Drone2.png)

1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`

   We will define the forces generated by the propellers $F_1 = k_f\omega^2_1$, $F_2 = k_f\omega^2_2$, $F_3 = k_f\omega^2_3$, $F_4 = k_f\omega^2_4$. The collective force directed upward $F_{total} = F_1 + F_2 + F_3 + F_4$.

   Next, we will keep track of the moments created by the propellers.   For roll motion, the moments generated by the first and fourth propellers are counteracted by the moment generated by the second and the third propellers. The $l$ is a distance between x-axis and propeller location, which is equal to half of the distance between neighboring propellers. In the same fashion, the pitch is generated by the mismatch of the moments created by first and second propellers and the moment generated by the third and fourth propellers.

   Contrary to the roll and pitch the yaw motion is executed by the mismatch of the moments generated by the propellers along the $z$ axis by the reactive force. The moment generated by the propeller is directed opposite of its rotation and is proportional to the square of the angular velocities.  

    $$
    \begin{align}
    \tau_x &= (F_1 + F_4 - F_2 - F_3)l \\
    \tau_y &= (F_1 + F_2 - F_3 - F_4)l \\
    \tau_z &= \tau_1 + \tau_2 + \tau_3 + \tau_4
    \end{align}
    $$

    Where $\tau_1 = - k_m \omega^2_1$, $\tau_2 = k_m \omega^2_2$, $\tau_3 = -k_m \omega^2_3$, $\tau_4 = k_m \omega^2_4$. In our notation, the propellers 1 and 3 rotate in clockwise thus producing the moment in the counterclockwise direction with negative sign and propellers 2 and 4 rotate in counterclockwise thus the resulting moments are in opposite and have the positive signs.  

   Based on the input from the controller we can set the angular velocities of the propellers. For this, we would like to solve the next linear equation. The first row represents the vertical acceleration, the second represents the equation for roll, the third row is the pitch equation, and the last one is derived from the yaw equation.

     $$
     \begin{pmatrix}
     1 & 1 & 1 & 1 \\
     1 & -1 & -1 & 1 \\
     1 & 1 & -1 & -1\\
     -1 & 1 & -1 & 1
     \end{pmatrix} \times \begin{pmatrix} \omega^2_1 \\ \omega^2_2 \\ \omega^2_3\\ \omega^2_4 \end{pmatrix} = \begin{pmatrix} \bar{c} \\ \bar{p} \\ \bar{q} \\ \bar{r} \end{pmatrix}
     $$

     We need to transform the inputs from the controller into the $\bar{c}$, $\bar{p}$, $\bar{q}$, $\bar{r}$ variables.

     The $c$ value received from the controller is equal to $\frac{F}{m} = c$, For equation $(\omega^2_1 + \omega^2_2 + \omega^2_3 + \omega^2_4) = \bar{c}$ the proper transformation will be $\bar{c} = \frac{F}{k_f} = \frac{c m }{k_f}$ (which transforms thrust in dimensionless form).

     For roll, pitch and yaw $\bar{u}_p = \dot{p} = \frac{\tau_x}{I_x}$ , $\bar{u}_q = \dot{q} = \frac{\tau_y}{I_y}$ , $\bar{u}_r = \dot{r} = \frac{\tau_z}{I_z}$. Transforming them into dimensionless form
     $$
     \begin{align}
     \bar{p} &= \frac{I_x \bar{u}_p}{k_f l } \\
     \bar{q} &= \frac{I_y \bar{u}_q}{k_f l } \\
     \bar{r} &= \frac{I_z \bar{u}_r}{k_m } \\
     \end{align}
     $$

     In the code, kappa is the ratio $\kappa=\frac{k_m}{k_f}$ since $k_m$ is the torque (Nm) produced for 1 rad/s of motor speed and $k_f$ is the force produced for 1 rad/s of motor speed, and that kappa is the torque (Nm) produced by motor per N of thrust produced

     And in fact this function is not setting the rotation speed of each motor, but the desired thrust $F_i$ for each motor. We thus need to refactor the above equations so that we can have the force for each motor $F_i$ with regards to the torques $\tau_x, \tau_y, \tau_z$ and the collective thrust $F_{Total}$.

     From above equations, we can derive that $F_1 = \frac{-\tau_1}{\kappa}, F_2 = \frac{\tau_2}{\kappa}, F_3 = \frac{-\tau_3}{\kappa}, F_4 = \frac{\tau_4}{\kappa}$

     $$
     \begin{pmatrix}
     l & -l & -l & l \\
     l & l & -l & -l \\
     -\kappa & \kappa & -\kappa & \kappa\\
     1 & 1 & 1 & 1
     \end{pmatrix} \times \begin{pmatrix} F_1 \\ F_2 \\ F_3 \\ F_4 \end{pmatrix} = \begin{pmatrix} \tau_x \\ \tau_y \\ \tau_z \\ F_{Total}\end{pmatrix}
     $$

     Which is, solved for $F_1, F_2, F_3, F_4$:
     $$
     \frac{1}{4} *
     \begin{pmatrix}
      1/l & 1/l & -1/\kappa & 1 \\
      -1/l & 1/l & 1/\kappa & 1 \\
      -1/l & -1/l & -1/\kappa & 1\\
      1/l & -1/l & 1/\kappa & 1
     \end{pmatrix} \times \begin{pmatrix} \tau_x \\ \tau_y \\ \tau_z \\ F_{Total} \end{pmatrix} = \begin{pmatrix} F_1 \\ F_2 \\ F_3 \\ F_4\end{pmatrix}
     $$

 - implement the code in the function `BodyRateControl()`.
   The body rate control is simply a P controller.

 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot

If successful, you should see the rotation of the vehicle about roll (omega.x) get controlled to 0 while other rates remain zero.  Note that the vehicle will keep flying off quite quickly, since the angle is not yet being controlled back to 0.  Also note that some overshoot will happen due to motor dynamics!.

If you come back to this step after the next step, you can try tuning just the body rate omega (without the outside angle controller) by setting `QuadControlParams.kpBank = 0`.

2. Implement roll / pitch control
We won't be worrying about yaw just yet.

 - implement the code in the function `RollPitchControl()`
 - Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

If successful you should now see the quad level itself (as shown below), though it’ll still be flying away slowly since we’re not controlling velocity/position!  You should also see the vehicle angle (Roll) get controlled to 0.

<p align="center">
<img src="animations/scenario2.gif" width="500"/>
</p>

For `RollPitchControl()`, we want to return a V3F containing the desired pitch and roll rates. The Z element of the V3F should be left at its default value (0).

We receive the commanded thrust and the desired acceleration in global XY coordinates as input. We first need to convert this commanded thrust into an acceleration.  The drone generates lateral acceleration by changing the body orientation which results in non-zero thrust in the desired direction. This will translate into the commanded rotation matrix elements  $b_c^x$  and  $b_c^y$ . The control equations have the following form:

$$
\begin{align}
\ddot{x}_{\text{command}} &=  c b^x_c \\
b^x_c &= \ddot{x}_{\text{command}}/c
\end{align}
$$

With $c = -\text{commandedThrust}/\text{mass}$, we have that $b_c^{xy} = \text{accelCmd} / c$


The roll-pitch controller is a P controller responsible for commanding the roll and pitch rates ($p_c$ and $q_c$) in the body frame.  First, it sets the desired rate of change of the given matrix elements using a P controller.

**Note** - subscript c means "commanded" and a means "actual"

$\dot{b}^x_c  = k_p(b^x_c - b^x_a)$

$\dot{b}^y_c  = k_p(b^y_c - b^y_a)$

where $b^x_a = R_{13}$ and $b^y_a = R_{23}$. The given values can be converted into the angular velocities into the body frame by the next matrix multiplication.

$$
\begin{pmatrix} p_c \\ q_c \\ \end{pmatrix}  = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c  \end{pmatrix}
$$


### Position/velocity and yaw angle control (scenario 3) ###

Next, you will implement the position, altitude and yaw control for your quad.  For the simulation, you will use `Scenario 3`.  This will create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

 - implement the code in the function `LateralPositionControl()`
   For the lateral controller, we receive as input actual and commanded position, aswell as the actual and commanded speeds and the acceleration command feed forward. We need to output acceleration command. For this the lateral controller will use a PD controller to command target values for elements of the drone's rotation matrix.
   $$
   \text{accelCmd} = \text{accelCmdFF} + k_p * (\text{pos}_c - \text{pos}_a) + k_d * (\text{vel}_c - \text{vel}_a)
   $$

 - implement the code in the function `AltitudeControl()`
   For the AltitudeControl we receive as input the commanded and actual vertical positions and speeds, aswell as the feed forward vertical acceleration. We must output the collective thrust in Newtons.
   Since $F = m*c$ we must compute the collective thrust acceleration.

   Linear acceleration can be expressed by the next linear equation
    $$
    \begin{pmatrix} \ddot{x} \\ \ddot{y} \\ \ddot{z}\end{pmatrix}  = \begin{pmatrix} 0 \\ 0 \\ g\end{pmatrix} + R \begin{pmatrix} 0 \\ 0 \\ c \end{pmatrix}
    $$

    where $R = R(\psi) \times R(\theta) \times R(\phi)$. The individual linear acceleration has the form of

    $$
    \begin{align}
    \ddot{x} &= c b^x \\
    \ddot{y} &= c b^y \\
    \ddot{z} &= c b^z +g
    \end{align}
    $$
    where $b^x = R_{13}$, $b^y= R_{23}$ and $b^z = R_{33}$ are the elements of the last column of the rotation matrix.

    We are controlling the vertical acceleration:

    $$\bar{u}_1 = \ddot{z} = c b^z +g$$

    Therefore

    $$c = (\bar{u}_1-g)/b^z$$  


    In this exercise a PD controller is used for the altitude which results in:

    $$\bar{u}_1 = k_{p-z}(z_{t} - z_{a}) + k_{d-z}(\dot{z}_{t} - \dot{z}_{a}) + \ddot{z}_t$$

 - tune parameters `kpPosZ` and `kpPosZ`
 - tune parameters `kpVelXY` and `kpVelZ`

If successful, the quads should be going to their destination points and tracking error should be going down (as shown below). However, one quad remains rotated in yaw.

 - implement the code in the function `YawControl()`

 Control over yaw is decoupled from the other directions. A P controller is used to control the drone's yaw.

 $r_c = k_p (\psi_t - \psi_a)$
 - tune parameters `kpYaw` and the 3rd (z) component of `kpPQR`

Tune position control for settling time. Don’t try to tune yaw control too tightly, as yaw control requires a lot of control authority from a quadcopter and can really affect other degrees of freedom.  This is why you often see quadcopters with tilted motors, better yaw authority!

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

**Hint:**  For a second order system, such as the one for this quadcopter, the velocity gain (`kpVelXY` and `kpVelZ`) should be at least ~3-4 times greater than the respective position gain (`kpPosXY` and `kpPosZ`).

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

1. Run your controller & parameter set from Step 3.  Do all the quads seem to be moving OK?  If not, try to tweak the controller parameters to work for all 3 (tip: relax the controller).

2. Edit `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

First make sure the velocity is between 0 and `maxDescentRate` if the velocity is positive (going down) and between `-maxAscentRate` and 0 if the velocity is negative (going up) since we're in NED coordinate system.

Then the altitude command is a PID controller controlling the vertical acceleration. I also implemented a clamping mechanism for the integral part to avoid integral windup. So the vertical acceleration is `u_bar = kpPosZ * errorPosZ + kpVelZ * errorVelZ + KiPosZ * integratedError` where integratedError is clamped and equal to `dt * errorPosZ`. The collective thrust is given by $c = (\bar{u} - g)/b^z$ where $b^z = R_{33}$. Since we ened the thrust (N), in the NED coordinate system, we have `thrust = -mass * c`

3. Tune the integral control, and other control parameters until all the quads successfully move properly.  Your drones' motion should look like this:

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>


### Tracking trajectories ###

Now that we have all the working parts of a controller, you will put it all together and test it's performance once again on a trajectory.  For this simulation, you will use `Scenario 5`.  This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.  For those interested in seeing how you might be able to improve the performance of your drone by adjusting how the trajectory is defined, check out **Extra Challenge 1** below!

How well is your drone able to follow the trajectory?  It is able to hold to the path fairly well?


### Extra Challenge 1 (Optional) ###

You will notice that initially these two trajectories are the same. Let's work on improving some performance of the trajectory itself.

1. Inspect the python script `traj/MakePeriodicTrajectory.py`.  Can you figure out a way to generate a trajectory that has velocity (not just position) information?

2. Generate a new `FigureEightFF.txt` that has velocity terms
Did the velocity-specified trajectory make a difference? Why?

Adding the velocities gives more information about the desired state of the drone at each point in the trajectory, and thus allows it to better predict where it should be and act accordingly.

With the two different trajectories, your drones' motions should look like this:

<p align="center">
<img src="animations/scenario5.gif" width="500"/>
</p>


### Extra Challenge 2 (Optional) ###

For flying a trajectory, is there a way to provide even more information for even better tracking?

How about trying to fly this trajectory as quickly as possible (but within following threshold)!

-> I managed to do the figure eight loop 12% faster, and the drone is still within the error margins.

## Evaluation ##

To assist with tuning of your controller, the simulator contains real time performance evaluation.  We have defined a set of performance metrics for each of the scenarios that your controllers must meet for a successful submission.

There are two ways to view the output of the evaluation:

 - in the command line, at the end of each simulation loop, a **PASS** or a **FAIL** for each metric being evaluated in that simulation
 - on the plots, once your quad meets the metrics, you will see a green box appear on the plot notifying you of a **PASS**


### Performance Metrics ###

The specific performance metrics are as follows:

 - scenario 2
   - roll should less than 0.025 radian of nominal for 0.75 seconds (3/4 of the duration of the loop)
   - roll rate should less than 2.5 radian/sec for 0.75 seconds

 - scenario 3
   - X position of both drones should be within 0.1 meters of the target for at least 1.25 seconds
   - Quad2 yaw should be within 0.1 of the target for at least 1 second


 - scenario 4
   - position error for all 3 quads should be less than 0.1 meters for at least 1.5 seconds

 - scenario 5
   - position error of the quad should be less than 0.25 meters for at least 3 seconds

## Authors ##

Thanks to Fotokite for the initial development of the project code and simulator.
