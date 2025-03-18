# Baxter_arm_control

## Content 

 The goal is this lab is to control one arm of the Baxter robot, first in simulation then on the real robot.
 The robot is simulated with CoppeliaSim1. In the simulation is placed a green sphere. The goal is to move the arm so that the sphere is centered and at a given distance from the camera.
 The robot control will be performed in C++ and will rely on:
 - The ROS framework to handle communication between the simulator and the control
 program (actually hidden inside the robot class).
 - The ViSP library2 to manipulate vectors and matrices and to perform linear algebra.

## Environment setup 

ROS environment should already be set up in the computers. ROS 1 is used so you need to type: *ros1ws*
In order to use an IDE like Qt Creator, you should open a terminal and launch the IDE from the command line.
Atutorial on how to use Qt Creator (or any other IDE able to load CMake files) can be found on: http://pagesperso.ls2n.fr/~kermorgant-o/coding%20tools.html#config.

This lab is available on GitHub as a ROS package called ecn_sensorbased. To donwnload it manually, you should first go in the ros/src directory. The package can
then be downloaded through git: 
```
git clone https://github.com/oKermorgant/ecn_baxter_vs.git
```
Remember to call catkin build after downloading the packages in your ROS workspace, and
before trying to load it in the IDE.

## Structure of the *ecn_baxter_vs* Package
The only file to be modified is main.cpp, which is the main file for the C++ program.
The simulation can be launched with: roslaunch ecn_baxter_vs sim.launch.
When both the simulation and the control program run, the ROS graph looks like this:

![image](https://github.com/user-attachments/assets/27e056b9-84af-48f0-93b5-bd43d3a1b461)

## Tasks

### 1) Basic visual servoing
The first goal is to actually compute a classical visual servoing. The features are defined as s =(x,y,a) where (x,y) is the position of the center of gravity of the sphere and a is its area
in the image. The current features are accessed through *arm.x()*, *arm.y()*, *arm.area()* while the desired ones are set to 0 for (x,y) and accessed through arm.area_d() for the area. The error vector
e has to be defined from the current and desired features.

Similarly, the interaction matrix of s should be defined in L. The feature Jacobian, that expresses the time-variation of s with regards to $\dot{q}$ should then be computed from L and
arm.cameraJacobian(). The desired joint velocities can then be computed with $\dot{q} = -\lambda J^+ e$. Of course this control law does not take into account the joint limits.

The gain $\lambda$ should be equal to arm.lambda(), in order to be tuned online from the Baxter image window slider.


### 2) Avoiding useless rotations

The 3 current features allow any rotation around the Z-axis of the camera as it does not change the object in the image. In order to stabilize the system we add a constraint that is having the
X-axis of the camera orthogonal to the Z-axis of the base frame. 

The rotation matrix $^bR_c$ can be obtained with:

```cpp
const auto bRc = arm.cameraPose().getRotationMatrix();
```

Knowing that $\dot{^b R_c} = {^b R_c} [\omega_c]_\times$, express the interaction matrix of this new feature and add it to the control.


### 3) Visual servoing with joint limits

In this framework, constraints are modeled as additional sensor features.  
The desired value corresponds to the avoidance direction and is typically defined using the center of the valid interval:


$q^* = \frac{1}{2} (q^+ + q^-)$


A **safe interval** $(q_s^-, q_s^+)$ is defined with a margin $\rho \in ]0, 0.5[$ as follows:

$q_s^+ = q^+ - \rho (q^+ - q^-)$

$q_s^- = q^- + \rho (q^+ - q^-)$

The margin $\rho$ can be **tuned online** through a slider, and its current value is available via:

```cpp
arm.rho()
```

### 4) Visual servoing and joint control with joint limits

If we use the above formulation, the robot successfully avoids its joint limits, but the control law may become **discontinuous**.  
This occurs because $(\tilde{H} J)$ is an $11 \times 7$ matrix whose rank may vary:

- **Rank 4** → When all joints are within their safe zone.
- **Rank 7** → When at least **three** joints are close to their limits.

### **Solution: Setting Arbitrary Values for Three Joints**
To maintain continuity in the control law, a solution is to set **arbitrary values** for three joints.  
A good approach is to use the **default values** (output of `arm.init()`) as the desired values for **joints 3 to 7** and keep their weight **equal to 1**.

- **Joints 1 to 4** → Used for **visual servoing** while avoiding their limits.
- **Joints 3 to 7** → Set to default values (`arm.init()`), ensuring smooth behavior.

```cpp
// Example: Retrieve default joint values
auto default_joints = arm.init();


