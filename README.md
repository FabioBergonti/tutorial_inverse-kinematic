Tutorial on Inverse Kinematic
=============================

This tutorial builds on top [tutorial_impedance-control](https://github.com/vvv-school/tutorial_impedance-control) to implement a simple inverse kinematic controller.
The robot will try to move the end effector (left hand) in a desired configuration.
The inverse kinematic is resolved using quadratic programming via osqp.
The optimisation problem is formulated as
    
$$
\begin{align}
\min_{\mathbf{\nu} = \begin{bmatrix} v_b \\\\ \dot{s} \end{bmatrix}} & \quad \frac{1}{2} {\|\| J_{ee}^{pos} \nu - v^{\star} \|\|}_ 2^ 2 + k_{\nu} {\|\| \nu \|\|}_ 2^ 2 \\
\text{s.t.} \quad & J_B \nu = 0 \\
& \dot{s}^{-} \leq \dot{s} \leq \dot{s}^{+} \\
& v^{\star} = -K_p \left( x_{ee} - x_{ee}^{des} \right)
\end{align}
$$

$\nu$ is the vector of configuration velocities, $v_b$ is the base velocity, $\dot{s}$ is the joint velocity.
$J_{ee}^{pos}$ is the Jacobian of the end effector position, $J_B$ is the Jacobian of the base.
$\dot{s}^{-}$ and $\dot{s}^{+}$ are the lower and upper bounds on the joint velocity.
$x_{ee}$ is the end effector position and $x_{ee}^{des}$ is the desired end effector position, and $K_p$ is the proportional gain.

## Installation

- Install and Configure [ironcub-mk1-software](https://github.com/ami-iit/ironcub-mk1-software)
- Build the repository
```bash
mkdir build
cd build
cmake ..
make install
```

## Usage

- run yarpserver
```bash
yarpserver
```
- run yarpmanager
```bash
yarpmanager
```
- open the application `tutorial_inverse-kinematic-system.xml` in yarpmanager and run all the modules
- open the application `tutorial_inverse-kinematic-app.xml` in yarpmanager and run the controller

https://github.com/FabioBergonti/tutorial_inverse-kinematic/assets/38210073/19d25541-a3d8-4037-8abe-c938127b8c4d
