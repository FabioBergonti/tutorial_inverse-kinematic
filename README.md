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
