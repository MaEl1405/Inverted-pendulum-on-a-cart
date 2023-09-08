# Inverted-pendulum-on-a-cart
[Inverted Pendulum swing up](./animations/Swing_up_InvertedP.mp4.mp4)

This project is a simulation and control implementation of an inverted pendulum mounted on a cart. The system's dynamics are modeled and controlled using MATLAB. The following files are part of this project:

1. `InvPendOnCart.m`: This MATLAB script contains the main code for simulating the inverted pendulum on a cart system. It sets up the system parameters, initializes the simulation, and visualizes the results. this functios give all states over time.

2. `odeSolver.m`: This MATLAB script contains a custom ordinary differential equation (ODE) solver used in the simulation. It  include numerical integration methods to solve the equations of motion for the system. methods like Rhngekutta4,Euler ,ode45.

3. `lqrControl.m`: This MATLAB script implements the Linear Quadratic Regulator (LQR) controller for stabilizing and controlling the inverted pendulum system. It designs and applies the control law to maintain balance.

4. `RK4.m`: This MATLAB script  contain the implementation of the Runge-Kutta 4th-order numerical integration method used in the simulation. this function output is next_state .
5. `EnergyShaping_Lqr_SwingUp.m`: This MATLAB script  implement an energy-shaping control strategy combined with LQR to perform the swing-up and stabilization of the inverted pendulum.

## Getting Started

To run this project on your local machine, follow these steps:

1. Ensure you have MATLAB installed on your computer.

2. Clone or download this repository to your local machine.

3. Open MATLAB and navigate to the project directory.

4. Run `lqrControl.m or EnergyShaping_Lqr_SwingUp.m ` to see outputs
## Usage

You can customize and extend this project in several ways:

- Adjust system parameters such as pendulum length, cart mass, and control gains in `You can customize and extend this project in several ways:

- Adjust system parameters such as pendulum length, cart mass, and control gains in `EnergyShaping_Lqr_SwingUp.m or lqrControl.m`.

- Modify the control strategy in `lqrControl.m or EnergyShaping_Lqr_SwingUp.m` to experiment with different control algorithms.

## Contributing

If you would like to contribute to this project, please open an issue or submit a pull request. We welcome improvements, bug fixes, and new features.


## Acknowledgments

Special thanks to JITENDRA SINGH for the paper "Model-Based Control Design for Swing-up & Balance of the Inverted Pendulum." The information from this paper was used in the design of the swing-up controller for this project.


