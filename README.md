# Code for interference with the Quanser Qube Servo 2 setup
This code is based on inspiritation from their sample code, select **Qube-Servo 2**: https://www.quanser.com/resource-type/technical-resources/ , latest accessed on **20-05-2024**

The code in this repository provides the following functions:
- Take measurements of the setup (voltage, angles, timestamp)
- Allow setup to operate in free-fall (Case 1)
- Apply white-noise to the setup (Case 2)
- Control the setup with a stabilizing controller in the pendulums unstable equilibrium $(\theta, \alpha) = (0, 0)$ (Case 3)


