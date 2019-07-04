# Meta Workstations Project Repository
Welcome to the repository of the Meta Workstation Project of Vis4Mechs, University of Brescia, Italy!<br>
The project aims at developing a ROS-based structure to interact with both Industrial Robots and Collaborative Robots using smart modules. The system is an open source alternative to traditional robot programming methods, and requires little training to be used effectively.

If you use this code for your research or find it useful, please cite it:

## Maintainers
- Cristina Nuzzi, [Krissy93](https://github.com/Krissy93)

## Table of Contents
- [Installation instructions](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Installation.md)
- [Overview]
- [The code explained]

## Custom modules
If you want to contribute adding custom modules and functionalities to the software, feel free to do so! Contact us if you need help or want to work with us by sending an e-mail to [Cristina Nuzzi](mailto:c.nuzzi@unibs.it).

## "To do" list
- **Safety module:** 
    - reads environmental data from a 3D camera such as Kinect or RealSense
    - detects human presence and computes a safety hit-box around it
    - always knows robot safety hit-box
    - in real-time checks if the two hit-box are dangerously near, eventually lowering robot speed or completely stopping it
