# MEGURU Project Repository
Welcome to the repository of MEGURU (**ME**ta-collaborative **G**est**U**re-based **R**obot program b**U**ilder) Project of Vis4Mechs, University of Brescia, Italy!<br>

<p align="center">
  <img height="400" src="https://github.com/Krissy93/meta-workstations-project/blob/master/images/MEGLOGO.png">
</p>

The project aims at developing a ROS-based structure to interact with both Industrial Robots and Collaborative Robots using smart modules. The system is an open source alternative to traditional robot programming methods, and requires little training to be used effectively.

If you use this code for your research or find it useful, please cite it:

```
@article{meguru2021,
author = "Nuzzi, Cristina and Pasinetti, Simone and Pagani, Roberto and Ghidini, Stefano and Beschi, Manuel and Coffetti, Gabriele and Sansoni, Giovanna",
title = "MEGURU: a gesture-based robot program builder for Meta-Collaborative workstations",
journal = "Robotics and Computer-Integrated Manufacturing",
volume = "68",
pages = "102085",
year = "2021",
issn = "0736-5845",
doi = "https://doi.org/10.1016/j.rcim.2020.102085",
url = "http://www.sciencedirect.com/science/article/pii/S0736584520302957"
}
```

```
@inproceedings{metacollaborative2019,
author="Nuzzi, Cristina and Pasinetti, Simone and Pagani, Roberto and Docchio, Franco and Sansoni, Giovanna",
editor="Cristani, Marco and Prati, Andrea and Lanz, Oswald and Messelodi, Stefano and Sebe, Nicu",
title="Hand Gesture Recognition for Collaborative Workstations: A Smart Command System Prototype",
booktitle="New Trends in Image Analysis and Processing -- ICIAP 2019",
year="2019",
publisher="Springer International Publishing",
address="Cham",
pages="332--342",
doi="10.1007/978-3-030-30754-7_33"
isbn="978-3-030-30754-7"}
```

## Maintainers
- Cristina Nuzzi, [Krissy93](https://github.com/Krissy93)
- Stefano Ghidini, [stefanoghidini](https://github.com/stefanoghidini)
- Roberto Pagani, [Roby-Pagani](https://github.com/Roby-Pagani)

## Table of Contents
- [Installation instructions](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Installation.md)
- [Overview](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Overview.md)
- [The gestures Language](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Gestures%20Node.md)
- [Points, Actions and Operations](https://github.com/Krissy93/meta-workstations-project/blob/master/docs/Movements%20Definition.md)

## Custom modules
If you want to contribute adding custom modules and functionalities to the software, feel free to do so! Contact us if you need help or want to work with us by sending an e-mail to [Cristina Nuzzi](mailto:c.nuzzi@unibs.it).

## "To do" list
- [ ] **Safety module:** 
    - reads environmental data from a 3D camera such as Kinect or RealSense
    - detects human presence and computes a safety hit-box around it
    - always knows robot safety hit-box
    - in real-time checks if the two hit-box are dangerously near, eventually lowering robot speed or completely stopping it
    
- [ ] **Automatic Point Acquisition module:**
    - move the robot in the space by pointing somewhere (e. g. from ROS Gazebo?)
    - automatically get robot points (XYZ/Joints)
