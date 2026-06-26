<h1><img src="docs/assets/images/logo.png" alt="logo" height="40" style="vertical-align:middle; margin-right:8px;" /> Quad-Quad Bot</h1>
<p>
  Custom desktop-size quadruped robot - from design, and CAD to ROS 2 simulation, and a physical build.
</p>

<hr/>

<img src="/docs/assets/images/gazebo_trot.gif" alt="Gazebo simulation" width="750"/>

## About

**Quad-Quad Bot** is a quadruped (four-legged) robot built by me as a solo side-project. It spans the full mechatronics stack:

- **Mechanical** — from-scratch design, CAD in SolidWorks, and 3D-printed parts.
- **Electronics** — custom power distribution, motor drivers, onboard compute
- **Software** — ROS 2 (Jazzy), Gazebo Harmonic simulation, inverse kinematics, gait planning, and feedback loops.

<i> Current status: simulation is running with a trot gait. Hardware assembly is in progress. </i>

<hr/>

## Gallery

<table>
  <tr>
    <td align="center" width="33%">
      <img src="docs/assets/images/current_cad_body.png" alt="CAD render" width="100%"/>
      <br/><sub><b>CAD Model</b></sub>
    </td>
    <td align="center" width="33%">
      <img src="/docs/assets/images/gazebo_trot.gif" alt="Gazebo simulation" width="100%"/>
      <br/><sub><b>Gazebo Trot</b></sub>
    </td>
    <td align="center" width="33%">
      <img src="docs/assets/images/physical.jpg" alt="Physical robot" width="100%"/>
      <br/><sub><b>Hardware <i>(WIP)</i></b></sub>
    </td>
  </tr>
</table>

<hr/>

## Repository Structure

```
quad-quad-bot/
├── ros2_ws/        # ROS 2 workspace — all software (nodes, controllers, bringup)
├── design/
│   ├── cad/        # SolidWorks source files
│   ├── slicer/     # 3D print slicer projects
│   └── electrical/ # Wiring diagrams
└── docs/           # Technical documentation (MkDocs)
```

<hr/>

## Documentation
Full technical documentation can be found [here](quad-quad-bot.andriibessarab.com).

<hr/>

<p><sub>Built by <a href="https://github.com/andriibessarab">Andrii Bessarab</a></sub></p>