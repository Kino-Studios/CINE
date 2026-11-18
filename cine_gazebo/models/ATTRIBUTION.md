# Third-party assets in the Gazebo world and drone model

## Walking person (`subject` actor in `worlds/cine_field.sdf.template`)

- Asset: Gazebo Fuel model `Mingfei/actor`, version 1 (uploaded 2019-08-21), skin and
  animation `meshes/walk.dae`.
- Fetched by Gazebo at run time from
  `https://fuel.gazebosim.org/1.0/Mingfei/models/actor/1/files/meshes/walk.dae`; the
  version number in the URL pins it.
- `walk.dae` SHA-256 on 2026-11-14:
  `49af0df3a319d1cb8ca2cebf02dbd00f625e5d5bec820bc5e109925b18b65c6e` (2,277,322 bytes).
- Author: Mingfei, on Gazebo Fuel (https://app.gazebosim.org/Mingfei/fuel/models/actor).
- License: Creative Commons Attribution 4.0 International
  (http://creativecommons.org/licenses/by/4.0/). Used unmodified.

## Drone model (`models/cine_iris/model.sdf.template`)

- Derived from `models/iris_with_ardupilot/model.sdf` of ArduPilot's `ardupilot_gazebo`,
  commit `f116b03552d05c8d0b0bed5ed67033a1183121d1`
  (https://github.com/ArduPilot/ardupilot_gazebo), licensed LGPL-3.0. Its model.config
  credits Fadri Furrer, Michael Burri, Mina Kamel, Janosch Nikolic and Markus Achtelik,
  maintained by John Hsu.
- Changes: renamed to `cine_iris`; commented-out debugging markers dropped; added a fixed
  camera link with the Pi camera's lens, wind on every link, and a pose publisher.
- `iris_with_standoffs` and its meshes are not copied; they are included from the same
  `ardupilot_gazebo` checkout at run time.

## Source One drone model (`models/cine_source_one/model.sdf.template`)

- The ArduPilot plugin block, the rotor joint and `LiftDrag` layout and the pose and joint
  state publishers follow the same `iris_with_ardupilot` file (commit
  `f116b03552d05c8d0b0bed5ed67033a1183121d1`, LGPL-3.0, credits as above).
- Changes: the Iris meshes are replaced by the owner's meshes (below); mass, inertia, motor
  positions, blade count and lift numbers are the TBS Source One build's; the ArduPilot
  plugin's multiplier is 3666.
- Visual meshes (`models/cine_source_one/meshes/`), committed:
  - `carbon.obj`, and the frame parts in `airframe.obj`: TBS Source One V5 CAD,
    `SO_V5.iges` from Team BlackSheep's `tbs-trappy/source_one` repository
    (https://github.com/tbs-trappy/source_one), licensed GPL-3.0. Converted to meshes,
    re-coloured and given planar texture coordinates.
  - `pi5.obj`: Raspberry Pi 5 STEP model
    (https://datasheets.raspberrypi.com/rpi5/RaspberryPi5-step.zip), MIT licence,
    "Copyright (c) 2026 Raspberry Pi Ltd". Decimated to about 44k triangles.
  - `camera.obj`: Camera Module 3 Wide STEP model
    (https://pip-assets.raspberrypi.com/categories/1207-design-files/documents/RP-008154-DS-1-camera-module-3-step.zip,
    terms at https://www.raspberrypi.com/licensing/), which Raspberry Pi permits
    anyone "to use, copy, modify, and/or distribute ... for any purpose". Decimated to
    about 23k triangles.
  - The rest of `airframe.obj`: the owner's CAD STLs in `models/cine_source_one/cad/`
    (battery box, battery wire part, Pi adapter plate, camera mount) and generated pieces
    (standoffs, wiring, capacitor, XT60), the project owner's own work.
  - `carbon_twill.png`: generated, the project owner's own work.
- Visual meshes named `restricted_*`, ignored by git here because their sources do not
  allow redistribution. They and their CAD sources live in the private
  `Kino-Studios/CINE-CAD` repository; `python3 tools/download_assets.py` copies them in
  for anyone with access:
  - `restricted_motors.obj`, `restricted_prop_ccw.obj`, `restricted_prop_cw.obj`,
    `restricted_stack.obj`: GrabCAD community models of a 2207 motor, a 5-inch tri-blade
    prop and the SpeedyBee F405 stack
    (https://grabcad.com/library/speedybee-f405-mini-stack-1; the motor and prop pages
    were not recorded). GrabCAD's terms allow private, non-commercial use only
    (https://help.grabcad.com/article/246-how-can-models-be-used-and-shared).
  - `restricted_guards.obj`: motor guards by Thingiverse user Uno3000
    (https://www.thingiverse.com/thing:6956818), licensed CC BY-NC-SA.
  - Without these files Gazebo loads the model with those visuals missing; physics is
    unaffected.

## Ground (`model://runway` in the world)

- Included unmodified from the same `ardupilot_gazebo` checkout (LGPL-3.0), as
  `worlds/iris_runway.sdf` there does.
