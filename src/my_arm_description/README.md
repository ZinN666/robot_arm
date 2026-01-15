# My Arm Description Package

This package contains URDF description files and meshes for the my_arm robot.

## Structure

- `my_arm/urdf/` - URDF xacro files
- `my_arm/meshes/` - 3D mesh files (visual and collision)
- `my_arm/rviz/` - RViz configuration files

## Note

Currently, the mesh files are placeholders. You need to add your own mesh files:
- `meshes/visual/link_X.dae` - Visual meshes
- `meshes/collision/link_X.stl` - Collision meshes

Where X is the link number (0-6).

