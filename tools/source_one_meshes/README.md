# Source One meshes

These scripts rebuild the visual meshes in
`cine_gazebo/models/cine_source_one/meshes/` from the original CAD. Collision shapes,
mass and rotor poses live in `model.sdf.template` and are not touched.

The CAD lives in the private CINE-CAD repository, under `sources/`, with each file's
licence beside it. The owner's 3D-printed parts come from the model's own `cad/` folder.

```bash
OCP_PYTHON=/path/to/venv/bin/python BLENDER=blender \
  tools/source_one_meshes/build.sh ../CINE-CAD /tmp/source_one_parts
```

`OCP_PYTHON` needs the OpenCascade bindings (`pip install cadquery-ocp`) and `BLENDER`
must be Blender 5. The build takes a few minutes, most of it splitting the Pi 5 STEP.

| Script | Runs in | Does |
| :-- | :-- | :-- |
| `split_iges_bodies.py` | OCP Python | writes each body of the TBS Source One V5 IGES as `body_NNN.stl` |
| `split_step_by_colour.py` | OCP Python | writes one STL per surface colour of a STEP, plus `colors.json` |
| `parts.py` | Blender | printed parts, standoffs, spacers and battery leads, which no vendor CAD covers |
| `build_meshes.py` | Blender | places everything, decimates each family to its budget, writes OBJ and MTL |

The `restricted_*` meshes (GrabCAD motor, prop and stack; CC BY-NC-SA guards) are written
too but are git-ignored; `tools/download_assets.py` fetches the published copies.
