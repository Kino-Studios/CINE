"""Write each body of an IGES file as its own binary STL, plus a bodies.json index.

Run with a Python that has the OpenCascade bindings (`pip install cadquery-ocp`):

    python3 split_iges_bodies.py <frame.iges> <output_directory>

Bodies are numbered in file order (body_000.stl, body_001.stl, ...); build_meshes.py picks
the frame's plates, standoffs and screws out by those numbers. bodies.json gives each
body's lower corner and size in millimetres, which is how the numbers were identified.
"""

import json
import sys
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.IGESControl import IGESControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer

DEFLECTION_MM = 0.05


def _sub_shapes(shape, kind):
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


def read_iges(path):
    reader = IGESControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise SystemExit(f'cannot read {path}')

    reader.TransferRoots()

    return reader.OneShape()


def describe(body, file_name):
    bounds = Bnd_Box()
    BRepBndLib.Add_s(body, bounds)
    lower, upper = bounds.CornerMin(), bounds.CornerMax()
    lower_corner = (lower.X(), lower.Y(), lower.Z())
    size = (upper.X() - lower.X(), upper.Y() - lower.Y(), upper.Z() - lower.Z())

    return {
        'file': file_name,
        'min': [round(value, 2) for value in lower_corner],
        'size': [round(value, 2) for value in size],
    }


def main():
    iges_path, output_directory = sys.argv[1], Path(sys.argv[2])
    output_directory.mkdir(parents=True, exist_ok=True)
    shape = read_iges(iges_path)
    has_solids = next(_sub_shapes(shape, TopAbs_SOLID), None) is not None
    rows = []

    for index, body in enumerate(_sub_shapes(shape, TopAbs_SOLID if has_solids else TopAbs_SHELL)):
        file_name = f'body_{index:03d}.stl'
        # Measured before meshing: once a body has triangles, its bounds come from those instead.
        rows.append(describe(body, file_name))
        BRepMesh_IncrementalMesh(body, DEFLECTION_MM, False, 0.3, True).Perform()
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        writer.Write(body, str(output_directory / file_name))

    (output_directory / 'bodies.json').write_text(json.dumps(rows, indent=1))
    print(f'{len(rows)} bodies written to {output_directory}')


if __name__ == '__main__':
    main()
