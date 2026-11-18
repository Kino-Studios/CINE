"""Write one binary STL per surface colour of a STEP file, plus a colors.json index.

Run with a Python that has the OpenCascade bindings (`pip install cadquery-ocp`):

    python3 split_step_by_colour.py <part.step> <output_directory> <deflection_mm>

colors.json lists each STL with its RGB colour (linear, 0 to 1) and face count, largest
first. Faces with no colour anywhere in their assembly path come out mid grey.
"""

import json
import sys
from pathlib import Path

from OCP.BRep import BRep_Builder
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.collections import Sequence_TDF_Label
from OCP.IFSelect import IFSelect_RetDone
from OCP.Quantity import Quantity_Color
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS_Compound
from OCP.XCAFDoc import (
    XCAFDoc_ColorGen,
    XCAFDoc_ColorSurf,
    XCAFDoc_ColorTool,
    XCAFDoc_DocumentTool,
)

UNCOLOURED = (0.6, 0.6, 0.6)


def _rounded_rgb(color):
    return (round(color.Red(), 3), round(color.Green(), 3), round(color.Blue(), 3))


def _faces(shape):
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


class ColourSplitter:
    """Walks a STEP document's assembly tree and sorts every face by its colour."""

    def __init__(self, document):
        self.shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
        self.color_tool = XCAFDoc_DocumentTool.ColorTool_s(document.Main())
        self.faces_by_colour: dict[tuple, list] = {}

    def label_colour(self, label):
        color = Quantity_Color()

        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if XCAFDoc_ColorTool.GetColor_s(label, kind, color):
                return _rounded_rgb(color)

        return None

    def shape_colour(self, shape):
        color = Quantity_Color()

        for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen):
            if self.color_tool.GetColor(shape, kind, color):
                return _rounded_rgb(color)

        return None

    def walk(self, label, location, inherited_colour):
        colour = self.label_colour(label) or inherited_colour

        if self.shape_tool.IsReference_s(label):
            referred = TDF_Label()
            self.shape_tool.GetReferredShape_s(label, referred)
            self.walk(referred, location * self.shape_tool.GetLocation_s(label), colour)
        elif self.shape_tool.IsAssembly_s(label):
            components = Sequence_TDF_Label()
            self.shape_tool.GetComponents_s(label, components)

            for index in range(1, components.Length() + 1):
                self.walk(components.Value(index), location, colour)
        else:
            self._sort_faces(label, location, colour)

    def _sort_faces(self, label, location, part_colour):
        # A colour set on a sub-shape (one face or one solid of the part) beats the part's own.
        sub_shape_colours = {}
        sub_shapes = Sequence_TDF_Label()
        self.shape_tool.GetSubShapes_s(label, sub_shapes)

        for index in range(1, sub_shapes.Length() + 1):
            sub_label = sub_shapes.Value(index)
            sub_colour = self.label_colour(sub_label)
            if sub_colour:
                sub_shape = self.shape_tool.GetShape_s(sub_label).Moved(location)
                for face in _faces(sub_shape):
                    sub_shape_colours[hash(face)] = sub_colour

        shape = self.shape_tool.GetShape_s(label).Moved(location)
        for face in _faces(shape):
            colour = (
                sub_shape_colours.get(hash(face))
                or self.shape_colour(face)
                or part_colour
                or UNCOLOURED
            )
            self.faces_by_colour.setdefault(colour, []).append(face)


def read_step(path):
    document = TDocStd_Document(TCollection_ExtendedString('step'))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)

    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise SystemExit(f'cannot read {path}')

    reader.Transfer(document)

    return document


def write_colour_stls(faces_by_colour, output_directory, deflection):
    rows = []
    builder = BRep_Builder()
    by_face_count = sorted(faces_by_colour.items(), key=lambda item: -len(item[1]))
    for number, (colour, faces) in enumerate(by_face_count):
        compound = TopoDS_Compound()
        builder.MakeCompound(compound)

        for face in faces:
            builder.Add(compound, face)

        BRepMesh_IncrementalMesh(compound, deflection, False, 0.3, True).Perform()
        stl_path = output_directory / f'color_{number:02d}.stl'
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        writer.Write(compound, str(stl_path))
        rows.append({'file': stl_path.name, 'rgb': colour, 'faces': len(faces)})

    return rows


def main():
    step_path, output_directory, deflection = sys.argv[1], Path(sys.argv[2]), float(sys.argv[3])
    output_directory.mkdir(parents=True, exist_ok=True)
    document = read_step(step_path)
    splitter = ColourSplitter(document)
    free_shapes = Sequence_TDF_Label()
    splitter.shape_tool.GetFreeShapes(free_shapes)

    for index in range(1, free_shapes.Length() + 1):
        splitter.walk(free_shapes.Value(index), TopLoc_Location(), None)

    rows = write_colour_stls(splitter.faces_by_colour, output_directory, deflection)
    (output_directory / 'colors.json').write_text(json.dumps(rows, indent=1))

    for row in rows:
        print(row)


if __name__ == '__main__':
    main()
