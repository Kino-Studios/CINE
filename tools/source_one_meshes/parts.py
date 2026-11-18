"""The pieces of the Source One that no vendor CAD covers, built in Blender.

That means the owner's 3D-printed parts (from the model's cad/ folder), and simple solids
for the hex standoffs, nylon spacers, battery leads and XT60 plug. Lengths are millimetres
in base_link's frame: x forward, y left, z up.

Each solid is filed under a material name in `material_groups`; build_meshes.py gives each
group one material from its palette.
"""

import math
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

CAD_DIRECTORY = (
    Path(__file__).resolve().parents[2] / 'cine_gazebo' / 'models' / 'cine_source_one' / 'cad'
)
MOTOR_SIGNS = ((1, -1), (-1, 1), (1, 1), (-1, -1))
CARBON_TILE_MM = 12.0

material_groups: dict[str, list] = {}


def add_to_group(group, obj):
    material_groups.setdefault(group, []).append(obj)

    return obj


def _object_from_bmesh(name, mesh_builder):
    mesh = bpy.data.meshes.new(name)
    mesh_builder.to_mesh(mesh)
    mesh_builder.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)

    return obj


def bevel(obj, width, segments=2, angle_degrees=40):
    modifier = obj.modifiers.new('bevel', 'BEVEL')
    modifier.width = width
    modifier.segments = segments
    modifier.limit_method = 'ANGLE'
    modifier.angle_limit = math.radians(angle_degrees)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=modifier.name)

    return obj


def rounded_rectangle_outline(center_x, center_y, length, width, radius, segments=6):
    outline = []
    half_length, half_width = length / 2 - radius, width / 2 - radius
    corners = ((1, 1), (-1, 1), (-1, -1), (1, -1))
    for corner_index, (sign_x, sign_y) in enumerate(corners):
        start_angle = math.pi / 2 * corner_index

        for index in range(segments + 1):
            angle = start_angle + math.pi / 2 * index / segments
            outline.append(
                (
                    center_x + sign_x * half_length + radius * math.cos(angle),
                    center_y + sign_y * half_width + radius * math.sin(angle),
                ),
            )

    return outline


def circle_outline(center_x, center_y, radius, segments=32):
    return [
        (
            center_x + radius * math.cos(2 * math.pi * index / segments),
            center_y + radius * math.sin(2 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def prism(name, outline, bottom, top):
    mesh_builder = bmesh.new()
    face = mesh_builder.faces.new([mesh_builder.verts.new((x, y, bottom)) for x, y in outline])
    extruded = bmesh.ops.extrude_face_region(mesh_builder, geom=[face])
    top_verts = [item for item in extruded['geom'] if isinstance(item, bmesh.types.BMVert)]
    bmesh.ops.translate(mesh_builder, verts=top_verts, vec=(0, 0, top - bottom))
    bmesh.ops.recalc_face_normals(mesh_builder, faces=mesh_builder.faces)

    return _object_from_bmesh(name, mesh_builder)


def box(name, center, size):
    outline = rounded_rectangle_outline(center[0], center[1], size[0], size[1], 0.01, 1)

    return prism(name, outline, center[2] - size[2] / 2, center[2] + size[2] / 2)


def cylinder(name, center_x, center_y, bottom, top, radius, segments=32):
    return prism(name, circle_outline(center_x, center_y, radius, segments), bottom, top)


def rod(name, start, end, radius, segments=12):
    start, end = Vector(start), Vector(end)
    obj = cylinder(name, 0, 0, 0, (end - start).length, radius, segments)
    rotation = (end - start).to_track_quat('Z', 'Y').to_matrix().to_4x4()
    obj.data.transform(Matrix.Translation(start) @ rotation)

    return obj


def import_stl(path, rotation_z_degrees, translation):
    bpy.ops.wm.stl_import(filepath=str(path))
    obj = bpy.context.selected_objects[0]
    obj.data.transform(
        Matrix.Translation(translation) @ Matrix.Rotation(math.radians(rotation_z_degrees), 4, 'Z'),
    )

    return obj


def build_battery():
    add_to_group('pla_blue', import_stl(CAD_DIRECTORY / 'battery_box.stl', 90, (0.0, 0.0, -10.0)))
    add_to_group(
        'pla_blue',
        import_stl(CAD_DIRECTORY / 'battery_wire_part.stl', 0, (-47.5, -11.86, -12.0)),
    )


def build_battery_leads():
    """The ESC's capacitor, its two power leads, and the XT60 plug they end in."""

    add_to_group('black_plastic', rod('capacitor', (-22, 0, 9.0), (-40, 0, 9.0), 5.0, 24))
    add_to_group('silver', rod('cap_top', (-40, 0, 9.0), (-41, 0, 9.0), 4.6, 24))
    add_to_group('red_wire', rod('lead_red', (-18, 4, 8), (-52, 4, -12), 1.6))
    add_to_group('black_plastic', rod('lead_black', (-18, -4, 8), (-52, -4, -12), 1.6))
    add_to_group('yellow', bevel(box('xt60', (-55, 0, -12), (8.2, 15.5, 16)), 0.8))


def assign_planar_uvs(obj):
    """Project the carbon twill texture flat onto top faces and along the sides elsewhere."""

    mesh_builder = bmesh.new()
    mesh_builder.from_mesh(obj.data)
    uv_layer = mesh_builder.loops.layers.uv.verify()

    for face in mesh_builder.faces:
        faces_up_or_down = abs(face.normal.z) > 0.7

        for loop in face.loops:
            x, y, z = loop.vert.co
            loop[uv_layer].uv = (
                (x / CARBON_TILE_MM, y / CARBON_TILE_MM)
                if faces_up_or_down
                else ((x + y) / CARBON_TILE_MM, z / CARBON_TILE_MM)
            )

    mesh_builder.to_mesh(obj.data)
    mesh_builder.free()
