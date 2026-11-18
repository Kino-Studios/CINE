"""Assemble the Source One from vendor CAD and write its Gazebo meshes.

Run with Blender 5 after the split scripts have filled the parts folder (build.sh does both):

    blender -b -P build_meshes.py -- <parts_directory> <guards_directory> <mesh_directory>

The parts folder holds the TBS Source One V5 frame split into bodies (tbs_v5/) and each
vendor STEP split by colour (motor/, prop/, stack/, pi5/, camera/). The guards folder is
CINE-CAD's sources/guards. Lengths are millimetres in base_link's frame until export,
which scales to metres.

One OBJ is written per family. carbon, airframe, pi5 and camera are freely licensed and
committed; the restricted_* families (GrabCAD motor, prop and stack, CC BY-NC-SA guards)
are git-ignored and fetched by tools/download_assets.py.
"""

import json
import math
import sys
from pathlib import Path

import bpy
from mathutils import Matrix

sys.path.insert(0, str(Path(__file__).resolve().parent))

import parts  # noqa: E402  (Blender does not put the script's folder on the import path)

# The printed Pi tray is turned 180 degrees so its 0.75 in hole square lands on the
# frame's 20x20 pad at (-55.14, 0).
TRAY_FRONT, TRAY_SIDE, TRAY_BOTTOM, TRAY_FLOOR = 27.31, 31.75, 41.0, 1.0
PI_BOARD_BOTTOM = TRAY_BOTTOM + TRAY_FLOOR + 6.0
PI_ORIGIN = (TRAY_FRONT - 4.82, TRAY_SIDE - 6.93)
PLATE_UNDERSIDE = 68.0
HEX_RADIUS = 5.0 / math.sqrt(3)
CAMERA_MOUNT_FLIP = (
    Matrix.Translation((0, 0, 18.0))
    @ Matrix.Rotation(math.pi, 4, 'X')
    @ Matrix.Translation((0, 0, -18.0))
)

MOTOR_X, MOTOR_Y = 72.65, 86.5
IGES_TO_MODEL = (
    Matrix.Translation((0, 0, -3.0))
    @ Matrix.Rotation(math.radians(90), 4, 'Z')
    @ Matrix.Translation((-1861.18, -240.52, 0))
)
# The real drone's top plate sits this much higher than the V5 design, on longer standoffs.
TOP_PLATE_LIFT = 41.0
PROP_SCALE = 129.5 / 120.0

PALETTE = {
    'pla_blue': (0.06, 0.28, 0.65, 0, 0.55),
    'black_plastic': (0.02, 0.02, 0.02, 0, 0.6),
    'black_metal': (0.02, 0.02, 0.025, 0.8, 0.35),
    'silver': (0.8, 0.8, 0.82, 1, 0.25),
    'red_wire': (0.6, 0.02, 0.02, 0, 0.5),
    'yellow': (0.9, 0.7, 0.05, 0, 0.5),
    'white_plastic': (0.85, 0.85, 0.8, 0, 0.6),
    'brass': (0.78, 0.6, 0.25, 1, 0.3),
}
TRIANGLE_BUDGETS = {
    'pi5': 40000,
    'camera': 20000,
    'restricted_motors': 60000,
    'restricted_guards': 40000,
    'restricted_stack': 40000,
    'restricted_prop_ccw': 12000,
    'restricted_prop_cw': 12000,
}
UNTEXTURED_WITHOUT_MATERIALS = ('carbon', 'restricted_prop_ccw', 'restricted_prop_cw')


def material(name, rgb, metallic=0.0, roughness=0.5):
    new_material = bpy.data.materials.new(name)
    new_material.use_nodes = True
    shader = new_material.node_tree.nodes['Principled BSDF']
    shader.inputs['Base Color'].default_value = (*rgb, 1)
    shader.inputs['Metallic'].default_value = metallic
    shader.inputs['Roughness'].default_value = roughness
    new_material.diffuse_color = (*rgb, 1.0)

    return new_material


def looks_like_grey_metal(rgb):
    return abs(rgb[0] - rgb[1]) < 0.03 and 0.2 < rgb[0] < 0.95


def import_stl(path, transform):
    bpy.ops.wm.stl_import(filepath=str(path))
    obj = bpy.context.selected_objects[0]
    obj.data.transform(transform)

    return obj


def build_printed_parts():
    parts.build_battery()
    adapter_plate = parts.import_stl(
        parts.CAD_DIRECTORY / 'rasp_pi_adapter_plate.stl',
        180,
        (TRAY_FRONT, TRAY_SIDE, TRAY_BOTTOM),
    )
    parts.add_to_group('pla_blue', adapter_plate)

    for post_x in (-45.14, -65.14):
        for post_y in (-10.0, 10.0):
            post = parts.cylinder('tray_post', post_x, post_y, 6.0, TRAY_BOTTOM, HEX_RADIUS, 6)
            parts.add_to_group('black_metal', post)

    for hole_x in (3.5, 61.5):
        for hole_y in (3.5, 52.5):
            standoff = parts.cylinder(
                'pi_standoff',
                PI_ORIGIN[0] - hole_x,
                PI_ORIGIN[1] - hole_y,
                TRAY_BOTTOM + TRAY_FLOOR,
                PI_BOARD_BOTTOM,
                HEX_RADIUS,
                6,
            )
            parts.add_to_group('brass', standoff)

    # Under the top plate's middle screws, each resting on the Pi part measured beneath it.
    for spacer_y, resting_height in ((9.5, 5.29), (-9.5, 2.54)):
        spacer = parts.cylinder(
            'nylon_spacer',
            -32.9,
            spacer_y,
            PI_BOARD_BOTTOM + resting_height,
            PLATE_UNDERSIDE,
            2.5,
        )
        parts.add_to_group('white_plastic', spacer)

    camera_mount = parts.import_stl(
        parts.CAD_DIRECTORY / 'rasp_pi_camera_mount_drone.stl',
        -90,
        (78.72, -14.43, 33.0),
    )
    camera_mount.data.transform(CAMERA_MOUNT_FLIP)
    parts.add_to_group('pla_blue', camera_mount)
    parts.build_battery_leads()

    for group, objects in parts.material_groups.items():
        red, green, blue, metallic, roughness = PALETTE[group]
        group_material = material(group, (red, green, blue), metallic, roughness)

        for obj in objects:
            obj.data.materials.clear()
            obj.data.materials.append(group_material)


def build_frame(parts_directory):
    carbon = material('tbs_carbon', (0.03, 0.03, 0.035), 0.1, 0.35)
    steel = material('tbs_steel', (0.55, 0.55, 0.58), 0.9, 0.3)
    aluminium = material('tbs_alu', (0.08, 0.08, 0.09), 0.8, 0.35)

    def place_body(index, body_material, extra=None):
        body_path = parts_directory / 'tbs_v5' / f'body_{index:03d}.stl'
        body = import_stl(body_path, IGES_TO_MODEL if extra is None else extra @ IGES_TO_MODEL)
        body.data.materials.append(body_material)

    def stretch_upwards(base, old_top, new_top):
        factor = (new_top - base) / (old_top - base)

        return (
            Matrix.Translation((0, 0, base))
            @ Matrix.Diagonal((1, 1, factor, 1))
            @ Matrix.Translation((0, 0, -base))
        )

    lift = Matrix.Translation((0, 0, TOP_PLATE_LIFT))

    for index in (52, 53, 54, 55, 56, 2, 4):
        place_body(index, carbon)

    place_body(3, carbon, lift)

    for index in (7, 9, 11, 13):
        place_body(index, aluminium, stretch_upwards(-3.0, 27.0, 27.0 + TOP_PLATE_LIFT))

    for index in (12, 14):
        place_body(index, aluminium, stretch_upwards(5.0, 27.0, 27.0 + TOP_PLATE_LIFT))

    lower_screws = (19, 24, 25, 36, 28, 29, 39, 40, 30, 31, 41, 42, 26, 27, 37, 38)
    for index in (*lower_screws, *range(44, 52), *range(15, 19)):
        place_body(index, steel)

    for index in (21, 22, 32, 33, 34, 43):
        place_body(index, steel, lift)

    for index in (23, 35):
        place_body(index, steel, Matrix.Translation((0, 0, TOP_PLATE_LIFT - 2.0)))

    place_body(0, material('pad', (0.05, 0.05, 0.05), 0, 0.9))


def build_guards(guards_directory):
    guard_material = material('guard_pla', (0.8, 0.12, 0.05), 0, 0.5)
    guards = []

    for sign_x, sign_y in parts.MOTOR_SIGNS:
        x, y = sign_x * MOTOR_X, sign_y * MOTOR_Y
        end = 'front' if sign_x > 0 else 'back'
        side = 'left' if sign_y > 0 else 'right'
        inward = math.atan2(-y, -x)
        flip = Matrix.Rotation(math.pi, 4, 'Y') if side == 'right' else Matrix.Identity(4)
        transform = (
            Matrix.Translation((x, y, -3.0))
            @ Matrix.Rotation(inward - math.pi / 2, 4, 'Z')
            @ flip
            @ Matrix.Translation((-34.66, -145.35, 0))
        )
        guard = import_stl(guards_directory / f'motor_guard_{end}_{side}.stl', transform)
        guard.data.materials.append(guard_material)
        guards.append(guard)

    return guards


def place_coloured_part(parts_directory, folder, transform):
    """Import every colour of one split STEP part, each with a material of that colour."""

    rows = json.loads((parts_directory / folder / 'colors.json').read_text())
    objects = []

    for row in rows:
        obj = import_stl(parts_directory / folder / row['file'], transform)
        rgb = tuple(row['rgb'])
        metallic = 0.5 if looks_like_grey_metal(rgb) else 0.0
        obj.data.materials.append(material(f'{folder}_{row["file"]}', rgb, metallic, 0.4))
        objects.append(obj)

    return objects


def build_vendor_parts(parts_directory):
    families = {'restricted_motors': [], 'restricted_stack': [], 'pi5': [], 'camera': []}

    for sign_x, sign_y in parts.MOTOR_SIGNS:
        motor_transform = Matrix.Translation((sign_x * MOTOR_X, sign_y * MOTOR_Y, 8.0))
        families['restricted_motors'] += place_coloured_part(
            parts_directory,
            'motor',
            motor_transform,
        )

    families['restricted_stack'] = place_coloured_part(
        parts_directory,
        'stack',
        Matrix.Translation((0, 0, 5.0)),
    )
    pi_transform = Matrix.Translation((*PI_ORIGIN, PI_BOARD_BOTTOM)) @ Matrix.Rotation(
        math.pi,
        4,
        'Z',
    )
    families['pi5'] = place_coloured_part(parts_directory, 'pi5', pi_transform)
    camera_transform = CAMERA_MOUNT_FLIP @ Matrix(
        ((0, 0, -1, 77.98), (0, -1, 0, 12.5), (-1, 0, 0, 29.63), (0, 0, 0, 1)),
    )
    families['camera'] = place_coloured_part(parts_directory, 'camera', camera_transform)

    return families


def build_props(parts_directory):
    """One prop of each spin direction at the origin; model.sdf.template places them."""

    props = {}

    for family, mirrored in (('restricted_prop_ccw', False), ('restricted_prop_cw', True)):
        mirror = Matrix.Scale(-1, 4, (1, 0, 0)) if mirrored else Matrix.Identity(4)
        transform = Matrix.Scale(PROP_SCALE, 4) @ mirror @ Matrix.Rotation(math.radians(90), 4, 'X')
        prop = import_stl(parts_directory / 'prop' / 'color_00.stl', transform)

        if mirrored:
            prop.data.flip_normals()

        props[family] = [prop]

    return props


def triangle_count(obj):
    obj.data.calc_loop_triangles()

    return len(obj.data.loop_triangles)


def decimate(obj, ratio):
    if ratio < 1 and triangle_count(obj) > 300:
        modifier = obj.modifiers.new('budget', 'DECIMATE')
        modifier.ratio = ratio
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=modifier.name)


def export_family(name, objects, mesh_directory):
    bpy.ops.object.select_all(action='DESELECT')
    total = sum(triangle_count(obj) for obj in objects)
    ratio = min(1.0, TRIANGLE_BUDGETS.get(name, total) / total)

    for obj in objects:
        decimate(obj, ratio)
        obj.select_set(True)

    bpy.context.view_layer.objects.active = objects[0]

    if len(objects) > 1:
        bpy.ops.object.join()

    joined = bpy.context.view_layer.objects.active
    joined.name = name

    if name == 'carbon':
        parts.assign_planar_uvs(joined)

    bpy.ops.wm.obj_export(
        filepath=str(mesh_directory / f'{name}.obj'),
        export_selected_objects=True,
        global_scale=0.001,
        forward_axis='Y',
        up_axis='Z',
        export_materials=name not in UNTEXTURED_WITHOUT_MATERIALS,
        export_pbr_extensions=False,
        export_uv=name == 'carbon',
        export_normals=True,
        export_triangulated_mesh=True,
        path_mode='STRIP',
    )
    print(f'{name}: {triangle_count(joined)} triangles')


def _to_srgb(linear):
    return 12.92 * linear if linear <= 0.0031308 else 1.055 * linear ** (1 / 2.4) - 0.055


def rewrite_mtl_colours(mtl_path):
    """Make each material's display and ambient colour the sRGB of its diffuse colour.

    Blender writes linear Kd and a white Ka, while Gazebo shows Kd as an sRGB colour and
    lights Ka as ambient, so unconverted parts render washed out and glow in shadow.
    """

    rewritten = []

    for line in mtl_path.read_text().splitlines():
        if line.startswith('Ka '):
            continue

        if line.startswith('Kd '):
            srgb = ' '.join(f'{_to_srgb(float(value)):.6f}' for value in line.split()[1:4])
            line = f'Ka {srgb}\nKd {srgb}'
        elif line.startswith('Ks '):
            line = 'Ks 0.300000 0.300000 0.300000'

        rewritten.append(line)

    mtl_path.write_text('\n'.join(rewritten) + '\n')


def main():
    arguments = sys.argv[sys.argv.index('--') + 1 :]
    parts_directory, guards_directory = Path(arguments[0]), Path(arguments[1])
    mesh_directory = Path(arguments[2])
    mesh_directory.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)

    build_printed_parts()
    build_frame(parts_directory)
    families = {'restricted_guards': build_guards(guards_directory)}
    families |= build_vendor_parts(parts_directory)
    assigned = {obj.name for objects in families.values() for obj in objects}

    for obj in bpy.context.scene.objects:
        if obj.type == 'MESH' and obj.name not in assigned:
            is_carbon = obj.data.materials[0].name.startswith('tbs_carbon')
            families.setdefault('carbon' if is_carbon else 'airframe', []).append(obj)

    families |= build_props(parts_directory)
    for name, objects in families.items():
        export_family(name, objects, mesh_directory)

    for mtl_path in mesh_directory.glob('*.mtl'):
        rewrite_mtl_colours(mtl_path)


main()
