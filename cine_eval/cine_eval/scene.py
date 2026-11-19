"""Static obstacles read from the Gazebo world file, and signed distances to them."""

import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import cine_eval
from cine_eval.record import FlightRecord

STATIC_FLAGS = ('true', '1')
DEFAULT_WORLD = Path('cine_gazebo/worlds/cine_field.sdf.template')


@dataclass(frozen=True)
class Box:
    name: str
    centre: np.ndarray
    size: np.ndarray
    yaw: float


@dataclass(frozen=True)
class Cylinder:
    name: str
    centre: np.ndarray
    radius: float
    length: float


def repo_root() -> Path:
    return Path(cine_eval.__file__).resolve().parents[2]


def resolve_world_path(record: FlightRecord) -> Path:
    """The world file named in run.json, else the committed template."""

    world_file = record.metadata.get('run', {}).get('world_file')
    if world_file is None:
        return repo_root() / DEFAULT_WORLD

    path = Path(world_file)

    return path if path.is_absolute() else repo_root() / path


@dataclass(frozen=True)
class World:
    """What the scorer reads from a world file."""

    obstacles: list[Box | Cylinder]
    skipped: list[str]
    frame: str


class WorldError(ValueError):
    """The world file is missing or cannot be read as a Gazebo world."""


def load_world(world_path: Path) -> World:
    """Static collision shapes, the models left out, and the frame the poses are in.

    Gazebo names the world frame after the world, so ``<world name>`` is that frame.
    """

    world_path = Path(world_path)

    try:
        root = ElementTree.parse(world_path).getroot()
    except OSError as exception:
        raise WorldError(f'cannot read world file {world_path}: {exception}') from None
    except ElementTree.ParseError as exception:
        raise WorldError(f'world file {world_path} is not valid XML: {exception}') from None

    world = root.find('world')
    if world is None:
        raise WorldError(f'world file {world_path} has no <world> element')

    obstacles, skipped = _read_world(world, world_path)

    return World(obstacles=obstacles, skipped=skipped, frame=world.get('name', ''))


def load_scene(world_path: Path) -> list[Box | Cylinder]:
    """Collision boxes and cylinders of the static models in the world file."""

    return load_world(world_path).obstacles


def skipped_models(world_path: Path) -> list[str]:
    """Included models and unsupported collision shapes, which ``load_scene`` leaves out."""

    return load_world(world_path).skipped


def _read_world(
    world: ElementTree.Element,
    world_path: Path,
) -> tuple[list[Box | Cylinder], list[str]]:
    obstacles: list[Box | Cylinder] = []
    skipped: list[str] = []

    for include in world.findall('include'):
        uri = include.findtext('uri', default='?').strip()
        skipped.append(f'{uri.rsplit("/", 1)[-1]} (include)')

    for model in world.findall('model'):
        if (model.findtext('static') or '').strip().lower() not in STATIC_FLAGS:
            skipped.append(f'{model.get("name")} (not static)')

            continue

        collisions = model.findall('link/collision')
        for collision in collisions:
            name = model.get('name')

            if len(collisions) > 1:
                name = f'{name}/{collision.get("name")}'

            geometry = collision.find('geometry')

            try:
                obstacle = _obstacle(name, _read_pose(model.find('pose')), geometry)
            except (AttributeError, IndexError, TypeError, ValueError) as exception:
                raise WorldError(
                    f'world file {world_path}: model {name} cannot be read: {exception}',
                ) from None

            if obstacle is None:
                shape = next(iter(geometry), None) if geometry is not None else None
                skipped.append(f'{name} ({shape.tag if shape is not None else "no geometry"})')
            else:
                obstacles.append(obstacle)

    return obstacles, skipped


def _read_pose(pose: ElementTree.Element | None) -> tuple[np.ndarray, float]:
    if pose is None or not (pose.text or '').strip():
        return np.zeros(3), 0.0

    values = [float(token) for token in pose.text.split()]
    if len(values) != 6:
        raise ValueError(f'pose has {len(values)} values, expected 6')

    yaw = values[5]

    if pose.get('degrees', 'false').lower() == 'true':
        yaw = float(np.radians(yaw))

    return np.array(values[:3]), yaw


def _obstacle(
    name: str,
    pose: tuple[np.ndarray, float],
    geometry: ElementTree.Element | None,
) -> Box | Cylinder | None:
    if geometry is None:
        return None

    centre, yaw = pose
    box = geometry.find('box')
    if box is not None:
        size = np.array([float(token) for token in (box.findtext('size') or '').split()])
        if size.shape != (3,):
            raise ValueError(f'box size has {size.size} values, expected 3')

        return Box(name=name, centre=centre, size=size, yaw=yaw)

    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        return Cylinder(
            name=name,
            centre=centre,
            radius=float(cylinder.findtext('radius')),
            length=float(cylinder.findtext('length')),
        )

    return None


def distance_to_vertical_cylinder(
    points: np.ndarray,
    centres: np.ndarray,
    radius: float,
    length: float,
) -> np.ndarray:
    """Signed distance to an upright finite cylinder (``centres``: one centre or one per point)."""

    offset = np.asarray(points, dtype=float) - centres
    radial = np.linalg.norm(offset[..., :2], axis=-1)
    excess = np.stack([radial - radius, np.abs(offset[..., 2]) - length / 2.0], axis=-1)
    outside = np.linalg.norm(np.maximum(excess, 0.0), axis=-1)
    inside = np.minimum(np.max(excess, axis=-1), 0.0)

    return outside + inside


def distance_to_obstacle(points: np.ndarray, obstacle: Box | Cylinder) -> np.ndarray:
    """Signed distance from each point to the obstacle's surface; negative inside."""

    points = np.asarray(points, dtype=float)

    if isinstance(obstacle, Cylinder):
        return distance_to_vertical_cylinder(
            points,
            obstacle.centre,
            obstacle.radius,
            obstacle.length,
        )

    offset = points - obstacle.centre
    cosine, sine = np.cos(-obstacle.yaw), np.sin(-obstacle.yaw)
    local = np.stack(
        [
            cosine * offset[:, 0] - sine * offset[:, 1],
            sine * offset[:, 0] + cosine * offset[:, 1],
            offset[:, 2],
        ],
        axis=1,
    )
    excess = np.abs(local) - obstacle.size / 2.0
    outside = np.linalg.norm(np.maximum(excess, 0.0), axis=1)
    inside = np.minimum(np.max(excess, axis=1), 0.0)

    return outside + inside
