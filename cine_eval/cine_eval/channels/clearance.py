"""Clearance channel: how close the drone came to obstacles, the subject and the ground."""

import numpy as np

from cine_eval.channels.result import OK, ChannelResult, unchecked
from cine_eval.record import DRONE_TRUE, SUBJECT_TRUE, FlightRecord, Stream
from cine_eval.scene import Box, Cylinder, distance_to_obstacle, distance_to_vertical_cylinder
from cine_eval.settings import ScoringSettings
from cine_eval.timeline import SampleGap, fixed_clock, largest_gap, sample

NAME = 'clearance'


def score_clearance(
    record: FlightRecord,
    settings: ScoringSettings,
    scene: list[Box | Cylinder],
    world_frame: str,
    world_label: str = '',
    skipped: list[str] | None = None,
) -> ChannelResult:
    """Surface distance minus the airframe size, over the whole flight (not only filming).

    ``collision_free`` is 1.0 only when static obstacles, the subject and the ground were
    each checked over the whole flight; 0.0 when any contact was seen; otherwise omitted,
    with the reason saying which contact class went unchecked.
    """

    if not record.has(DRONE_TRUE):
        return unchecked(NAME, f'missing streams: {DRONE_TRUE}')

    drone = record.streams[DRONE_TRUE]
    if drone.frame_id != world_frame:
        return unchecked(
            NAME,
            f'{DRONE_TRUE} is in frame {drone.frame_id!r} but the world is in frame '
            f'{world_frame!r}: positions cannot be compared',
        )

    clock = fixed_clock(drone.time[0], drone.time[-1], settings.clock_hz)
    position = sample(drone, 'position', clock, settings.max_pose_gap_s)
    notes: list[str] = []
    unchecked_classes: list[str] = []
    candidates: list[tuple[float, str]] = []
    summary: dict[str, float] = {}
    series: dict[str, np.ndarray] = {'time': clock}

    drone_gap = largest_gap(drone.time, clock[0], clock[-1])
    if drone_gap.length_s > settings.max_pose_gap_s:
        unchecked_classes.append(
            f'{DRONE_TRUE} has a dropout ({_gap_text(DRONE_TRUE, drone_gap, settings)})',
        )

    static_clearance, static_names = _static_clearance(position, scene, settings)
    if static_clearance is None:
        notes.append('no static obstacles in the world file')
        unchecked_classes.append('static obstacles not checked (none in the world file)')
    else:
        series['static_clearance_m'] = np.fmin.reduce(static_clearance, axis=1)
        summary['min_static_clearance_m'] = float(np.nanmin(static_clearance))
        flat_index = int(np.nanargmin(static_clearance))
        candidates.append(
            (summary['min_static_clearance_m'], static_names[flat_index % len(static_names)]),
        )

    subject_frame = record.streams[SUBJECT_TRUE].frame_id if record.has(SUBJECT_TRUE) else None
    if subject_frame is None:
        notes.append('subject_true absent: subject clearance skipped')
        unchecked_classes.append(f'subject not checked ({SUBJECT_TRUE} absent)')
    elif subject_frame != world_frame:
        notes.append(
            f'{SUBJECT_TRUE} is in frame {subject_frame!r}, not {world_frame!r}: '
            'subject clearance skipped',
        )
        unchecked_classes.append(f'subject not checked ({SUBJECT_TRUE} in another frame)')
    else:
        subject = record.streams[SUBJECT_TRUE]
        subject_clearance = _subject_clearance(subject, clock, position, settings)
        series['subject_clearance_m'] = subject_clearance
        summary['subject_covered_share'] = float(np.isfinite(subject_clearance).mean())
        subject_gap = largest_gap(subject.time, clock[0], clock[-1])
        if subject_gap.length_s > settings.max_pose_gap_s:
            unchecked_classes.append(
                'subject not checked over the whole flight '
                f'({_gap_text(SUBJECT_TRUE, subject_gap, settings)})',
            )

        if np.isfinite(subject_clearance).any():
            summary['min_subject_clearance_m'] = float(np.nanmin(subject_clearance))
            candidates.append((summary['min_subject_clearance_m'], 'subject'))
        else:
            notes.append('subject_true does not overlap the flight in time')

    ground_clearance = _ground_clearance(position, settings)
    if np.isfinite(ground_clearance).any():
        series['ground_clearance_m'] = ground_clearance
        summary['min_ground_clearance_m'] = float(np.nanmin(ground_clearance))
        candidates.append((summary['min_ground_clearance_m'], 'ground'))
    else:
        notes.append('never airborne: ground clearance omitted')
        unchecked_classes.append('ground not checked (never airborne)')

    if not candidates:
        return unchecked(NAME, '; '.join(notes))

    stacked = np.stack([values for key, values in series.items() if key != 'time'], axis=1)
    overall = np.fmin.reduce(stacked, axis=1)
    series['clearance_m'] = overall
    in_contact = overall < 0
    summary['min_clearance_m'] = float(np.nanmin(overall))
    summary['contact_frames'] = float(in_contact.sum())

    if in_contact.any():
        summary['collision_free'] = 0.0
        summary['first_contact_s'] = float(clock[np.argmax(in_contact)])
    elif not unchecked_classes:
        summary['collision_free'] = 1.0

    nearest_clearance, nearest_name = min(candidates)
    reason_parts = [f'nearest {nearest_name} at {nearest_clearance:.2f} m']

    if world_label:
        reason_parts.append(f'world {world_label}')

    if skipped:
        reason_parts.append(f'skipped {", ".join(skipped)}')

    if 'collision_free' not in summary:
        reason_parts.append('collision_free unknown: ' + ', '.join(unchecked_classes))

    reason_parts.extend(notes)

    return ChannelResult(
        name=NAME,
        status=OK,
        reason='; '.join(reason_parts),
        summary=summary,
        series=series,
        clock_hz=settings.clock_hz,
    )


def _gap_text(stream_name: str, gap: SampleGap, settings: ScoringSettings) -> str:
    return (
        f'largest {stream_name} gap {gap.length_s:.2f} s from {gap.start_s:.2f} s, '
        f'above max_pose_gap_s {settings.max_pose_gap_s:.2f} s'
    )


def _static_clearance(
    position: np.ndarray,
    scene: list[Box | Cylinder],
    settings: ScoringSettings,
) -> tuple[np.ndarray | None, list[str]]:
    if not scene:
        return None, []

    columns = [distance_to_obstacle(position, obstacle) for obstacle in scene]

    return np.stack(columns, axis=1) - settings.airframe_radius_m, [
        obstacle.name for obstacle in scene
    ]


def _subject_clearance(
    subject: Stream,
    clock: np.ndarray,
    position: np.ndarray,
    settings: ScoringSettings,
) -> np.ndarray:
    subject_position = sample(subject, 'position', clock, settings.max_pose_gap_s)
    centres = np.stack(
        [
            subject_position[:, 0],
            subject_position[:, 1],
            np.full(len(clock), settings.subject_height_m / 2.0),
        ],
        axis=1,
    )
    distance = distance_to_vertical_cylinder(
        position,
        centres,
        settings.subject_radius_m,
        settings.subject_height_m,
    )

    return distance - settings.airframe_radius_m


def _ground_clearance(position: np.ndarray, settings: ScoringSettings) -> np.ndarray:
    """Height above ground inside the window from first to last airborne sample, else NaN."""

    clearance = np.full(len(position), np.nan)
    airborne = np.flatnonzero(position[:, 2] > settings.airborne_height_m)
    if len(airborne):
        window = slice(airborne[0], airborne[-1] + 1)
        clearance[window] = position[window, 2] - settings.airframe_half_height_m

    return clearance
