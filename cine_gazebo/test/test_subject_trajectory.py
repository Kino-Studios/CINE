from pathlib import Path

import pytest
from cine_gazebo.model_rendering import WindProfile, render_world
from cine_gazebo.subject_trajectory import parse_actor_script, subject_pose_at

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CALM = WindProfile(
    linear_velocity=(0.0, 0.0, 0.0),
    horizontal_magnitude_stddev=0.0,
    horizontal_direction_stddev=0.0,
    vertical_stddev=0.0,
)

# The shipped subject's loop, cut down to what each test needs to read.
LOOP_WORLD = """<sdf version="1.9">
  <world name="test_field">
    <actor name="subject">
      <script>
        <loop>true</loop>
        <delay_start>0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walk">
          <waypoint><time>0</time><pose>-3 9 1.0 0 0 0</pose></waypoint>
          <waypoint><time>5</time><pose>3 9 1.0 0 0 0</pose></waypoint>
          <waypoint><time>6</time><pose>3 9 1.0 0 0 1.5708</pose></waypoint>
          <waypoint><time>7.7</time><pose>3 11 1.0 0 0 1.5708</pose></waypoint>
          <waypoint><time>8.7</time><pose>3 11 1.0 0 0 3.1416</pose></waypoint>
          <waypoint><time>13.7</time><pose>-3 11 1.0 0 0 3.1416</pose></waypoint>
          <waypoint><time>14.7</time><pose>-3 11 1.0 0 0 4.7124</pose></waypoint>
          <waypoint><time>16.4</time><pose>-3 9 1.0 0 0 4.7124</pose></waypoint>
          <waypoint><time>17.4</time><pose>-3 9 1.0 0 0 6.2832</pose></waypoint>
        </trajectory>
      </script>
    </actor>
  </world>
</sdf>"""

DELAYED_ONE_WAY_WORLD = """<sdf version="1.9">
  <world name="test_field">
    <actor name="subject">
      <script>
        <loop>false</loop>
        <delay_start>2</delay_start>
        <trajectory id="0" type="walk">
          <waypoint><time>0</time><pose>0 0 1 0 0 0</pose></waypoint>
          <waypoint><time>4</time><pose>4 0 1 0 0 0</pose></waypoint>
        </trajectory>
      </script>
    </actor>
  </world>
</sdf>"""


def test_time_on_a_waypoint_returns_that_waypoint_position():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 5.0).position == pytest.approx((3.0, 9.0, 1.0))


def test_halfway_along_a_straight_leg_returns_its_midpoint():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 2.5).position == pytest.approx((0.0, 9.0, 1.0))


def test_quarter_way_along_a_leg_follows_gazebo_spline_not_a_straight_line():
    script = parse_actor_script(LOOP_WORLD)

    # A straight line would give x = -1.5; Gazebo's Catmull-Rom segment eases in.
    assert subject_pose_at(script, 1.25).position[0] == pytest.approx(-1.78125)


def test_turning_on_the_spot_drifts_past_the_corner_as_gazebo_does():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 5.5).position == pytest.approx((3.375, 8.875, 1.0))


def test_time_past_one_loop_wraps_to_the_same_place_in_the_loop():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 22.4).position == pytest.approx((3.0, 9.0, 1.0))


def test_time_two_loops_in_lands_on_the_leg_midpoint_again():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 37.3).position == pytest.approx((0.0, 9.0, 1.0))


def test_orientation_on_a_waypoint_is_its_yaw():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 6.0).orientation_xyzw == pytest.approx(
        (0.0, 0.0, 0.7071080798, 0.7071054825)
    )


def test_orientation_halfway_through_a_turn_is_half_the_yaw():
    script = parse_actor_script(LOOP_WORLD)

    assert subject_pose_at(script, 5.5).orientation_xyzw == pytest.approx(
        (0.0, 0.0, 0.3826842, 0.9238792), abs=1e-6
    )


def test_before_the_start_delay_the_subject_holds_its_first_waypoint():
    script = parse_actor_script(DELAYED_ONE_WAY_WORLD)

    assert subject_pose_at(script, 1.0).position == pytest.approx((0.0, 0.0, 1.0))


def test_the_start_delay_shifts_the_whole_trajectory_later():
    script = parse_actor_script(DELAYED_ONE_WAY_WORLD)

    assert subject_pose_at(script, 4.0).position == pytest.approx((2.0, 0.0, 1.0))


def test_without_loop_the_subject_stays_on_its_last_waypoint():
    script = parse_actor_script(DELAYED_ONE_WAY_WORLD)

    assert subject_pose_at(script, 50.0).position == pytest.approx((4.0, 0.0, 1.0))


def test_waypoint_pose_in_degrees_is_read_as_radians():
    world = """<sdf version="1.9"><world name="w"><actor name="subject"><script>
      <trajectory id="0" type="walk">
        <waypoint><time>0</time><pose degrees="true">0 0 1 0 0 90</pose></waypoint>
        <waypoint><time>1</time><pose degrees="true">1 0 1 0 0 90</pose></waypoint>
      </trajectory></script></actor></world></sdf>"""

    script = parse_actor_script(world)

    assert script.waypoints[0].rpy[2] == pytest.approx(1.5707963268)


def test_world_without_the_named_actor_is_rejected():
    world = '<sdf version="1.9"><world name="w"></world></sdf>'

    with pytest.raises(ValueError, match='subject'):
        parse_actor_script(world)


def test_shipped_world_names_the_frame_poses_are_published_in():
    template = (PACKAGE_ROOT / 'worlds' / 'cine_field.sdf.template').read_text()

    script = parse_actor_script(render_world(template, CALM))

    assert script.world_name == 'cine_field'


def test_shipped_world_actor_loops_without_delay():
    template = (PACKAGE_ROOT / 'worlds' / 'cine_field.sdf.template').read_text()

    script = parse_actor_script(render_world(template, CALM))

    assert (script.loop, script.delay_start) == (True, 0.0)


def test_shipped_world_actor_has_the_nine_written_waypoints():
    template = (PACKAGE_ROOT / 'worlds' / 'cine_field.sdf.template').read_text()

    script = parse_actor_script(render_world(template, CALM))

    assert [waypoint.time for waypoint in script.waypoints] == [
        0.0,
        5.0,
        6.0,
        7.7,
        8.7,
        13.7,
        14.7,
        16.4,
        17.4,
    ]


def test_shipped_world_actor_waypoint_carries_its_written_pose():
    template = (PACKAGE_ROOT / 'worlds' / 'cine_field.sdf.template').read_text()

    script = parse_actor_script(render_world(template, CALM))

    assert (script.waypoints[7].position, script.waypoints[7].rpy) == (
        (-3.0, 9.0, 1.0),
        (0.0, 0.0, 4.7124),
    )
