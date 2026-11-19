#!/usr/bin/env python3
"""Count messages on several ROS 2 topics at once and print their rates as one JSON object.

    python3 tools/measure_topic_rates.py --seconds 5 /camera/image_raw=sensor_msgs/msg/Image

One process subscribes to every topic for the same window. Running `ros2 topic hz` once per
topic instead waits out discovery each time and, under the simulator's CPU load, often printed
no rate at all for topics that were flowing. Rates are wall-clock messages per second.
"""

import argparse
import json
import sys
import time

USAGE_EXIT_CODE = 2


def rate_from_arrivals(arrival_times: list[float]) -> float | None:
    """Messages per second between the first and last arrival; None with fewer than two."""

    if len(arrival_times) < 2:
        return None

    return round((len(arrival_times) - 1) / (arrival_times[-1] - arrival_times[0]), 3)


def build_report(arrivals_by_topic: dict[str, list[float]], window_seconds: float) -> dict:
    return {
        'window_seconds': window_seconds,
        'topics': {
            topic: {'messages': len(arrivals), 'rate_hz': rate_from_arrivals(arrivals)}
            for topic, arrivals in arrivals_by_topic.items()
        },
    }


def parse_topic_types(arguments: list[str]) -> dict[str, str]:
    topic_types = {}

    for argument in arguments:
        topic, separator, message_type = argument.partition('=')
        if not separator or not topic or not message_type:
            raise ValueError(f'{argument} needs the form TOPIC=MESSAGE_TYPE')

        topic_types[topic] = message_type

    return topic_types


def _arrival_recorder(arrivals: list[float]):
    # A one-parameter function: rclpy passes message info as well to callbacks that take two.
    def record(serialized_message) -> None:
        arrivals.append(time.monotonic())

    return record


def measure(topic_types: dict[str, str], window_seconds: float) -> dict[str, list[float]]:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node('measure_topic_rates')
    arrivals_by_topic = {topic: [] for topic in topic_types}

    try:
        for topic, message_type in topic_types.items():
            # Raw subscriptions skip deserializing camera frames; best-effort quality of
            # service still matches reliable publishers, so every publisher is counted.
            node.create_subscription(
                get_message(message_type),
                topic,
                _arrival_recorder(arrivals_by_topic[topic]),
                qos_profile_sensor_data,
                raw=True,
            )

        deadline = time.monotonic() + window_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    return arrivals_by_topic


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seconds', type=float, default=5.0, help='measurement window')
    parser.add_argument('topics', nargs='+', metavar='TOPIC=MESSAGE_TYPE')
    arguments = parser.parse_args(argv)

    try:
        topic_types = parse_topic_types(arguments.topics)
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)

        return USAGE_EXIT_CODE

    arrivals_by_topic = measure(topic_types, arguments.seconds)
    print(json.dumps(build_report(arrivals_by_topic, arguments.seconds)))

    return 0


if __name__ == '__main__':
    sys.exit(main())
