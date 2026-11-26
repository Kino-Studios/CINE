import csv
from pathlib import Path

import yaml
from geometry_msgs.msg import PoseArray


class WaypointIOMixin:
    def _load_waypoints_from_file(self, path: str):
        path_obj = Path(path)
        if not path_obj.is_file():
            self.get_logger().error(f'Waypoints file not found: {path}')

            return

        try:
            if path_obj.suffix.lower() in ('.yaml', '.yml'):
                waypoints = self._read_yaml_waypoints(path_obj)
            else:
                waypoints = self._read_csv_waypoints(path_obj)

            if waypoints:
                self.static_waypoints = waypoints
                self.get_logger().info(f'Loaded {len(waypoints)} waypoints from file: {path}')
            else:
                self.get_logger().warn(f'No valid waypoints found in file: {path}')
        except (OSError, csv.Error, yaml.YAMLError) as error:
            self.get_logger().error(f'Failed to load waypoints from file: {error}')

    def _read_csv_waypoints(self, path_obj: Path):
        waypoints = []

        with path_obj.open('r', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue

                if len(row) >= 3:
                    try:
                        x = float(row[0])
                        y = float(row[1])
                        z = float(row[2])
                        waypoints.append((x, y, z))
                    except ValueError:
                        continue

        return waypoints

    def _read_yaml_waypoints(self, path_obj: Path):
        waypoints = []

        with path_obj.open('r', encoding='utf-8') as file_handle:
            entries = yaml.safe_load(file_handle)

        for entry in entries if isinstance(entries, list) else []:
            if isinstance(entry, (list, tuple)) and len(entry) >= 3:
                try:
                    waypoints.append((float(entry[0]), float(entry[1]), float(entry[2])))
                except (TypeError, ValueError):
                    continue

        return waypoints

    def waypoints_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        waypoints = []

        for pose in msg.poses:
            waypoints.append((pose.position.x, pose.position.y, pose.position.z))

        if waypoints:
            self.static_waypoints = waypoints
            self.get_logger().info(f'Received {len(waypoints)} waypoints from topic')
