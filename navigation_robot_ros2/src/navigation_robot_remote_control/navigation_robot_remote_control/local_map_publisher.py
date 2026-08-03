"""Publish a saved Nav2 PGM/YAML map locally on the Ubuntu control PC."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


@dataclass(frozen=True)
class MapFile:
    """Map metadata and row-major occupancy values."""

    resolution: float
    width: int
    height: int
    origin: tuple[float, float, float]
    data: list[int]


def _pgm_token(stream) -> bytes:
    """Read one whitespace-delimited PGM token while skipping comments."""
    token = bytearray()
    while True:
        byte = stream.read(1)
        if not byte:
            break
        if byte == b'#':
            stream.readline()
            if token:
                break
            continue
        if byte.isspace():
            if token:
                break
            continue
        token.extend(byte)
    if not token:
        raise ValueError('unexpected end of PGM header')
    return bytes(token)


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    """Read an 8-bit P2 or P5 PGM image."""
    with path.open('rb') as stream:
        magic = _pgm_token(stream)
        width = int(_pgm_token(stream))
        height = int(_pgm_token(stream))
        maximum = int(_pgm_token(stream))
        if width <= 0 or height <= 0 or maximum <= 0 or maximum > 255:
            raise ValueError('only non-empty 8-bit PGM maps are supported')
        count = width * height
        if magic == b'P5':
            pixels = list(stream.read(count))
        elif magic == b'P2':
            pixels = [int(_pgm_token(stream)) for _ in range(count)]
        else:
            raise ValueError(f'unsupported PGM format: {magic!r}')
    if len(pixels) != count:
        raise ValueError(
            f'PGM data is truncated: expected {count}, got {len(pixels)}')
    if maximum != 255:
        pixels = [round(pixel * 255 / maximum) for pixel in pixels]
    return width, height, pixels


def load_map_file(yaml_path: str | Path) -> MapFile:
    """Load the Nav2 map YAML and reproduce trinary map-server conversion."""
    yaml_path = Path(yaml_path).expanduser().resolve()
    with yaml_path.open('r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError('map YAML must contain a mapping')
    if str(metadata.get('mode', 'trinary')).lower() != 'trinary':
        raise ValueError('the local publisher currently supports trinary maps')

    image_path = Path(str(metadata['image'])).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height, pixels = read_pgm(image_path.resolve())
    resolution = float(metadata['resolution'])
    origin_values = metadata['origin']
    if resolution <= 0.0 or len(origin_values) != 3:
        raise ValueError('invalid map resolution or origin')
    origin = tuple(float(value) for value in origin_values)
    negate = bool(int(metadata.get('negate', 0)))
    occupied_threshold = float(metadata['occupied_thresh'])
    free_threshold = float(metadata['free_thresh'])

    data: list[int] = []
    # PGM rows start at the top; OccupancyGrid rows start at map origin.
    for map_y in range(height):
        image_y = height - map_y - 1
        row_start = image_y * width
        for pixel in pixels[row_start:row_start + width]:
            shade = pixel / 255.0
            occupancy = shade if negate else 1.0 - shade
            if occupancy > occupied_threshold:
                data.append(100)
            elif occupancy < free_threshold:
                data.append(0)
            else:
                data.append(-1)
    return MapFile(resolution, width, height, origin, data)


class LocalMapPublisher(Node):
    """Load a map from disk and publish it without sending it over Wi-Fi."""

    def __init__(self) -> None:
        super().__init__('navigation_robot_local_map_publisher')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_topic', '/map_pc')
        self.declare_parameter('publish_period_sec', 2.0)
        yaml_path = str(self.get_parameter('map_yaml').value)
        map_topic = str(self.get_parameter('map_topic').value)
        period = float(self.get_parameter('publish_period_sec').value)
        if not yaml_path:
            raise ValueError('map_yaml parameter is required')
        if period <= 0.0:
            raise ValueError('publish_period_sec must be positive')

        source = load_map_file(yaml_path)
        self.message = OccupancyGrid()
        self.message.header.frame_id = 'map'
        self.message.info.resolution = source.resolution
        self.message.info.width = source.width
        self.message.info.height = source.height
        self.message.info.origin.position.x = source.origin[0]
        self.message.info.origin.position.y = source.origin[1]
        half_yaw = source.origin[2] / 2.0
        self.message.info.origin.orientation.z = math.sin(half_yaw)
        self.message.info.origin.orientation.w = math.cos(half_yaw)
        self.message.data = source.data

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            OccupancyGrid, map_topic, qos)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_anchor()
        self._publish()
        self.timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f'local map {source.width}x{source.height} loaded from {yaml_path}; '
            f'publishing {map_topic}')

    def _publish_anchor(self) -> None:
        """Make the map frame selectable before AMCL has an initial pose."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'map_visualization_anchor'
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        self.message.header.stamp = now
        self.message.info.map_load_time = now
        self.publisher.publish(self.message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LocalMapPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
