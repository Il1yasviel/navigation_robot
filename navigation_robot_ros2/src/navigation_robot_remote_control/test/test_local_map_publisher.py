from pathlib import Path

import yaml

from navigation_robot_remote_control.local_map_publisher import load_map_file
from navigation_robot_remote_control.local_map_publisher import read_pgm


def test_read_binary_pgm_with_comment(tmp_path: Path):
    pgm = tmp_path / 'test.pgm'
    pgm.write_bytes(b'P5\n# comment\n2 2\n255\n' + bytes([0, 255, 205, 127]))
    assert read_pgm(pgm) == (2, 2, [0, 255, 205, 127])


def test_load_map_flips_rows_and_applies_trinary_thresholds(tmp_path: Path):
    pgm = tmp_path / 'test.pgm'
    pgm.write_bytes(b'P5\n2 2\n255\n' + bytes([0, 255, 205, 127]))
    metadata = {
        'image': 'test.pgm',
        'mode': 'trinary',
        'resolution': 0.05,
        'origin': [-1.0, -2.0, 0.5],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    yaml_path = tmp_path / 'test.yaml'
    yaml_path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    loaded = load_map_file(yaml_path)
    assert (loaded.width, loaded.height) == (2, 2)
    assert loaded.resolution == 0.05
    assert loaded.origin == (-1.0, -2.0, 0.5)
    assert loaded.data == [0, -1, 100, 0]
