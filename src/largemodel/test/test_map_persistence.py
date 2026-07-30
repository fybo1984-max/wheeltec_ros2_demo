import os

import yaml

from largemodel.action_service import CustomActionServer
from largemodel.utils.compound_navigation import compose_compound_route


def test_atomic_map_write_preserves_install_symlink(tmp_path):
    build_file = tmp_path / "build" / "map_mapping.yaml"
    build_file.parent.mkdir()
    build_file.write_text("{}\n", encoding="utf-8")
    install_link = tmp_path / "install" / "map_mapping.yaml"
    install_link.parent.mkdir()
    install_link.symlink_to(build_file)

    points = {"E": {"name": "a", "position": {"x": 1.0, "y": 2.0}}}
    CustomActionServer.write_map_mapping(str(install_link), points)

    assert install_link.is_symlink()
    with open(build_file, "r", encoding="utf-8") as file:
        assert yaml.safe_load(file) == points


def test_runtime_map_can_be_copied_atomically_to_source(tmp_path):
    runtime_file = tmp_path / "runtime" / "map_mapping.yaml"
    source_file = tmp_path / "source" / "map_mapping.yaml"
    points = {"F": {"name": "箱子"}}

    CustomActionServer.write_map_mapping(str(runtime_file), points)
    CustomActionServer.write_map_mapping(str(source_file), points)

    assert os.path.isfile(runtime_file)
    assert runtime_file.read_bytes() == source_file.read_bytes()


def test_new_point_keys_start_at_a_when_system_uses_wxyz():
    target_points = {
        "W": {"name": "原点起点"},
        "X": {"name": "水果店"},
        "Y": {"name": "工具间"},
        "Z": {"name": "便利店"},
    }
    assert CustomActionServer.next_point_key(target_points, "a") == "A"
    assert CustomActionServer.next_point_key(target_points, "桌子") == "E"
    assert CustomActionServer.next_point_key(target_points, "箱子") == "F"
    assert CustomActionServer.next_point_key(target_points, "沙发") == "G"
    assert CustomActionServer.next_point_key(target_points, "椅子") == "H"


def test_navigation_sequence_stops_after_failed_intermediate_goal():
    class FakeServer:
        interrupt_flag = False

        def __init__(self):
            self.visited = []

        def navigation(self, point_name, report_success=True):
            self.visited.append(point_name)
            return point_name != "R"

        def get_logger(self):
            class Logger:
                def warning(self, _message):
                    pass

            return Logger()

    server = FakeServer()
    assert CustomActionServer.navigation_sequence(server, "R", "M") is False
    assert server.visited == ["R"]


def test_compound_route_exits_current_room_before_target_entrance():
    points = {
        "M": {"name": "会议室", "position": {"x": 1.0, "y": 1.0}},
        "S": {"name": "会议室门口", "position": {"x": 0.0, "y": 1.0}},
        "K": {"name": "办公室", "position": {"x": 10.0, "y": 1.0}},
        "R": {"name": "办公室入口", "position": {"x": 9.0, "y": 1.0}},
    }

    assert compose_compound_route(
        points, "K", source_symbol="M"
    ) == ["S", "R", "K"]
    assert compose_compound_route(points, "K") == ["R", "K"]


def test_compound_route_does_not_use_non_meeting_room_source_entrance():
    points = {
        "M": {"name": "会议室", "position": {"x": 1.0, "y": 1.0}},
        "S": {"name": "会议室门口", "position": {"x": 0.0, "y": 1.0}},
        "K": {"name": "办公室", "position": {"x": 10.0, "y": 1.0}},
        "R": {"name": "办公室入口", "position": {"x": 9.0, "y": 1.0}},
    }

    assert compose_compound_route(
        points, "M", source_symbol="K"
    ) == ["S", "M"]
