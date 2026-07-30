import pytest
import yaml

from largemodel.model_service import LargeModelService
from largemodel.utils.compound_navigation import gateway_names


class DummyLogger:
    def warning(self, _message):
        pass


def make_service(tmp_path):
    service = LargeModelService.__new__(LargeModelService)
    service.fixed_command_mode = True
    service.integrated_nav_mode = True
    service.map_mapping_config = str(tmp_path / "map_mapping.yaml")
    service.get_logger = lambda: DummyLogger()
    with open(service.map_mapping_config, "w", encoding="utf-8") as file:
        yaml.safe_dump(
            {
                "X": {"name": "水果店"},
                "E": {"name": "a"},
                "I": {"name": "c"},
                "S": {"name": "沙发"},
                "M": {"name": "会议室"},
                "R": {"name": "会议室门口"},
                "T": {"name": "一号点入口"},
                "U": {"name": "三号点入口"},
                "V": {"name": "沙发入口"},
            },
            file,
            allow_unicode=True,
        )
    return service


def test_normalizes_observed_asr_errors(tmp_path):
    service = make_service(tmp_path)
    assert service.normalize_voice_prompt("小车前进无") == "小车前进"
    assert service.normalize_voice_prompt("小车后退役") == "小车后退"
    assert service.normalize_voice_prompt("全屏前往a点") == "前往a"
    assert service.normalize_voice_prompt("前往c店") == "前往c"


def test_fixed_navigation_prefers_recorded_name(tmp_path):
    service = make_service(tmp_path)
    assert service.build_fixed_command("前往a")[0] == ["navigation_compound(E)"]
    assert service.build_fixed_command("前往c")[0] == ["navigation_compound(I)"]
    assert service.build_fixed_command("前往沙发")[0] == ["navigation_compound(S)"]


def test_missing_point_never_guesses_a_destination(tmp_path):
    service = make_service(tmp_path)
    actions, response = service.build_fixed_command("前往箱子")
    assert actions == ["finishtask()"]
    assert "还没有记录" in response


def test_named_b_never_falls_back_to_builtin_key_b(tmp_path):
    service = make_service(tmp_path)
    actions, response = service.build_fixed_command("前往b")
    assert actions == ["finishtask()"]
    assert "还没有记录" in response


def test_integrated_navigation_blocks_mode_switches(tmp_path):
    service = make_service(tmp_path)
    actions, _ = service.build_fixed_command("打开自主建图")
    assert actions == ["finishtask()"]


@pytest.mark.parametrize(
    ("spoken", "canonical"),
    [
        ("前往一号点", "前往a"),
        ("前往二号点", "前往b"),
        ("前往三号点", "前往c"),
        ("前往四号点", "前往d"),
        ("记录当前位置为一号点", "记录当前位置为a"),
        ("记录当前位置为二号点", "记录当前位置为b"),
        ("记录当前位置为三号点", "记录当前位置为c"),
        ("记录当前位置为四号点", "记录当前位置为d"),
    ],
)
def test_chinese_only_hotword_aliases(tmp_path, spoken, canonical):
    service = make_service(tmp_path)
    assert service.normalize_voice_prompt(spoken) == canonical


@pytest.mark.parametrize(
    ("spoken", "canonical"),
    [
        ("当前位置为1号店", "记录当前位置为a"),
        ("位置为1号店", "记录当前位置为a"),
        ("记录当前位置为2号店", "记录当前位置为b"),
        ("前往3号店", "前往c"),
        ("前往4号点", "前往d"),
        ("前往2", "前往b"),
        ("记录当前位置为2", "记录当前位置为b"),
        ("记录当前位置为四", "记录当前位置为d"),
    ],
)
def test_online_asr_number_and_shop_character_corrections(
    tmp_path, spoken, canonical
):
    service = make_service(tmp_path)
    assert service.normalize_voice_prompt(spoken) == canonical


def test_incomplete_record_command_never_creates_an_unnamed_point(tmp_path):
    service = make_service(tmp_path)
    actions, response = service.build_fixed_command("当记录当前位置为")
    assert actions == ["finishtask()"]
    assert "没有听清" in response


@pytest.mark.parametrize(
    ("spoken", "canonical"),
    [
        *(('前往' + name + ('点' if len(name) == 1 else ''), '前往' + name)
          for name in (
              "a", "b", "c", "d", "沙发", "桌子", "箱子",
              "办公室", "电梯", "大厅", "前台", "会议室", "会议室门口",
              "充电桩", "充电桩入口",
          )),
        *(('记录当前位置为' + name + ('点' if len(name) == 1 else ''),
           '记录当前位置为' + name)
          for name in (
              "a", "b", "c", "d", "沙发", "桌子", "箱子",
              "办公室", "电梯", "大厅", "前台", "会议室", "会议室门口",
              "充电桩", "充电桩入口",
          )),
        *((command, command) for command in (
            "小车前进", "小车后退", "小车左转", "小车右转", "小车停",
            "小车休眠", "小车过来", "打开自主建图", "关闭自主建图",
            "开始导航", "关闭导航",
        )),
        ("小车去I点", "前往i"),
        ("小车去J点", "前往j"),
        ("小车去K点", "前往k"),
    ],
)
def test_user_fixed_phrase_list_is_normalized(tmp_path, spoken, canonical):
    service = make_service(tmp_path)
    assert service.normalize_voice_prompt(spoken) == canonical


@pytest.mark.parametrize(
    "name",
    ("办公室", "桌子", "箱子", "电梯", "大厅", "前台", "充电桩"),
)
def test_common_semantic_points_can_be_recorded_and_navigated(
    tmp_path, name
):
    service = make_service(tmp_path)
    actions, _ = service.build_fixed_command(f"记录当前位置为{name}")
    assert actions == [f"get_current_pose({name})", "finishtask()"]

    with open(service.map_mapping_config, "w", encoding="utf-8") as file:
        yaml.safe_dump(
            {
                "Q": {"name": name},
                "R": {"name": gateway_names(name)[0]},
            },
            file,
            allow_unicode=True,
        )
    actions, _ = service.build_fixed_command(f"前往{name}")
    assert actions == ["navigation_compound(Q)"]


def test_meeting_room_uses_entrance_then_room(tmp_path):
    service = make_service(tmp_path)

    actions, response = service.build_fixed_command("前往会议室")

    assert actions == ["navigation_compound(M)"]
    assert response == "好的，开始前往会议室"


def test_meeting_room_route_requires_entrance_point(tmp_path):
    service = make_service(tmp_path)
    with open(service.map_mapping_config, "w", encoding="utf-8") as file:
        yaml.safe_dump(
            {"M": {"name": "会议室"}},
            file,
            allow_unicode=True,
        )

    actions, response = service.build_fixed_command("前往会议室")

    assert actions == ["finishtask()"]
    assert "还没有记录会议室入口" in response
