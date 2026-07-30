import os
import yaml
import json
import sys
import rclpy
from rclpy.node import Node
from interfaces.action import Progress
from std_msgs.msg import String
from largemodel.utils import large_model_interface
from largemodel.utils.large_model_interface import ModelInitError
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory
from largemodel.utils.promot import get_prompt
from largemodel.utils.compound_navigation import (
    COMPOUND_DESTINATIONS,
    GATEWAY_POINT_NAMES,
    find_gateway_symbol,
)

import time
import re
import functools


def measure_execution_time(func):
    """
    装饰器：测量函数执行时间并使用 ROS 日志打印结果
    """
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        # 记录开始时间
        start_time = time.time()
        # 调用原函数
        result = func(self, *args, **kwargs)
        # 记录结束时间
        end_time = time.time()
        # 计算执行时间
        execution_time = end_time - start_time
        # 使用 ROS 日志系统记录执行时间
        if hasattr(self, 'get_logger'):
            self.get_logger().info(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        else:
            # 如果没有 ROS 日志系统，则打印执行时间
            print(f"[性能统计] {func.__name__} 函数执行时间: {execution_time:.4f} 秒")
        return result
    return wrapper

class LargeModelService(Node):
    def __init__(self):
        super().__init__("LargeModelService")

        self.init_param_config()  # 初始化参数配置 
        self.init_largemodel()  # 初始化大模型 
        self.init_ros_comunication()  # 初始化ROS通信 

        self.get_logger().info(
            "LargeModelService node Initialization completed..."
        )  # 打印日志 
        msg = String(data=f"初始化完成，可直接执行功能")
        self.text_pub.publish(msg)

    def init_largemodel(self):
        try:
            # 创建模型接口客户端 
            self.model_client = large_model_interface.model_interface()
        except ModelInitError as e:
            self.fatal(str(e))
        self.model_client.init_Multimodel()  # 初始化执行层模型，决策层模型无需初始化 
        self.model_client.init_Map_mapping(
            self.get_map_mapping()
        )
        self.new_order_cycle = True  # 新指令周期标志 

    def init_param_config(self):
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        # 参数声明 
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("is_dual_model", False)
        self.declare_parameter("fixed_command_mode", True)
        self.declare_parameter("integrated_nav_mode", False)
        # 获取参数服务器参数 
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        self.isdual_model = (
            self.get_parameter("is_dual_model").get_parameter_value().bool_value
        )
        self.fixed_command_mode = (
            self.get_parameter("fixed_command_mode").get_parameter_value().bool_value
        )
        self.integrated_nav_mode = (
            self.get_parameter("integrated_nav_mode").get_parameter_value().bool_value
        )
        # 设置夹取启动文件路径 
        self.map_mapping_config = os.path.join(self.pkg_path, "config", "map_mapping.yaml")
        self.seewhat_func = False


    def init_ros_comunication(self):
        # 创建执行动作状态订阅者 
        self.actionstatus_sub = self.create_subscription(
            String, "actionstatus", self.actionstatus_callback, 1
        )
        # 创建动作客户端，连接到 'action_service' 
        self._action_client = ActionClient(self, Progress, "action_service")
        # asr话题订阅者 
        self.asrsub = self.create_subscription(String, "voice_words", self.asr_callback, 1)
        # 创建seewhat订阅者 
        self.seewhat_sub = self.create_subscription(
            String, "seewhat_handle", self.seewhat_callback, 1
        )
        # 创建文字交互发布者 
        self.text_pub = self.create_publisher(String, "feedback_words", 1)
        

    def asr_callback(self, msg):
        prompt = msg.data.strip()
        if not prompt:
            self.get_logger().warning("Ignore empty voice_words message.")
            return
        normalized_prompt = self.normalize_voice_prompt(prompt)
        if normalized_prompt != prompt:
            self.get_logger().warning(
                f"语音白名单纠正: {prompt} -> {normalized_prompt}"
            )
            prompt = normalized_prompt
        try:
            self.action_agent(type="text", prompt=prompt)
        except Exception as exc:
            self.get_logger().error(f"Failed to process voice instruction: {exc}")
            self.text_pub.publish(String(data="指令处理失败，请稍后再试"))
        
    def actionstatus_callback(self, msg):
        if (
            msg.data == "finish"
        ):  # 如果收到的是finish则表示当前指令执行完成，开启新的指令执行周期 
            self.new_order_cycle = True
            self.get_logger().info(
                f"The current instruction cycle has ended"
            )  # 当前指令周期已结束...
        # if (
        #     msg.data == "get_current_pose_success"
        # ):
        else:  # 向指令执行层大模型反馈动作执行结果 
            self.get_logger().info(
                f"action_status:{msg.data}"
            ) 


    def seewhat_callback(self, msg):
        self.get_logger().info(
                f"seewhat_use_vision_largemodel"
            ) 
        if msg.data == "seewhat":
            self.action_agent(type="image",prompt="")
        else:
            self.seewhat_func = True
            self.action_agent(type="image",prompt=(f"继续执行{msg.data}"))
    
    def action_agent(self, type, prompt):
        if self.new_order_cycle:  # 判断是否是新任务周期 
            # 判断上一轮对话指令是否完成如果完成就清空历史上下文，开启新的上下文 
            self.model_client.init_Multimodel_history(
                get_prompt()
            )  # 初始化执行层上下文历史
            self.new_order_cycle = False 
            
        if type == "text":
            self.model_client.init_Map_mapping(
                self.get_map_mapping()
            )
            if self.isdual_model :
                execute_instructions = self.model_client.TaskDecision(
                    prompt
                )
                if not execute_instructions[0]:
                    self.get_logger().error(
                        f"LargeScaleModel return: {execute_instructions[1]} ,The format was unexpected. "
                    )
        
        self.instruction_process(
            type=type, prompt=prompt
        )  # 调用执行层大模型生成成动作列表并执行 


    # @measure_execution_time
    def instruction_process(self, type, prompt):
        """
        根据输入信息的类型（文字/图片），构建不同的请求体进行推理，并返回结果）
        Based on the type of input information (text/image), construct different request bodies for inference and return the result.
        """
        if type == "text":
            fixed_result = self.build_fixed_command(prompt)
            if fixed_result is not None:
                action_list, response = fixed_result
                if self.text_chat_mode:
                    self.text_pub.publish(String(data=response))
                self.get_logger().info(
                    f'固定口令直接执行: "action": {action_list}, '
                    f'"response": {response}'
                )
                self.send_action_service(action_list, response)
                return

            raw_content = self.model_client.multimodelinfer(prompt)
            parsed_response = self.parse_model_response(raw_content)
            if parsed_response is None:
                self.get_logger().error(
                    f"Invalid model response format: {str(raw_content)[:500]}"
                )
                return
            action_list, llm_response = parsed_response
            action_list = self.apply_voice_motion_safety(action_list, prompt)
            action_list = self.correct_navigation_target(action_list, prompt)
            if self.text_chat_mode:
                msg = String(data=f'{llm_response}')
                self.text_pub.publish(msg)
            self.get_logger().info(
                f'"action": {action_list}, "response": {llm_response}'
            )
            self.send_action_service(
                action_list, llm_response
            )  # 异步发送动作列表、回复内容给ActionServer 
        
        elif type == "image":
            if self.seewhat_func :
                prompt_seewhat = "识别图片中的所有物体,并以JSON格式输出其bbox的坐标及其中文名称"
                bbox_json = self.model_client.multimodelinfer(
                    prompt_seewhat, image_path=self.image_save_path, seewhat_func=True
                )
                self.get_logger().info(f'{bbox_json}')
                raw_content = self.model_client.multimodelinfer(
                    prompt+bbox_json, image_path=self.image_save_path
                )
                
            else:
                prompt_seewhat = "机器人反馈:执行seewhat()完成"
                raw_content = self.model_client.multimodelinfer(
                    prompt_seewhat+prompt, image_path=self.image_save_path
                )
            parsed_response = self.parse_model_response(raw_content)
            if parsed_response is None:
                self.get_logger().error(
                    f"Invalid vision model response format: {str(raw_content)[:500]}"
                )
                return
            action_list, llm_response = parsed_response
            if self.text_chat_mode and not self.seewhat_func:
                msg = String(data=f'{llm_response}')
                self.text_pub.publish(msg)
            self.get_logger().info(
                f'"response": {llm_response}'
            )
            self.send_action_service(
                action_list, llm_response
            )  # 异步发送动作列表、回复内容给ActionServe
            self.seewhat_func = False

    def apply_voice_motion_safety(self, action_list, prompt):
        """未听到明确距离时，将单次平移动作限制在约20厘米内。"""
        has_distance = re.search(
            r"(?:\d+(?:\.\d+)?|[零一二两三四五六七八九十百半]+)\s*"
            r"(?:毫米|厘米|公分|米)",
            prompt,
        )
        if has_distance:
            return action_list

        safe_actions = []
        for action in action_list:
            match = re.fullmatch(r"\s*set_cmdvel\(([^)]*)\)\s*", action)
            if not match:
                safe_actions.append(action)
                continue
            try:
                linear_x, linear_y, angular_z, duration = (
                    float(value.strip()) for value in match.group(1).split(",")
                )
            except (TypeError, ValueError):
                safe_actions.append(action)
                continue
            if linear_x == 0.0 and linear_y == 0.0:
                safe_actions.append(action)
                continue

            linear_x = max(-0.2, min(0.2, linear_x))
            linear_y = max(-0.2, min(0.2, linear_y))
            duration = max(0.0, min(1.0, duration))
            safe_action = (
                f"set_cmdvel({linear_x:g},{linear_y:g},"
                f"{angular_z:g},{duration:g})"
            )
            self.get_logger().warning(
                f"语音未包含明确距离，安全限幅: {action} -> {safe_action}"
            )
            safe_actions.append(safe_action)
        return safe_actions

    def correct_navigation_target(self, action_list, prompt):
        """优先按用户说出的点位名称解析导航目标，避免名称与字母键冲突。"""
        resolved_target = self.resolve_named_navigation_target(prompt)
        if resolved_target is None:
            return action_list
        matched_symbol, matched_name = resolved_target

        corrected_actions = []
        for action in action_list:
            if re.fullmatch(r"\s*navigation\([^)]*\)\s*", action):
                corrected_action = f"navigation({matched_symbol})"
                if corrected_action != action:
                    self.get_logger().warning(
                        f"按点位名称修正导航目标: {action} -> {corrected_action}"
                        f" ({matched_name})"
                    )
                corrected_actions.append(corrected_action)
            else:
                corrected_actions.append(action)
        return corrected_actions

    @staticmethod
    def compact_voice_text(prompt):
        """去掉识别文本中的空格和标点，但保留命令本身。"""
        return re.sub(r"[\s，。！？、；：,.!?;:\"'‘’“”]+", "", prompt).casefold()

    @staticmethod
    def canonical_point_name(raw_target):
        target = raw_target.strip(" ‘ ’ “ ” \" '，。！？!?、")
        target = re.sub(r"^(?:点位|位置)", "", target)
        target = re.sub(r"(?:那里|那儿|位置|处)$", "", target)
        chinese_aliases = {
            "一": "a", "1": "a",
            "一号": "a", "一号点": "a", "一号店": "a",
            "二": "b", "2": "b",
            "二号": "b", "二号点": "b", "二号店": "b",
            "三": "c", "3": "c",
            "三号": "c", "三号点": "c", "三号店": "c",
            "四": "d", "4": "d",
            "四号": "d", "四号点": "d", "四号店": "d",
            "1号": "a", "1号点": "a", "1号店": "a",
            "2号": "b", "2号点": "b", "2号店": "b",
            "3号": "c", "3号点": "c", "3号店": "c",
            "4号": "d", "4号点": "d", "4号店": "d",
        }
        if target in chinese_aliases:
            return chinese_aliases[target]
        letter_match = re.fullmatch(r"([a-dA-Di-kI-K])(?:点|店)?", target)
        if letter_match:
            return letter_match.group(1).casefold()
        return target

    def normalize_voice_prompt(self, prompt):
        """把固定口令和可确定的点位口令收敛为标准写法。"""
        compact = self.compact_voice_text(prompt)

        fixed_aliases = {
            "小车向前": "小车前进",
            "小车前进无": "小车前进",
            "小车向后": "小车后退",
            "小车后退役": "小车后退",
            "小车停止": "小车停",
            "开启自主建图": "打开自主建图",
            "结束自主建图": "关闭自主建图",
            "开启导航": "开始导航",
            "结束导航": "关闭导航",
        }
        canonical_fixed = {
            "小车前进", "小车后退", "小车左转", "小车右转", "小车停",
            "小车休眠", "小车过来", "打开自主建图", "关闭自主建图",
            "开始导航", "关闭导航",
        }
        if compact in fixed_aliases:
            return fixed_aliases[compact]
        if compact in canonical_fixed:
            return compact

        record_match = re.search(
            r"(?:(?:记录|记住)(?:一下)?)?"
            r"(?:当前位置|现在位置|这里|这个位置|位置)"
            r"(?:为|是|叫做|叫)(.+)$",
            compact,
        )
        if record_match:
            target = self.canonical_point_name(record_match.group(1))
            if self.is_fixed_point_name(target):
                return f"记录当前位置为{target}"

        resolved_target = self.resolve_named_navigation_target(prompt)
        if resolved_target is not None:
            _, matched_name = resolved_target
            return f"前往{matched_name}"

        nav_match = re.search(
            r"(?:导航到|前+往|到达|移动到|小车去|去到)(.+)$",
            compact,
        )
        if nav_match:
            prefix = compact[:nav_match.start()]
            if not re.search(r"(?:不要|别|取消|停止|禁止|不用|不想).{0,8}$", prefix):
                target = self.canonical_point_name(nav_match.group(1))
                if self.is_fixed_point_name(target):
                    return f"前往{target}"
        return prompt

    @staticmethod
    def is_fixed_point_name(target):
        normalized_target = target.casefold()
        return normalized_target in {
            "a", "b", "c", "d", "i", "j", "k",
            "沙发", "桌子", "箱子", "办公室", "电梯", "大厅", "前台",
            "会议室", "会议室门口",
        } or normalized_target in COMPOUND_DESTINATIONS \
            or normalized_target in GATEWAY_POINT_NAMES

    def load_target_points(self):
        try:
            with open(self.map_mapping_config, "r", encoding="utf-8") as file:
                return yaml.safe_load(file) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"读取点位映射失败: {exc}")
            return {}

    def resolve_point_symbol(self, target):
        """点位名称优先于字母键，例如名称 a 可以实际存放在 E 键。"""
        target_points = self.load_target_points()
        normalized_target = self.canonical_point_name(target).casefold()
        for symbol, point in target_points.items():
            if not isinstance(point, dict):
                continue
            name = str(point.get("name", "")).strip(" ‘ ’ “ ” \" '")
            if name.casefold() == normalized_target:
                return str(symbol), name

        # a-d 是用户记录的点位名称，绝不能回退成系统内置键 A-D。
        if normalized_target in {"a", "b", "c", "d"}:
            return None

        symbol = normalized_target.upper()
        if len(symbol) == 1 and symbol in target_points:
            point = target_points[symbol]
            name = str(point.get("name", symbol)) if isinstance(point, dict) else symbol
            return symbol, name
        return None

    def build_fixed_command(self, prompt):
        """固定演示口令不再请求大模型，直接生成确定性的安全动作。"""
        if not self.fixed_command_mode:
            return None

        compact = self.compact_voice_text(prompt)
        if re.search(
            r"(?:记录|记住).*(?:当前位置|现在位置|位置)(?:为|是|叫做|叫)?$",
            compact,
        ):
            return ["finishtask()"], "没有听清点位名称，请重新说完整指令"

        fixed_actions = {
            "小车前进": (["set_cmdvel(0.2,0,0,1)", "finishtask()"], "好的，向前移动二十厘米"),
            "小车后退": (["set_cmdvel(-0.2,0,0,1)", "finishtask()"], "好的，向后移动二十厘米"),
            "小车左转": (["move_left(30,0.5)", "finishtask()"], "好的，向左转三十度"),
            "小车右转": (["move_right(30,0.5)", "finishtask()"], "好的，向右转三十度"),
            "小车停": (["stop()", "finishtask()"], "小车已停止"),
            "小车休眠": (["stop()", "finishtask()"], "小车已停止，等待下次唤醒"),
            "小车过来": (["finishtask()"], "当前大模型模式未启用声源跟随，请使用雷达跟随"),
        }
        if prompt in fixed_actions:
            return fixed_actions[prompt]

        record_match = re.fullmatch(r"记录当前位置为(.+)", prompt)
        if record_match:
            name = self.canonical_point_name(record_match.group(1))
            if self.is_fixed_point_name(name):
                return [f"get_current_pose({name})", "finishtask()"], f"正在记录当前位置为{name}"

        nav_match = re.fullmatch(r"前往(.+)", prompt)
        if nav_match:
            target = self.canonical_point_name(nav_match.group(1))
            if self.is_fixed_point_name(target):
                if target.casefold() in COMPOUND_DESTINATIONS:
                    target_point = self.resolve_point_symbol(target)
                    if target_point is None:
                        return (
                            ["finishtask()"],
                            f"还没有记录{target}点，请先记录当前位置",
                        )
                    target_symbol, name = target_point
                    target_points = self.load_target_points()
                    entrance_symbol = find_gateway_symbol(target_points, target)
                    if entrance_symbol is None:
                        return (
                            ["finishtask()"],
                            f"还没有记录{target}入口，请先记录入口位置和朝向",
                        )
                    return (
                        [f"navigation_compound({target_symbol})"],
                        f"好的，开始前往{name}",
                    )
                resolved = self.resolve_point_symbol(target)
                if resolved is None:
                    return ["finishtask()"], f"还没有记录{target}点，请先记录当前位置"
                symbol, name = resolved
                return [f"navigation({symbol})"], f"好的，开始前往{name}"

        mode_actions = {
            "打开自主建图": ("slam_start()", "开始自主建图"),
            "关闭自主建图": ("slam_stop()", "关闭自主建图并保存地图"),
            "开始导航": ("navigation_start()", "开始导航"),
            "关闭导航": ("navigation_stop()", "关闭导航"),
        }
        if prompt in mode_actions:
            if self.integrated_nav_mode:
                if prompt == "开始导航":
                    return ["finishtask()"], "导航已经随当前程序启动"
                return ["finishtask()"], "当前是联合导航模式，请在终端切换建图或导航"
            action, response = mode_actions[prompt]
            return [action, "finishtask()"], response
        return None

    def resolve_named_navigation_target(self, prompt):
        """从语音文本中解析已有点位名称；否定指令不会被转换为导航。"""
        target_match = re.search(
            r"(?:导航到|前往|到达|移动到|去到|去)\s*[‘’“”\"']?(.+?)"
            r"[‘’“”\"']?[。！？!?\s]*$",
            prompt,
            re.IGNORECASE,
        )
        if not target_match:
            return None

        prefix = prompt[:target_match.start()]
        if re.search(r"(?:不要|别|取消|停止|禁止|不用|不想).{0,8}$", prefix):
            return None

        spoken_target = target_match.group(1).strip(" ‘ ’ “ ” \" '")
        if not spoken_target:
            return None

        target_points = self.load_target_points()
        if not target_points:
            return None

        normalized_target = spoken_target.casefold()
        target_variants = {normalized_target}
        if normalized_target.startswith("点位"):
            target_variants.add(normalized_target[2:])
        for suffix in ("位置", "那里", "那儿", "处"):
            if normalized_target.endswith(suffix):
                target_variants.add(normalized_target[:-len(suffix)])
        # 在线识别偶尔会把“点”识别成“店”，只在已有点位名称中做精确回查。
        if normalized_target.endswith("店"):
            target_variants.add(normalized_target[:-1])
        for symbol, point in target_points.items():
            if not isinstance(point, dict):
                continue
            name = str(point.get("name", "")).strip(" ‘ ’ “ ” \" '")
            if not name:
                continue
            normalized_name = name.casefold()
            if target_variants.intersection((normalized_name, normalized_name + "点")):
                return str(symbol), name
        return None
            
    def send_action_service(self, actions, text):
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Action service is not available.")
            self.text_pub.publish(String(data="动作服务未就绪，请稍后再试"))
            return
        goal_msg = Progress.Goal()  # 创建目标消息对象 
        goal_msg.actions = actions  # 设置目标消息中的动作列表 
        goal_msg.llm_response = text
        self._send_goal_future = self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        # 添加目标发送后的响应回调函数 
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()  # 获取目标句柄
        except Exception as exc:
            self.get_logger().error(f"Failed to send action goal: {exc}")
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info(
                "action_client message: action service rejected action list"
            )  # 目标被拒绝...
            return
    #     self.get_logger().info(
    #         "action_client message: action service accepted action list"
    #     )
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
        
    
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            "Received feedback: {}".format(feedback_msg.feedback.status)
        )
        
    @staticmethod
    def parse_model_response(raw_content):
        json_str = LargeModelService.extract_json_content(raw_content)
        if json_str is None:
            return None
        try:
            action_plan_json = json.loads(json_str)
        except (TypeError, json.JSONDecodeError):
            return None

        action_list = action_plan_json.get("action", [])
        if isinstance(action_list, str):
            action_list = [action_list]
        if not isinstance(action_list, list):
            return None
        action_list = [action for action in action_list if isinstance(action, str) and action.strip()]
        llm_response = str(action_plan_json.get("response", ""))
        return action_list, llm_response

    @staticmethod
    def extract_json_content(
        raw_content,
    ):  # 解析变量提取json 
        try:
            # 方法一：分割代码块 
            if "```json" in raw_content:
                # 分割代码块并取中间部分 
                json_str = raw_content.split("```json")[1].split("```")[0].strip()
            elif "```" in raw_content:
                # 处理没有指定类型的代码块 
                json_str = raw_content.split("```")[1].strip()
            else:
                # 直接尝试解析 
                json_str = raw_content

            # 方法二：正则表达式提取（备用方案） 
            if not json_str:

                match = re.search(r"\{.*\}", raw_content, re.DOTALL)
                if match:
                    json_str = match.group()
            return json_str
        except Exception as e:
            return None
    
    def fatal(self, msg: str):
        self.get_logger().fatal(msg)
        rclpy.shutdown()   # 停止 executor
        sys.exit(1)        # 告诉 launch 异常退出

    def get_map_mapping(self):
        '''
        获取地图映射关系
        '''
        with open(self.map_mapping_config, 'r', encoding='utf-8') as file:
            yaml_data = yaml.safe_load(file)
        map_mapping = "#地图映射\n\n"
        # 遍历 YAML 数据，提取符号和名称
        for symbol, area_info in yaml_data.items():
            name = area_info['name']
            map_mapping += f"'{symbol}': '{name}',\n"
        return map_mapping


def main(args=None):
    rclpy.init(args=args)
    model_service = LargeModelService()
    rclpy.spin(model_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
