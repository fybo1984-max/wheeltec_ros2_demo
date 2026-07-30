import cv2
import re
import rclpy
import string
import subprocess
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import time
from cv_bridge import CvBridge
from std_msgs.msg import String,Int8,Bool
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from interfaces.action import Progress
import math
import yaml
import tempfile
import psutil
from concurrent.futures import Future
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from turn_on_wheeltec_robot.msg import Position 
from rclpy.qos import qos_profile_sensor_data
from largemodel.utils.compound_navigation import compose_compound_route

# ---------- 参数 ----------
PREDICT_TIME = 1.5          # s，预测前瞻时间
ROBOT_WIDTH  = 0.3         # m，车身半宽
LIDAR_RANGE  = 0.8         # m，最大刹车距离

def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

class CustomActionServer(Node):
    def __init__(self):
        super().__init__("action_service_node")
        # 初始化参数配置 
        self.init_param_config()
        # 初始化ROS通信 
        self.init_ros_comunication()
        self.init_navigation_client()
        self.get_logger().info("action service started...")

    def init_param_config(self):
        """
        初始化参数配置 / Initialize parameter configuration
        """
        # 设置夹取启动文件路径 
        pkg_share = get_package_share_directory("largemodel")
        self.map_mapping_config = os.path.join(pkg_share, "config", "map_mapping.yaml")
        workspace_root = os.path.dirname(
            os.path.dirname(get_package_prefix("largemodel"))
        )
        default_backup_path = os.path.join(
            workspace_root, "src", "largemodel", "config", "map_mapping.yaml"
        )
        if not os.path.isfile(default_backup_path):
            default_backup_path = ""
        config_param_file = os.path.join(pkg_share, "config", "model_config.yaml")
        with open(config_param_file, "r") as file:
            config_param = yaml.safe_load(file)
        self.multimodel = config_param.get("multimodel")
        # 声明参数 
        self.declare_parameter("Speed_topic", "/cmd_vel")
        self.declare_parameter("text_chat_mode", False)
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("map_mapping_backup_path", default_backup_path)
        self.declare_parameter("monitor_camera", False)
        # 获取参数值 
        self.Speed_topic = (
            self.get_parameter("Speed_topic").get_parameter_value().string_value
        )
        self.text_chat_mode = (
            self.get_parameter("text_chat_mode").get_parameter_value().bool_value
        )
        self.map_mapping_backup_path = (
            self.get_parameter("map_mapping_backup_path")
            .get_parameter_value().string_value
        )
        self.monitor_camera = (
            self.get_parameter("monitor_camera").get_parameter_value().bool_value
        )
        # 创建文字交互发布者 
        self.text_pub = self.create_publisher(String, "feedback_words", 1)
        print(self.text_chat_mode)
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.pkg_path = get_package_share_directory("largemodel")
        self.image_save_path = os.path.join(
            self.pkg_path, "resources_file", "image.png"
        )
        self.positionSubscriber = self.create_subscription(
		    Position,
		    '/object_tracker/current_position',
		    self.distanceCallback,
		    qos_profile=qos_profile_sensor_data)

        def completed_future():
            future = Future()
            future.set_result(True)
            return future

        # 未启动的持续任务必须是“已完成”状态，否则首次导航会误判为正在建图。
        self.visual_follower_future = completed_future()
        self.laser_follower_future = completed_future()
        self.line_follower_future = completed_future()
        self.KCF_follow_future = completed_future()
        self.navigation_process = None
        self.slam_process = None
        self.process_lock = threading.RLock()

        self.interrupt_flag = False  # 打断标志 
        self.action_runing = False  # 动作执行状态 
        self.IS_SAVING = False #是否正在保存图像
        self.obstacle_angle = 0.0 
        self.obstacle_dist = 0.0

        # 图像处理对象 
        self.image_msg = None
        self.bridge = CvBridge()

        self.feedback_largemoel_dict =  {  
            "navigation_1": "机器人反馈:导航目标{point_name}被拒绝",
            "navigation_2": "机器人反馈:执行navigation({point_name})完成",
            "navigation_3": "机器人反馈:执行navigation({point_name})失败，目标点不存在",
            "navigation_4": "机器人反馈:执行navigation({point_name})失败",
            "navigation_5": "机器人反馈:已取消导航到{point_name}",
            "get_current_pose_success": "机器人反馈:get_current_pose()成功",
            "get_current_pose_failed": "机器人反馈:get_current_pose()失败，请确认导航定位正常",
            "wait_done": "机器人反馈:执行wait({duration})完成",
            "set_cmdvel_done": "机器人反馈:执行set_cmdvel({linear_x},{linear_y},{angular_z},{duration})完成",
            "seewhat_done": "机器人反馈:执行seewhat()完成",
            "seewhat_func": "seewhat_func",
            "move_left_done": "机器人反馈:执行move_left({angle},{angular_speed})完成",
            "move_right_done": "机器人反馈:执行move_right({angle},{angular_speed})完成",
            "response_done": "机器人反馈：回复用户完成",
            "failure_execute_action_function_not_exists": "机器人反馈:动作函数不存在，无法执行",
            "failure_execute_action_error": "机器人反馈:动作执行异常，请查看日志",
            "finish": "finish",
            "finish_task": "f机器人反馈：执行跟随任务完成",
            "multiple_done": "机器人反馈：执行{actions}完成"
        }
        self._sensor_map = {
            '雷达': '/scan',
            '里程计': '/odom'
        }
        if self.monitor_camera:
            self._sensor_map['相机'] = '/camera/color/image_raw'
        self._last_missing = set()
        self._reported_missing = set()

    def init_ros_comunication(self):
        """
        初始化创建ros通信对象、函数 / Initialize creation of ROS communication objects and functions
        """
        # 创建速度话题发布者 
        self.publisher = self.create_publisher(Twist, self.Speed_topic, 10)
        # 创建动作执行服务器，用于接受动作列表，并执行动作 
        # 动作列表必须串行执行；唤醒回调使用独立回调组，确保仍可立即打断导航。
        self.action_callback_group = MutuallyExclusiveCallbackGroup()
        self.interrupt_callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, Progress, "action_service", self.execute_callback,
            callback_group=self.action_callback_group
        )
        # 创建执行动作状态发布者 
        self.actionstatus_pub = self.create_publisher(String, "actionstatus", 3)
        # 创建发布者，发布 seewhat_handle 话题 
        self.seewhat_handle_pub = self.create_publisher(String, "seewhat_handle", 1)
        # 创建打断状态订阅者
        self.wakeup = self.create_subscription(
            Int8, "awake_flag", self.wakeup_callback, 1,
            callback_group=self.interrupt_callback_group
        )

        # 创建tf监听者，监听坐标变换 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 图像话题订阅者
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 2
        )
        self._check_timer = self.create_timer(5.0, self._check_sensor_timer)
        
        
    
    def init_navigation_client(self):
        # 创建导航功能客户端，请求导航动作服务器 
        self.load_target_points()
        self.navclient = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.current_pose = PoseWithCovarianceStamped()
        self.record_pose = PoseStamped()
        # self.get_current_pose()
        #打断正在导航标志
        self.nav_runing = False 
        self.nav_status = False
        # 仅记住本进程中已成功完成的复合导航目标，避免按“附近点”误判来源区域。
        self.last_compound_target_symbol = None


    def load_target_points(self):
        """
        加载地图映射文件 /Load map mapping file
        """
        with open(self.map_mapping_config, "r") as file:
            target_points = yaml.safe_load(file)
        self.navpose_dict = {}
        for name, data in target_points.items():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = data["position"]["x"]
            pose.pose.position.y = data["position"]["y"]
            pose.pose.position.z = data["position"]["z"]
            pose.pose.orientation.x = data["orientation"]["x"]
            pose.pose.orientation.y = data["orientation"]["y"]
            pose.pose.orientation.z = data["orientation"]["z"]
            pose.pose.orientation.w = data["orientation"]["w"]
            self.navpose_dict[name] = pose


    def get_current_pose(self,name: str= "" ):
        """
        获取当前在全局地图坐标系下的位置与名称，并写入 map_mapping.yaml
        """
        # 1. 获取当前位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            msg = String(data=f"获取失败，请重新定位")
            self.text_pub.publish(msg)
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_failed")
            return

        # 2. 读取已有 YAML 或新建
        if os.path.isfile(self.map_mapping_config):
            with open(self.map_mapping_config, "r", encoding="utf-8") as f:
                target_points = yaml.safe_load(f) or {}
        else:
            target_points = {}
            os.makedirs(os.path.dirname(self.map_mapping_config), exist_ok=True)
        # 3. 检查是否有重复的名称
        name=name.strip('"\'')
        target_points = {k: v for k, v in target_points.items()
                 if v["name"].strip('"\'') != name}
        
        # 4.分配下一个字母键（防跳跃）
        next_key = self.next_point_key(target_points, name)
        if next_key is None:
            self.get_logger().error("Too many poses, ran out of letters A-Z!")
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_failed")
            return

        if not name:
            name = f"未命名{len(target_points)}"
        
        # 5. 组装数据结构
        target_points[next_key] = {
            "name": name,
            "position": {
                "x": float(transform.transform.translation.x),
                "y": float(transform.transform.translation.y),
                "z": 0.0,
            },
            "orientation": {
                "x": float(transform.transform.rotation.x),
                "y": float(transform.transform.rotation.y),
                "z": float(transform.transform.rotation.z),
                "w": float(transform.transform.rotation.w),
            },
        }

        # 6. 原子写入运行文件，并自动同步回源码，避免重新编译丢失点位。
        try:
            self.write_map_mapping(self.map_mapping_config, target_points)
        except OSError as e:
            self.get_logger().error(f"Write map_mapping.yaml failed: {e}")
            if not self.interrupt_flag:
                self.action_status_pub("get_current_pose_failed")
            return

        backup_failed = False
        if self.map_mapping_backup_path:
            runtime_path = os.path.realpath(self.map_mapping_config)
            backup_path = os.path.realpath(self.map_mapping_backup_path)
            if runtime_path != backup_path:
                try:
                    self.write_map_mapping(
                        self.map_mapping_backup_path, target_points
                    )
                    self.get_logger().info(
                        f"Point mapping automatically backed up to {backup_path}"
                    )
                except OSError as exc:
                    backup_failed = True
                    self.get_logger().error(
                        f"Point recorded, but source backup failed: {exc}"
                    )
        # 7. 打印日志
        self.get_logger().info(
            f"Recorded pose {next_key}: '{name}' -> \n"
            f"  position: x={target_points[next_key]['position']['x']:.2f}, "
            f"y={target_points[next_key]['position']['y']:.2f}, z=0.0\n"
        )

        if backup_failed:
            self.text_pub.publish(
                String(data="位置已记录，但源码备份失败，请查看终端")
            )

        if not self.interrupt_flag:
            self.action_status_pub("get_current_pose_success")

    @staticmethod
    def next_point_key(target_points, point_name=""):
        """固定常用语义键，其他名称从 H 开始分配。"""
        preferred_keys = {
            "a": "A",
            "b": "B",
            "c": "C",
            "d": "D",
            "桌子": "E",
            "箱子": "F",
            "沙发": "G",
        }
        used = {
            key for key in target_points
            if len(key) == 1 and key in string.ascii_uppercase
        }
        preferred_key = preferred_keys.get(point_name.casefold())
        if preferred_key and preferred_key not in used:
            return preferred_key

        reserved = set(preferred_keys.values())
        return next(
            (
                key for key in string.ascii_uppercase
                if key not in used and key not in reserved
            ),
            None,
        )

    @staticmethod
    def write_map_mapping(path, target_points):
        """原子写入真实目标文件；path 是符号链接时保留链接本身。"""
        actual_path = os.path.realpath(path)
        directory = os.path.dirname(actual_path)
        os.makedirs(directory, exist_ok=True)
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=directory,
                delete=False,
            ) as temp_file:
                temp_path = temp_file.name
                yaml.dump(
                    target_points,
                    temp_file,
                    allow_unicode=True,
                    sort_keys=False,
                    default_flow_style=False,
                )
                temp_file.flush()
                os.fsync(temp_file.fileno())
            os.replace(temp_path, actual_path)
            temp_path = None
        finally:
            if temp_path and os.path.exists(temp_path):
                os.unlink(temp_path)

    def action_status_pub(self, key, **kwargs):
        """
        动作结果发布方法
        :param key: 文本标识
        :param**kwargs: 占位符参数
        """
        text_template = self.feedback_largemoel_dict.get(key)

        if text_template is None:
            self.get_logger().error(f"Unknown action status key: {key}")
            message = f"机器人反馈:未知动作状态({key})"
            self.actionstatus_pub.publish(String(data=message))
            return

        try:
            message = text_template.format(**kwargs)
        except KeyError as e:
            self.get_logger().error(f"Translation placeholder error: {e} (key: {key})")
            message = f"[Translation failed: {key}]"
        # 发布消息
        self.actionstatus_pub.publish(String(data=message))
        self.get_logger().info(f"Published message: {message}")
        


    def navigation(self, point_name, report_success=True):
        """
        从navpose_dict字典中获取目标点坐标.并导航到目标点
        """
        # 1. 获取当前位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            msg = String(data=f"导航失败，请重新定位")
            self.text_pub.publish(msg)
            return False
        self.load_target_points()
        navigation_done = threading.Event()
        navigation_state = {
            "goal_handle": None,
            "cancel_requested": False,
            "succeeded": False,
        }
        point_name = point_name.strip("'\"")
        if point_name not in self.navpose_dict:
            self.get_logger().error(
                f"Target point '{point_name}' does not exist in the navigation dictionary."
            )
            self.action_status_pub(
                "navigation_3", point_name=point_name
            )  # 目标点地图映射中不存在
            return False
        # 获取目标点坐标 
        target_pose = self.navpose_dict.get(point_name)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        if not self.navclient.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("NavigateToPose action server is not available.")
            self.action_status_pub("navigation_4", point_name=point_name)
            return False
        send_goal_future = self.navclient.send_goal_async(goal_msg)

        def goal_response_callback(future):
            try:
                goal_handle = future.result()
            except Exception as exc:
                self.get_logger().error(f"Failed to send navigation goal: {exc}")
                self.action_status_pub("navigation_4", point_name=point_name)
                navigation_done.set()
                return

            navigation_state["goal_handle"] = goal_handle
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error("Goal was rejected!")
                self.action_status_pub("navigation_1", point_name=point_name)
                navigation_done.set()
                return

            if navigation_state["cancel_requested"] or self.interrupt_flag:
                navigation_state["cancel_requested"] = True
                goal_handle.cancel_goal_async()

            get_result_future = goal_handle.get_result_async()

            def result_callback(future_result):
                try:
                    result = future_result.result()
                    if navigation_state["cancel_requested"] or result.status == 5:
                        self.action_status_pub(
                            "navigation_5", point_name=point_name
                        )  # 执行导航取消
                    elif result.status == 4:
                        navigation_state["succeeded"] = True
                        if report_success:
                            self.action_status_pub(
                                "navigation_2", point_name=point_name
                            )  # 执行导航成功
                    else:
                        self.get_logger().info(
                            f"Navigation failed with status: {result.status}"
                        )
                        self.action_status_pub(
                            "navigation_4", point_name=point_name
                        )  # 执行导航失败
                except Exception as exc:
                    self.get_logger().error(f"Failed to get navigation result: {exc}")
                    self.action_status_pub("navigation_4", point_name=point_name)
                finally:
                    navigation_done.set()

            get_result_future.add_done_callback(result_callback)

        send_goal_future.add_done_callback(goal_response_callback)
        
        while not navigation_done.wait(timeout=0.1):
            if self.interrupt_flag:
                navigation_state["cancel_requested"] = True
                goal_handle = navigation_state["goal_handle"]
                if goal_handle is not None:
                    goal_handle.cancel_goal_async()
                break
        self.stop()
        return navigation_state["succeeded"]

    def navigation_sequence(self, *point_names):
        """依次导航多个点；任一中间点失败时不再发送后续目标。"""
        for index, point_name in enumerate(point_names):
            report_success = index == len(point_names) - 1
            if self.interrupt_flag or not self.navigation(
                point_name, report_success=report_success
            ):
                self.get_logger().warning(
                    f"Navigation sequence stopped before completing {point_names}"
                )
                return False
        return True

    def navigation_compound(self, target_symbol):
        """离开当前区域的入口点，再经目标入口到达最终点。"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().warning(f"Compound navigation TF lookup failed: {exc}")
            self.text_pub.publish(String(data="导航失败，请重新定位"))
            return False

        try:
            with open(self.map_mapping_config, "r", encoding="utf-8") as file:
                target_points = yaml.safe_load(file) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f"Read compound navigation points failed: {exc}")
            self.text_pub.publish(String(data="导航点位读取失败"))
            return False

        target_symbol = target_symbol.strip("'\"")
        source_symbol = self.last_compound_target_symbol
        if source_symbol is not None:
            source = target_points.get(source_symbol, {})
            source_position = source.get("position", {})
            try:
                source_distance = math.hypot(
                    float(source_position["x"])
                    - float(transform.transform.translation.x),
                    float(source_position["y"])
                    - float(transform.transform.translation.y),
                )
            except (KeyError, TypeError, ValueError):
                source_distance = float("inf")
            if source_distance > 0.6:
                self.get_logger().info(
                    "Robot moved away from the last compound destination; "
                    "skip its exit waypoint"
                )
                source_symbol = None
                self.last_compound_target_symbol = None

        route = compose_compound_route(
            target_points,
            target_symbol,
            source_symbol=source_symbol,
        )
        if not route:
            self.get_logger().error(
                f"Compound route unavailable for target {target_symbol}"
            )
            self.text_pub.publish(String(data="目标入口点未记录，无法开始导航"))
            return False

        route_names = [
            str(target_points[symbol].get("name", symbol)) for symbol in route
        ]
        self.get_logger().info(
            f"Compound navigation route: {' -> '.join(route_names)}"
        )
        succeeded = self.navigation_sequence(*route)
        self.last_compound_target_symbol = target_symbol if succeeded else None
        return succeeded
        

    def wait(self, duration):
        duration = float(duration)
        time.sleep(duration)
        if not self.interrupt_flag:
            self.action_status_pub("wait_done", duration=duration)


    def seewhat(self,func=None):
        self.save_single_image()
        if func is not None:
            msg = String(data=f'{func}')
        else :
            msg = String(data="seewhat")
        self.seewhat_handle_pub.publish(
            msg
        )  # 归一化，发布seewhat话题，由model_service调用大模型
        self.action_status_pub("seewhat_done")
            
    def set_cmdvel(self, linear_x, linear_y, angular_z, duration):  # 发布cmd_vel
        # 将参数从字符串转换为浮点数
        linear_x = float(linear_x)
        linear_y = float(linear_y)
        angular_z = float(angular_z)
        duration = float(duration)
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._execute_action(twist, durationtime=duration+0.3)
        if not self.interrupt_flag:
            self.action_status_pub(
                "set_cmdvel_done",
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=angular_z,
                duration=duration,
            )

    def move_left(self, angle, angular_speed):  # 左转x度
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)  # 将角度转换为弧度
        duration = abs(angle_rad / angular_speed)
        angular_speed = abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration+0.8)
        self.stop()
        if not self.interrupt_flag:
            self.action_status_pub(
                "move_left_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def move_right(self, angle, angular_speed):  # 右转x度
        angle = float(angle)
        angular_speed = float(angular_speed)
        angle_rad = math.radians(angle)  # 将角度转换为弧度
        duration = abs(angle_rad / angular_speed)
        angular_speed = -abs(angular_speed)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self._execute_action(twist, 1, duration+0.8)
        self.stop()
        if not self.interrupt_flag:
            self.action_status_pub(
                "move_right_done",
                angle=angle,
                angular_speed=angular_speed,
            )

    def stop(self):  # 停止
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def stop_follow(self):
        self.get_logger().info("stop procress.....")
        futures = [
            self.visual_follower_future,
            self.laser_follower_future,
            self.line_follower_future,
            self.KCF_follow_future,
        ]
        for future in futures:
            if not future.done():
                future.set_result(True)
        if self.interrupt_flag:
            return
        else:
            self.action_status_pub("finish_task")


    def cancel(self):
        cmd1 = "ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
        cmd2 = '''"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'''
        cmd = cmd1 +cmd2
        os.system(cmd)

    def slam_start(self):
        with self.process_lock:
            self.navigation_stop()
            if self.slam_process is not None and self.slam_process.poll() is None:
                self.get_logger().info("SLAM is already running.")
                return
            subprocess.run(['pkill', '-f', 'turn_on_wheeltec_robot'], check=False)
            time.sleep(0.5)
            subprocess.Popen(['ros2', 'launch', 'turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch.py'])
            time.sleep(1.0)
            self.slam_process = subprocess.Popen(
                ['ros2', 'run', 'slam_gmapping', 'slam_gmapping',
                 '--ros-args', '-p', 'use_sim_time:=false']
            )
            self.get_logger().info(f"SLAM started, pid={self.slam_process.pid}")

    def slam_stop(self):
        with self.process_lock:
            if self.slam_process is None or self.slam_process.poll() is not None:
                self.slam_process = None
                self.get_logger().info("SLAM is not running; skip map saving.")
                return
            subprocess.Popen(['ros2', 'launch', 'wheeltec_nav2', 'save_map.launch.py'])
            time.sleep(5)
            self.kill_process_tree(self.slam_process.pid)
            self.slam_process = None
            self.cancel()
            msg = String(data="建图结束,地图保存完成")
            self.text_pub.publish(msg)
            
    def navigation_start(self):
        with self.process_lock:
            self.slam_stop()
            if self.navigation_process is not None and self.navigation_process.poll() is None:
                self.get_logger().info("Navigation is already running.")
                return
            self.navigation_process = subprocess.Popen(
                ['ros2', 'launch', 'wheeltec_nav2', 'wheeltec_nav2_model.launch.py']
            )
            self.get_logger().info(f"Navigation started, pid={self.navigation_process.pid}")

    def navigation_stop(self):
        with self.process_lock:
            if self.navigation_process is None:
                return
            if self.navigation_process.poll() is None:
                self.kill_process_tree(self.navigation_process.pid)
            self.navigation_process = None
            self.cancel()
            self.get_logger().info("Navigation stopped.")

    def shutdown_background_processes(self):
        """退出节点时终止由大模型启动的后台任务，不触发保存地图。"""
        with self.process_lock:
            for process in (self.navigation_process, self.slam_process):
                if process is not None and process.poll() is None:
                    self.kill_process_tree(process.pid)
            self.navigation_process = None
            self.slam_process = None
        for future in (
            self.visual_follower_future,
            self.laser_follower_future,
            self.line_follower_future,
            self.KCF_follow_future,
        ):
            if not future.done():
                future.set_result(True)

    def KCF_follow(self,x1=0,y1=0,x2=0,y2=0):
        if x1==y1==x2==y2==0:
            self.seewhat('KCF_follow(x1,y1,x2,y2)')
        self.get_logger().info(f'kcf_follow: x1:{x1};y1:{y1};x2:{x2};y2:{y2}')
        scale = 640 / 1000 if self.multimodel.strip("'\"") == 'qwen3-vl-plus' else 1
        x1, y1, x2, y2 = (
                        int(round(int(v) * (scale if i % 2 == 0 else scale * 480 / 640)))
                        for i, v in enumerate((x1, y1, x2, y2))
                    )
        self.KCF_follow_future = Future() #复位Future对象 
        process_fuc = subprocess.Popen(['ros2', 'run', 'wheeltec_robot_kcf', 'run_tracker_model','--ros-args','-p',f'x1:={x1}','-p',f'y1:={y1}','-p',f'x2:={x2}','-p',f'y2:={y2}'])
        # time.sleep(1.0)#睡眠2秒等待线程稳定
        while not self.KCF_follow_future.done():
            if self.interrupt_flag:
                break
            time.sleep(0.1)
        self.kill_process_tree(process_fuc.pid)
        self.cancel()

    def visual_follower(self,color):
        try:
            self.visual_follower_future = Future() 
            color = color.strip("'\"")
            if color == 'red':
                target_color = int(0)
            elif color == 'green':
                target_color = int(1)
            elif color == 'blue':
                target_color = int(2)
            elif color == 'yellow':
                target_color = int(3)
            else:
                target_color = int(0)
            process_fuc1 = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'visualtracker','--ros-args','-p',f'target_color:={target_color}'])
            process_fuc2 = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'visualfollow'])
            while not self.visual_follower_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_fuc1.pid)
            self.kill_process_tree(process_fuc2.pid)
            self.cancel()
        except:
            self.get_logger().error('visual_follower Startup failure')
            return
        
    def laser_follower(self):
        self.laser_follower_future = Future()
        process_fuc = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'laserfollower'])
        # time.sleep(1.0)#睡眠2秒等待线程稳定

        while not self.laser_follower_future.done():
            if self.interrupt_flag:
                break
            time.sleep(0.1)

        self.kill_process_tree(process_fuc.pid)
        self.cancel()

        
    def line_follower(self,color):
        try:
            self.line_follower_future = Future() 
            color = color.strip("'\"")
            # self.get_logger().info(f'line_follower start!') 
            if color == 'red':
                target_color = int(0)
            elif color == 'green':
                target_color = int(1)
            elif color == 'blue':
                target_color = int(2)
            elif color == 'yellow':
                target_color = int(3)
            else:
                target_color = int(0)
            process_fuc = subprocess.Popen(['ros2', 'run', 'simple_follower_ros2', 'line_follow_model','--ros-args','-p',f'target_color:={target_color}'])
            while not self.line_follower_future.done():
                if self.interrupt_flag:
                    break
                time.sleep(0.1)
            self.get_logger().info(f'killed process_pid') 
            self.kill_process_tree(process_fuc.pid)
            self.cancel()     
        except:
            self.get_logger().error('line_follower Startup failure')
            return

    def _execute_action(self, twist, num=1, durationtime=3.0):
        for _ in range(num):
            start_time = time.time()
            count= 0
            while (time.time() - start_time) < durationtime:
                if self.obstacle_in_path(twist, self.obstacle_dist,self.obstacle_angle):
                    count += 1
                    if count >= 3:
                        twist.linear.x = 0.0
                        twist.linear.y = 0.0
                        twist.angular.z = 0.0
                        self.publisher.publish(twist)
                        msg = String(data=f"遇到障碍物,停止移动")
                        self.text_pub.publish(msg)
                        return
                if self.interrupt_flag:
                    self.stop()
                    return
                self.publisher.publish(twist)
                time.sleep(0.1)
                
    @staticmethod
    def obstacle_in_path(cmd: Twist, d: float, ang: float) -> bool:
        if d > LIDAR_RANGE:
            return False
        v = cmd.linear.x
        w = cmd.angular.z
        # 1. 前瞻距离（带符号）
        if abs(w) < 1e-3:                 # 直线
            along = v * PREDICT_TIME
            across = ROBOT_WIDTH
        else:                             # 圆弧
            r = v /(w+1e-3)                     # 转弯半径（带符号）
            along = r * math.sin(w * PREDICT_TIME)
            across = ROBOT_WIDTH + abs(r * (1 - math.cos(w * PREDICT_TIME)))
        # 2. 障碍物在车体坐标系
        x = d * math.cos(ang)
        y = d * math.sin(ang)
        # 3. 只挡“行驶方向同侧”
        x_in = (0 <= x <= along) if along >= 0 else (along <= x <= 0)
        y_in = abs(y) <= across
        # print(f"x_in:{x_in}, y_in:{y_in},x:{x},along:{along},y:{y},across:{across},d:{d},ang:{ang}",flush=True)
        return x_in and y_in

        
    @staticmethod
    def kill_process_tree(pid):
        try:
            parent = psutil.Process(pid)
            for child in parent.children(recursive=True):
                child.kill()
            parent.kill()
        except psutil.NoSuchProcess:
            pass

                
    # 核心程序，解析动作列表并执行  
    def execute_callback(self, goal_handle):
        feedback_msg = Progress.Feedback()
        actions = goal_handle.request.actions
        self.action_runing = True
        if not actions:  # 动作列表为空  
                self.action_status_pub("response_done")
        else:  
            for action in actions:
                time.sleep(1)
                if self.interrupt_flag:
                    break
                match = re.match(r"(\w+)\((.*)\)", action)
                if match is None:
                    self.get_logger().warning(
                        f"action_service: malformed action '{action}', skip execution"
                    )
                    self.action_status_pub(
                        "failure_execute_action_function_not_exists"
                    )
                    continue
                action_name, args_str = match.groups()
                args = [arg.strip() for arg in args_str.split(",")] if args_str else []

                if not hasattr(self, action_name):
                    self.get_logger().warning(
                        f"action_service: {action} is invalid action，skip execution" 
                    )
                    self.action_status_pub(
                        "failure_execute_action_function_not_exists"
                    )
                else:
                    method = getattr(self, action_name)
                    try:
                        method(*args)
                        feedback_msg.status = f"action service execute  {action}  successed"
                    except Exception as exc:
                        self.get_logger().error(
                            f"Action '{action}' execution failed: {exc}"
                        )
                        self.action_status_pub("failure_execute_action_error")
            
            if not self.interrupt_flag:
                self.action_status_pub(
                    "multiple_done", actions=actions
                )
                
        self.stop()  # 执行完全部动作停止机器人 
        self.action_runing = False  # 重置运行标志位  
        self.interrupt_flag = False
        goal_handle.succeed()
        result = Progress.Result()
        result.success = True
        return result

    def _check_sensor_timer(self):
        curr = {n for n, t in self._sensor_map.items() if not self.get_publishers_info_by_topic(t)}
        persistent_missing = self._last_missing & curr
        newly_reported = persistent_missing - self._reported_missing
        if newly_reported:
            msg = String()
            msg.data = '数据异常，请检查: ' + ', '.join(sorted(newly_reported))
            self.text_pub.publish(msg)
        # A recovered sensor is eligible for one new report if it fails again.
        self._reported_missing.intersection_update(curr)
        self._reported_missing.update(newly_reported)
        self._last_missing = curr


    def finishtask(self):  # 发布AI模型结束当前流程标志
        self.action_status_pub("finish")  # 结束当前任务

    def save_single_image(self):
        """
        保存一张图片 
        """
        self.IS_SAVING=True
        time.sleep(0.1)
        if self.image_msg is None:
            self.get_logger().warning("No image received yet.")  # 尚未接收到图像...
            return
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, "bgr8")
            # 保存图片
            cv2.imwrite(self.image_save_path, cv_image)

        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}")  # 保存图像时出错...
        self.IS_SAVING=False

    def display_saved_image(self):
        """
        显示已保存的图片4秒后关闭窗口 
        """
        try:
            img = cv2.imread(self.image_save_path)
            if img is not None:
                cv2.imshow("Saved Image", img)
                cv2.waitKey(4000)  # 等待4秒
                cv2.destroyAllWindows()
            else:
                self.get_logger().error(
                    "Failed to load saved image for display."
                )  # 加载保存的图像以供显示失败...
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {e}")  # 显示图像时出错...

    def image_callback(self, msg):  # 图像回调函数 
        if not self.IS_SAVING:
            self.image_msg = msg
        else:
            self.get_logger().error("The image is being saved and no new information will be accepted")

    def wakeup_callback(self, msg):
        if msg.data==1:
            self.interrupt_flag = True
            self.stop()
            self.stop_follow()
            time.sleep(1)
            self.interrupt_flag = False

    
    def distanceCallback(self, msg: Position):
        angle = msg.angle_x
        self.obstacle_angle = normalize_angle(angle)
        self.obstacle_dist = msg.distance
        # x=  abs(self.obstacle_dist) * math.cos(self.obstacle_angle)
        # y = abs(self.obstacle_dist) * math.sin(self.obstacle_angle)
        # print(f'angle:{angle}, dist:{self.obstacle_dist}', flush=True)
        # print(f'angle:{angle},obstacle_angle:{self.obstacle_angle},obstacle_dist,x:{x}, y:{y}', flush=True)
    

def main(args=None):
    rclpy.init(args=args)
    custom_action_server = CustomActionServer()
    
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(custom_action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        custom_action_server.stop()
        pass
    finally:
        custom_action_server.shutdown_background_processes()
        custom_action_server.stop()
        custom_action_server.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
