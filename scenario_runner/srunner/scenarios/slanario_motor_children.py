import py_trees
import carla
import math
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import KeepVelocity, StopVehicle
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest, Criterion
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.timer import GameTime

# 导入您定义的三个 Criteria（从 atomic_criteria.py）
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    EbikeDetectionAndDecelerateCriterion,
    PedestrianDetectionAndStopCriterion,
    ResumeAfterPedestrianCriterion
)


class EbikeAndPedestrianCross(BasicScenario):
    """
    场景描述：主车沿路线行驶，电瓶车和行人从侧面斜穿过马路。
    评估标准：
    1. 识别电瓶车并减速：25分
    2. 识别行人并刹车：50分
    3. 离开风险区并恢复通行：25分（到达终点即成功）
    
    参数配置（从XML读取）：
    - init_speed: 自车初始速度（km/h），默认40
    - difficulty: 难度等级（level1/level2/level3），默认level2
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        print("=" * 60)
        print("【场景初始化】EbikeAndPedestrianCross 场景正在加载...")
        print("=" * 60)
        
        # ========== 读取 XML 参数 ==========
        # 读取自车初始速度（单位：km/h）
        self._init_speed_kmh = float(
            config.other_parameters.get("init_speed", {}).get("value", 40.0)
        )
        # 转换为 m/s
        self._init_speed_ms = self._init_speed_kmh / 3.6
        
        # 读取难度等级
        self._difficulty = config.other_parameters.get("difficulty", {}).get("value", "level2")
        
        # 读取电瓶车速度（如果有配置）
        self._ebike_speed = float(
            config.other_parameters.get("ebike_speed", {}).get("value", 15.0)
        )
        
        # 读取行人速度（如果有配置）
        self._pedestrian_speed = float(
            config.other_parameters.get("pedestrian_speed", {}).get("value", 1.5)
        )
        
        trigger_transform = config.trigger_points[0]

        self._trigger_location = trigger_transform.location
        print(f"📊 参数配置:")
        print(f"  自车初始速度: {self._init_speed_kmh} km/h ({self._init_speed_ms:.2f} m/s)")
        print(f"  难度等级: {self._difficulty}")
        print(f"  电瓶车速度: {self._ebike_speed} m/s")
        print(f"  行人速度: {self._pedestrian_speed} m/s")
        
        # 根据难度设置刹车判定阈值
        if self._difficulty == "level1":
            self._brake_threshold = 0.10      # 刹车阈值降低，更容易成功
            self._stop_speed_threshold = 2.0  # 速度阈值提高
            self._min_stop_duration = 0.2     # 刹停时间要求降低
            self._max_response_time = 15.0    # 响应时间放宽
            print(f"  难度设置: 简单模式（刹车判定宽松）")
        elif self._difficulty == "level2":
            self._brake_threshold = 0.15
            self._stop_speed_threshold = 1.5
            self._min_stop_duration = 0.3
            self._max_response_time = 12.0
            print(f"  难度设置: 中等模式")
        else:  # level3
            self._brake_threshold = 0.20
            self._stop_speed_threshold = 1.0
            self._min_stop_duration = 0.5
            self._max_response_time = 10.0
            print(f"  难度设置: 困难模式（刹车判定严格）")
        
        self.timeout = timeout
        super(EbikeAndPedestrianCross, self).__init__(
            "EbikeAndPedestrianCross",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )
        
        # ========== 设置自车初始速度 ==========
        if self.ego_vehicles:
            ego = self.ego_vehicles[0]
            # 获取自车当前朝向
            yaw = math.radians(ego.get_transform().rotation.yaw)
            # 设置目标速度
            ego.set_target_velocity(carla.Vector3D(
                math.cos(yaw) * self._init_speed_ms,
                math.sin(yaw) * self._init_speed_ms
            ))
            print(f"🚗 自车初始速度已设置为: {self._init_speed_kmh} km/h")
        
        print(f"场景初始化完成，超时时间: {timeout}秒")
        print("=" * 60)

    def _initialize_actors(self, config):
        """
        根据 XML 配置自动生成 Actor
        """
        print("\n" + "=" * 60)
        print("【生成 Actor】开始生成电瓶车和行人...")
        print("=" * 60)
        print(f"XML 中配置的 other_actors 数量: {len(config.other_actors)}")
        
        for i, actor_conf in enumerate(config.other_actors):
            print(f"\n--- 第 {i+1} 个 Actor ---")
            print(f"模型: {actor_conf.model}")
            print(f"位置: x={actor_conf.transform.location.x}, y={actor_conf.transform.location.y}, z={actor_conf.transform.location.z}")
            print(f"朝向: yaw={actor_conf.transform.rotation.yaw}")
            
            actor = CarlaDataProvider.request_new_actor(actor_conf.model, actor_conf.transform)
            if actor:
                print(f"✅ 成功生成: {actor_conf.model} (ID: {actor.id})")
                self.other_actors.append(actor)
            else:
                print(f"❌ 警告: 生成失败 {actor_conf.model}")
        
        print(f"\n总共成功生成 {len(self.other_actors)} 个 Actor")
        if len(self.other_actors) >= 2:
            print(f"电瓶车: {self.other_actors[0].type_id if self.other_actors[0] else 'None'}")
            print(f"行人: {self.other_actors[1].type_id if self.other_actors[1] else 'None'}")
        print("=" * 60 + "\n")

    def _create_behavior(self):
        """
        封装原脚本中的核心控制逻辑。
        """
        print("\n" + "=" * 60)
        print("【创建行为树】开始创建电瓶车和行人的行为...")
        print("=" * 60)
        
        ego = self.ego_vehicles[0]
        ebike = self.other_actors[0] if len(self.other_actors) > 0 else None
        pedestrian = self.other_actors[1] if len(self.other_actors) > 1 else None
        
        print(f"自车位置: {ego.get_location()}")
        if ebike:
            print(f"电瓶车位置: {ebike.get_location()}")
        else:
            print("❌ 电瓶车不存在！")
        if pedestrian:
            print(f"行人位置: {pedestrian.get_location()}")
        else:
            print("❌ 行人不存在！")

        trigger_location = carla.Location(x=-25, y=-65, z=0.5)
        print(f"触发点位置: {self._trigger_location}")
        print(f"自车到触发点距离: {ego.get_location().distance(self._trigger_location):.2f}米")

        root = py_trees.composites.Parallel(
            "CrossBehavior", 
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )

        if pedestrian:
            # ==========================================
            # 1. 行人控制逻辑（使用XML配置的速度）
            # ==========================================
            ped_sequence = py_trees.composites.Sequence("PedestrianBehavior")
            ped_trigger = InTriggerDistanceToLocation(ego, self._trigger_location, distance=1.5)
            ped_walk = KeepVelocity(pedestrian, target_velocity=self._pedestrian_speed, duration=18.0)
            ped_stop = StopVehicle(pedestrian, 1.0)
            ped_sequence.add_children([ped_trigger, ped_walk, ped_stop])
            print(f"✅ 行人行为已添加: 速度{self._pedestrian_speed} m/s, 持续18秒")
        else:
            print("⚠️ 跳过行人行为（行人不存在）")

        if ebike:
            # ==========================================
            # 2. 电瓶车控制逻辑（使用XML配置的速度）
            # ==========================================
            bike_sequence = py_trees.composites.Sequence("EbikeBehavior")
            bike_trigger = InTriggerDistanceToLocation(ego, self._trigger_location, distance=1.5)
            bike_drive = KeepVelocity(ebike, target_velocity=self._ebike_speed, duration=10.0)
            bike_stop = StopVehicle(ebike, 1.0)
            bike_sequence.add_children([bike_trigger, bike_drive, bike_stop])
            print(f"✅ 电瓶车行为已添加: 速度{self._ebike_speed} m/s, 持续10秒")
        else:
            print("⚠️ 跳过电瓶车行为（电瓶车不存在）")

        if ebike and pedestrian:
            root.add_children([ped_sequence, bike_sequence])
            print("✅ 行为树创建完成，电瓶车和行人将同时移动")
        else:
            print("❌ 行为树创建失败：缺少电瓶车或行人")
        
        print("=" * 60 + "\n")
        return root

    def _create_test_criteria(self):
        """
        定义场景测试通过的标准 - 使用从 atomic_criteria.py 导入的类
        """
        print("\n" + "=" * 60)
        print("【创建评估准则】开始创建场景评估标准...")
        print("=" * 60)
        
        criteria = []
        
        ego = self.ego_vehicles[0]
        ebike = self.other_actors[0] if len(self.other_actors) > 0 else None
        pedestrian = self.other_actors[1] if len(self.other_actors) > 1 else None
        
        # 终点位置：使用 XML 中最后一个 waypoint
        goal_location = carla.Location(x=-49.379311, y=-34.194073, z=0.5)
        print(f"终点位置: {goal_location}")
        
        # ========== 1. 识别电瓶车并减速（25分） ==========
        if ebike:
            criteria.append(
                EbikeDetectionAndDecelerateCriterion(
                    actor=ego,
                    hazard_actor=ebike,
                    brake_threshold=0.05,
                    throttle_reduction=0.2,
                    min_decelerate_duration=0.3,
                    max_response_time=8.0,
                    terminate_on_failure=False
                )
            )
            print("✅ 电瓶车减速评估准则已添加 (25分)")
        else:
            print("⚠️ 跳过电瓶车减速评估准则（电瓶车不存在）")
        
        # ========== 2. 识别行人并刹车（50分）- 使用难度参数 ==========
        if pedestrian:
            criteria.append(
                PedestrianDetectionAndStopCriterion(
                    actor=ego,
                    hazard_actor=pedestrian,
                    brake_threshold=self._brake_threshold,
                    stop_speed_threshold=self._stop_speed_threshold,
                    min_stop_duration=self._min_stop_duration,
                    max_response_time=self._max_response_time,
                    terminate_on_failure=False
                )
            )
            print(f"✅ 行人刹车评估准则已添加 (50分) - 刹车阈值:{self._brake_threshold}, 速度阈值:{self._stop_speed_threshold}")
        else:
            print("⚠️ 跳过行人刹车评估准则（行人不存在）")
        
        # ========== 3. 离开风险区并恢复通行（25分） ==========
        criteria.append(
            ResumeAfterPedestrianCriterion(
                actor=ego,
                hazard_actor=pedestrian,
                goal_location=goal_location,
                goal_dist_threshold=50.0,
                min_resume_speed=0.0,
                terminate_on_failure=False
            )
        )
        print("✅ 恢复通行评估准则已添加 (25分)")
        
        # ========== 碰撞检测 ==========
        criteria.append(CollisionTest(ego))
        print("✅ 碰撞检测已添加")
        
        print(f"\n总共添加了 {len(criteria)} 个评估准则")
        print("=" * 60 + "\n")
        
        return criteria

    def __del__(self):
        """清理资源"""
        print("\n" + "=" * 60)
        print("【场景清理】正在清理资源...")
        print("=" * 60)
        self.remove_all_actors()
        print("✅ 清理完成")