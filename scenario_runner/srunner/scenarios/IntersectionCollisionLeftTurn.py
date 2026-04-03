import py_trees
import carla
import math
import numpy as np
# 导入 Scenario_Runner 的核心类
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (IntersectionCollisionLeftTurnBrakeCriterion, IntersectionCollisionLeftTurnResumeCriterion)


class EgoSpeedControl(py_trees.behaviour.Behaviour):
    """
    二合一节点：统一管理主车速度和接管检测。
    一旦检测到刹车/油门大于阈值，立即放弃速度控制并永久返回 SUCCESS。
    """
    def __init__(self, ego_vehicle, target_velocity=10.0, name="EgoSpeedControl"):
        super(EgoSpeedControl, self).__init__(name)
        self.ego_vehicle = ego_vehicle
        self.target_velocity = target_velocity
        # 核心：状态锁。记录是否已经被接管
        self._taken_over = False

    def update(self):
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            return py_trees.common.Status.FAILURE

        if self._taken_over:
            return py_trees.common.Status.SUCCESS

        control = self.ego_vehicle.get_control()

        if control.throttle > 0.001 or control.brake > 0.001:
            self._taken_over = True
            return py_trees.common.Status.SUCCESS

        # 3. 如果没被接管，继续保持目标速度
        # 使用 WaypointFollower 或 PID 控制速度
        velocity_vector = self.ego_vehicle.get_velocity()
        current_speed = math.sqrt(velocity_vector.x**2 + velocity_vector.y**2 + velocity_vector.z**2)

        # 简单 PID 控制
        # self.target_speed = self.target_speed
        throttle = np.clip((self.target_velocity - current_speed) * 0.5, 0.0, 1.0)
        control = carla.VehicleControl()
        control.throttle = throttle
        control.steer = 0.0
        self.ego_vehicle.apply_control(control)
        
        return py_trees.common.Status.RUNNING


class IntersectionCollisionLeftTurn(BasicScenario):
    """
    场景描述：自车在Route模式下以约 40km/h 直行。
    NPC从左侧进入路口并左转。若自车不避让，将在终点发生碰撞。
    """
    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):

        self._init_speed = float(
            config.other_parameters.get("init_speed", {}).get("value", 10.0)
        )
        self._npc_speed = float(
            config.other_parameters.get("npc_speed", {}).get("value", 11.0)
        )

        super(IntersectionCollisionLeftTurn, self).__init__(
            "IntersectionCollisionLeftTurn",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )

        self.scenario_type = "IntersectionCollisionLeftTurn"

        if self.ego_vehicles and self.ego_vehicles[0]:
            ego = self.ego_vehicles[0]
            transform = ego.get_transform()
            yaw = transform.rotation.yaw * (math.pi / 180)
            velocity = carla.Vector3D(
                math.cos(yaw) * self._init_speed,
                math.sin(yaw) * self._init_speed,
                0
            )
            ego.set_target_velocity(velocity)


    def _initialize_actors(self, config):
        """
        根据 XML 配置自动生成 Actor
        """
        
        # 使用 standard 模式从 config 生成 Actor，这能避免重复生成和物理冲突
        for actor_conf in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(actor_conf.model, actor_conf.transform)
            if actor:
                self.other_actors.append(actor)
            else:
                print(f"Warning: Failed to spawn {actor_conf.model}")

        for actor_conf in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(actor_conf.model, actor_conf.transform)
            if actor:
                # 开启灯光（推荐：示宽灯 + 近光灯）
                actor.set_light_state(carla.VehicleLightState(
                    carla.VehicleLightState.Position |
                    carla.VehicleLightState.LowBeam 
                ))

                self.other_actors.append(actor)
        if self.ego_vehicles:
            ego = self.ego_vehicles[0]
            ego.set_light_state(carla.VehicleLightState(
                carla.VehicleLightState.Position |
                carla.VehicleLightState.LowBeam 
            ))


    def _create_behavior(self):
        """
        构建行为树：手动计算圆弧路径，并强制投影为合法的 Waypoint
        """
        # ————————场景调节条件——————————
        # 初始化触发点
        self._trigger_npc_point = carla.Location(
            x=-40.0,
            y=205.1,
            z=4.0
        )

        # NPC 速度
        self._npc_speed = self._npc_speed

        # 碰撞点
        self._collision_location = carla.Location(
            x=43.2,
            y=205.3,
            z=0.5
        )
        # --------------------------------------------

        all_traffic_lights = CarlaDataProvider.get_world().get_actors().filter('*traffic_light*')
        for light in all_traffic_lights:
            # 锁定绿灯 100 秒
            light.set_state(carla.TrafficLightState.Green)
            light.set_green_time(1000.0)

        root = py_trees.composites.Parallel(
            "IntersectionCollisionLeftTurnBehavior",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )

        # 安全检查
        if not self.ego_vehicles or len(self.other_actors) < 1:
            return root

        ego = self.ego_vehicles[0]
        left_car = self.other_actors[0]

        # ================= 分支1: 主车控制逻辑 =================
        ego_control_node = EgoSpeedControl(ego, self._init_speed)
        root.add_child(ego_control_node)

        # ===============================
        # 2️⃣ NPC 行为
        # ===============================
        npc_sequence = py_trees.composites.Sequence("NPCSequence")

        # 触发条件
        trigger = InTriggerDistanceToLocation(
            ego,
            self._trigger_npc_point,
            distance=3.0,
            name="TriggerDistance"
        )

        # ===============================
        # 3️⃣ NPC 左转（WaypointFollower）
        # ===============================
        amap = CarlaDataProvider.get_map()

        entry_loc = carla.Location(x=24.5, y=185.4, z=0.5)
        end_loc = carla.Location(x=43.2, y=205.3, z=0.5)

        custom_plan = []
        # 左转
        # Bézier 曲线
        p0 = entry_loc
        p1 = carla.Location(x=24.5, y=205.3, z=0.5)
        p2 = end_loc

        for i in range(1, 40):
            t = i / 40.0
            x = (1-t)**2*p0.x + 2*(1-t)*t*p1.x + t**2*p2.x
            y = (1-t)**2*p0.y + 2*(1-t)*t*p1.y + t**2*p2.y

            loc = carla.Location(x=x, y=y, z=0.5)
            wp = amap.get_waypoint(loc, project_to_road=True)
            wp.transform.location = loc

            custom_plan.append((wp, 0))  # RoadOption.LEFT 也行
        
        # 到达终点后直行
        for j in range(1, 80):   # 延伸80米
            x = end_loc.x + j
            y = end_loc.y

            loc = carla.Location(x=x, y=y, z=0.5)
            wp = amap.get_waypoint(loc, project_to_road=True)
            wp.transform.location = loc

            custom_plan.append((wp, 4))  # LANEFOLLOW

        npc_move = WaypointFollower(
            left_car,
            self._npc_speed,
            plan=custom_plan
        )

        npc_sequence.add_child(trigger)
        npc_sequence.add_child(npc_move)   

        root.add_child(npc_sequence)
        return root

    def _create_test_criteria(self):
        criteria = []

        ego = self.ego_vehicles[0]
        # hazard = self.other_actors[0]   # 你的占道车辆
        goal_loc = carla.Location(x=43.2, y=205.3, z=0.5)

        criteria.append(
            IntersectionCollisionLeftTurnBrakeCriterion(
                actor=self.ego_vehicles[0],
                hazard_actor=self.other_actors[0],
                trigger_x=5.0,
                brake_threshold=0.2,
                min_brake_duration=0.3
            )
        )

        criteria.append(
            IntersectionCollisionLeftTurnResumeCriterion(
                actor=ego,
                goal_location=goal_loc,
                route_center_x=42,
                goal_dist_threshold=3.0,
                center_recover_threshold=2.0,
                min_resume_speed=1.0
            )
        )
        return criteria

    def __del__(self):
        """
        清理工作：移除生成的 Actor
        """
        self.remove_all_actors()

# =========================================================
# 动态注册场景到 Route 
# =========================================================
try:
    from srunner.scenarios import route_scenario
    route_scenario.IntersectionCollisionLeftTurn = IntersectionCollisionLeftTurn
except ImportError:
    print("Warning: Could not import route_scenario for dynamic registration.")