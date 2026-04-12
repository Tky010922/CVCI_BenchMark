import math

import py_trees
import carla
import numpy as np

from agents.tools.misc import get_speed

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    WaypointFollower,
    LaneChange
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToVehicle,
    InTriggerDistanceToLocation,
    InTriggerRegion,
    DriveDistance
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import Criterion, WrongLaneTest


def _read_param(config, name, default_value, cast_type=float):
    if name not in config.other_parameters:
        return default_value
    return cast_type(config.other_parameters[name].get('value', default_value))


def _relative_coordinates(reference_transform, target_location):
    offset = target_location - reference_transform.location
    forward = reference_transform.get_forward_vector()
    right = reference_transform.get_right_vector()

    longitudinal = offset.x * forward.x + offset.y * forward.y
    lateral = offset.x * right.x + offset.y * right.y
    return longitudinal, lateral


def _get_trigger_location(config):
    if getattr(config, 'trigger_points', None) and config.trigger_points[0]:
        return config.trigger_points[0].location
    return None


class CutInBrakeResponseCriterion(Criterion):
    def __init__(
        self,
        actor,
        hazard_actor,
        trigger_distance=20.0,
        brake_threshold=0.15,
        speed_drop_ratio=0.25,
        min_brake_duration=0.2,
        max_response_time=4.0,
        lateral_limit=7.5,
        terminate_on_failure=False,
        name='CutInBrakeResponseCriterion'
    ):
        super(CutInBrakeResponseCriterion, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self.hazard_actor = hazard_actor
        self.trigger_distance = trigger_distance
        self.brake_threshold = brake_threshold
        self.speed_drop_ratio = speed_drop_ratio
        self.min_brake_duration = min_brake_duration
        self.max_response_time = max_response_time
        self.lateral_limit = lateral_limit

        self._activated = False
        self._start_time = None
        self._baseline_speed = None
        self._brake_start_time = None
        self.actual_value = 0
        self.success_value = 1

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if not self.actor or not self.hazard_actor:
            return new_status

        longitudinal, lateral = _relative_coordinates(self.actor.get_transform(), self.hazard_actor.get_location())
        if not self._activated and 0.0 < longitudinal <= self.trigger_distance and abs(lateral) <= self.lateral_limit:
            self._activated = True
            self._start_time = GameTime.get_time()
            self._baseline_speed = max(get_speed(self.actor), 0.1)

        if not self._activated:
            self.test_status = 'RUNNING'
            return new_status

        current_time = GameTime.get_time()
        current_speed = get_speed(self.actor)
        control = self.actor.get_control()

        if control.brake >= self.brake_threshold:
            if self._brake_start_time is None:
                self._brake_start_time = current_time
            if current_time - self._brake_start_time >= self.min_brake_duration:
                self.test_status = 'SUCCESS'
                self.actual_value = 1
                return py_trees.common.Status.SUCCESS
        else:
            self._brake_start_time = None

        speed_drop_ratio = (self._baseline_speed - current_speed) / max(self._baseline_speed, 0.1)
        if speed_drop_ratio >= self.speed_drop_ratio:
            self.test_status = 'SUCCESS'
            self.actual_value = 1
            return py_trees.common.Status.SUCCESS

        if current_time - self._start_time > self.max_response_time:
            self.test_status = 'FAILURE'
            self.actual_value = 0
            if self._terminate_on_failure:
                return py_trees.common.Status.FAILURE

        self.test_status = 'RUNNING'
        return new_status

    def terminate(self, new_status):
        if self.test_status != 'SUCCESS':
            self.test_status = 'FAILURE'
            self.actual_value = 0
        super(CutInBrakeResponseCriterion, self).terminate(new_status)


class CutInSafeRecoveryCriterion(Criterion):
    def __init__(
        self,
        actor,
        hazard_actor,
        route_end_location=None,
        pass_distance=10.0,
        min_speed=8.0,
        max_speed=22.0,
        lane_center_tolerance=1.75,
        goal_distance_threshold=15.0,
        terminate_on_failure=False,
        name='CutInSafeRecoveryCriterion'
    ):
        super(CutInSafeRecoveryCriterion, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self.hazard_actor = hazard_actor
        self.route_end_location = route_end_location
        self.pass_distance = pass_distance
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.lane_center_tolerance = lane_center_tolerance
        self.goal_distance_threshold = goal_distance_threshold
        self._map = CarlaDataProvider.get_map()
        self.actual_value = 0
        self.success_value = 1

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if not self.actor or not self.hazard_actor:
            return new_status

        actor_location = self.actor.get_location()
        actor_waypoint = self._map.get_waypoint(
            actor_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        if actor_waypoint is None:
            self.test_status = 'RUNNING'
            return new_status

        longitudinal, _ = _relative_coordinates(self.actor.get_transform(), self.hazard_actor.get_location())
        current_speed = get_speed(self.actor)
        lane_center_error = actor_location.distance(actor_waypoint.transform.location)
        reached_goal_zone = False

        if self.route_end_location is not None:
            reached_goal_zone = actor_location.distance(self.route_end_location) <= self.goal_distance_threshold

        has_recovered = longitudinal < -self.pass_distance and lane_center_error <= self.lane_center_tolerance
        has_safe_speed = self.min_speed <= current_speed <= self.max_speed

        if (has_recovered or reached_goal_zone) and has_safe_speed:
            self.test_status = 'SUCCESS'
            self.actual_value = 1
            return py_trees.common.Status.SUCCESS

        self.test_status = 'RUNNING'
        return new_status

    def terminate(self, new_status):
        if self.test_status != 'SUCCESS':
            self.test_status = 'FAILURE'
            self.actual_value = 0
        super(CutInSafeRecoveryCriterion, self).terminate(new_status)

# class EgoSpeedControl(py_trees.behaviour.Behaviour):
#     """
#     二合一节点：管理主车速度。若检测到人为接管（刹车/油门 > 0.05），则停止控制并返回 SUCCESS。
#     """
#     def __init__(self, ego_vehicle, target_velocity=10.0, name="EgoSpeedControl"):
#         super(EgoSpeedControl, self).__init__(name)
#         self.ego_vehicle = ego_vehicle
#         self.target_velocity = target_velocity
#         self._taken_over = False

#     def update(self):
#         if not self.ego_vehicle or not self.ego_vehicle.is_alive:
#             return py_trees.common.Status.FAILURE
        
#         if self._taken_over:
#             return py_trees.common.Status.SUCCESS

#         control = self.ego_vehicle.get_control()
#         if control.throttle > 0.05 or control.brake > 0.05:
#             self._taken_over = True
#             return py_trees.common.Status.SUCCESS

#         # 保持目标速度
#         transform = self.ego_vehicle.get_transform()
#         yaw = math.radians(transform.rotation.yaw)
#         velocity = carla.Vector3D(math.cos(yaw) * self.target_velocity, 
#                                   math.sin(yaw) * self.target_velocity, 0)
#         self.ego_vehicle.set_target_velocity(velocity)
#         return py_trees.common.Status.RUNNING

class EgoSpeedControl(py_trees.behaviour.Behaviour):
    """
    主车速度保持节点（适合有坡度）
    - 未接管时：仅做纵向速度保持
    - 检测到人工输入后：永久放权并返回 SUCCESS
    - 不使用 set_target_velocity
    - 不修改 steer，避免覆盖人工转向/route 跟踪
    """
    def __init__(
        self,
        ego_vehicle,
        target_speed=10.0,
        throttle_gain=0.20,
        brake_gain=0.10,
        max_throttle=0.4,
        max_brake=0.50,
        takeover_steer_threshold=0.02,
        takeover_throttle_threshold=0.02,
        takeover_brake_threshold=0.02,
        name="EgoSpeedControl"
    ):
        super(EgoSpeedControl, self).__init__(name)
        self.ego_vehicle = ego_vehicle
        self.target_speed = target_speed

        self.throttle_gain = throttle_gain
        self.brake_gain = brake_gain
        self.max_throttle = max_throttle
        self.max_brake = max_brake

        self.takeover_steer_threshold = takeover_steer_threshold
        self.takeover_throttle_threshold = takeover_throttle_threshold
        self.takeover_brake_threshold = takeover_brake_threshold

        self._taken_over = False

    def _get_speed(self):
        v = self.ego_vehicle.get_velocity()
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def update(self):
        if not self.ego_vehicle or not self.ego_vehicle.is_alive:
            return py_trees.common.Status.FAILURE

        if self._taken_over:
            return py_trees.common.Status.SUCCESS

        current_control = self.ego_vehicle.get_control()

        # 接管判定：方向/油门/刹车任一超过阈值，立即放权
        # 注意：这仍然不是最理想的“原始输入检测”，但比只看 throttle/brake 好很多
        if (
            abs(current_control.steer) > self.takeover_steer_threshold or
            current_control.throttle > self.takeover_throttle_threshold or
            current_control.brake > self.takeover_brake_threshold
        ):
            print("[EgoSpeedControl] Manual takeover detected, release control.")
            self._taken_over = True
            return py_trees.common.Status.SUCCESS

        current_speed = self._get_speed()
        speed_error = self.target_speed - current_speed

        new_control = carla.VehicleControl()

        # 不碰方向，避免覆盖人工/上层横向控制
        new_control.steer = current_control.steer
        new_control.hand_brake = False
        new_control.reverse = False
        new_control.manual_gear_shift = False

        if speed_error >= 0.0:
            # 速度偏低：补油
            throttle_cmd = np.clip(speed_error * self.throttle_gain, 0.0, self.max_throttle)
            new_control.throttle = float(throttle_cmd)
            new_control.brake = 0.0
        else:
            # 速度偏高：轻刹
            brake_cmd = np.clip((-speed_error) * self.brake_gain, 0.0, self.max_brake)
            new_control.throttle = 0.0
            new_control.brake = float(brake_cmd)

        self.ego_vehicle.apply_control(new_control)
        return py_trees.common.Status.RUNNING

class MinTTCAutoCriterion(Criterion):
    def __init__(
        self,
        actor,
        other_actors,
        distance_threshold=40.0,
        forward_angle_deg=140.0,
        safety_buffer=4.0,
        terminate_on_failure=False,
        name='MinTTCAutoCriterion'
    ):
        super(MinTTCAutoCriterion, self).__init__(name, actor, terminate_on_failure=terminate_on_failure)
        self.other_actors = other_actors
        self.distance_threshold = distance_threshold
        self.forward_angle_deg = forward_angle_deg
        self.safety_buffer = safety_buffer
        self.actual_value = float('inf')
        self.success_value = 2.0
        self.units = 's'

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if not self.actor:
            return new_status

        ego_location = self.actor.get_location()
        ego_transform = self.actor.get_transform()
        ego_velocity = self.actor.get_velocity()
        ego_forward = ego_transform.get_forward_vector()
        ego_forward_norm = math.sqrt(ego_forward.x * ego_forward.x + ego_forward.y * ego_forward.y)

        min_ttc = self.actual_value
        for other_actor in self.other_actors:
            if not other_actor or not other_actor.is_alive or other_actor.id == self.actor.id:
                continue

            other_location = other_actor.get_location()
            offset_x = other_location.x - ego_location.x
            offset_y = other_location.y - ego_location.y
            distance = math.sqrt(offset_x * offset_x + offset_y * offset_y)
            if distance <= 0.1 or distance > self.distance_threshold:
                continue

            rel_norm = math.sqrt(offset_x * offset_x + offset_y * offset_y)
            cosine = ((ego_forward.x * offset_x) + (ego_forward.y * offset_y)) / max(ego_forward_norm * rel_norm, 1e-6)
            cosine = max(min(cosine, 1.0), -1.0)
            angle_deg = math.degrees(math.acos(cosine))
            if angle_deg > self.forward_angle_deg / 2.0:
                continue

            rel_dir_x = offset_x / rel_norm
            rel_dir_y = offset_y / rel_norm
            other_velocity = other_actor.get_velocity()
            closing_speed = (
                (ego_velocity.x - other_velocity.x) * rel_dir_x +
                (ego_velocity.y - other_velocity.y) * rel_dir_y
            )
            if closing_speed <= 0.05:
                continue

            ttc = max(distance - self.safety_buffer, 0.1) / closing_speed
            min_ttc = min(min_ttc, ttc)

        self.actual_value = min_ttc
        self.test_status = 'SUCCESS'
        return new_status



class CutInCollision(BasicScenario):
    """
    场景：自车直行，右侧一辆车辆保持直行，另一辆车辆从右侧切入自车车道。
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=150):

        self.timeout = timeout
        self._trigger_location = _get_trigger_location(config)

        # 场景行为参数
        self._trigger_distance = _read_param(config, 'trigger_distance', 35.0)
        self._trigger_region_x = _read_param(config, 'trigger_region_x', 0.0)
        self._trigger_region_y = _read_param(config, 'trigger_region_y', 0.0)
        self._cutin_ready_distance = _read_param(config, 'cutin_ready_distance', 12.0)
        self._spawn_ahead_distance = _read_param(config, 'spawn_ahead_distance', 12.0)
        self._straight_speed = _read_param(config, 'straight_speed', 25.0)
        self._cutin_initial_speed = _read_param(config, 'cutin_initial_speed', 10.0)
        self._cutin_lane_change_speed = _read_param(config, 'cutin_lane_change_speed', 16.0)
        self._distance_same_lane = _read_param(config, 'distance_same_lane', 3.0)
        self._distance_other_lane = _read_param(config, 'distance_other_lane', 30.0)
        self._distance_lane_change = _read_param(config, 'distance_lane_change', 10.0)
        self._end_distance = _read_param(config, 'end_distance', 150.0)
        self._brake_trigger_distance = _read_param(config, 'brake_trigger_distance', 20.0)
        self._brake_threshold = _read_param(config, 'brake_threshold', 0.15)
        self._speed_drop_ratio = _read_param(config, 'speed_drop_ratio', 0.25)
        self._response_timeout = _read_param(config, 'response_timeout', 4.0)
        self._pass_distance = _read_param(config, 'pass_distance', 10.0)
        self._recovery_min_speed = _read_param(config, 'recovery_min_speed', 8.0)
        self._recovery_max_speed = _read_param(config, 'recovery_max_speed', 22.0)
        self._lane_center_tolerance = _read_param(config, 'lane_center_tolerance', 1.75)
        self._goal_distance_threshold = _read_param(config, 'goal_distance_threshold', 15.0)
        self._route_end_location = None

        if getattr(config, 'route', None):
            self._route_end_location = config.route[-1][0].location

        super(CutInCollision, self).__init__("CutInCollision",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode=debug_mode,
                                                   criteria_enable=criteria_enable)
        if self.ego_vehicles:
            ego = self.ego_vehicles[0]
            yaw = math.radians(ego.get_transform().rotation.yaw)
            ego.set_target_velocity(carla.Vector3D(math.cos(yaw) * (self._straight_speed-5), math.sin(yaw) * (self._straight_speed-5)))

    def _create_space_trigger(self, ego, cutin_vehicle):
        if self._trigger_location is not None:
            if self._trigger_region_x > 0.0 and self._trigger_region_y > 0.0:
                return InTriggerRegion(
                    ego,
                    self._trigger_location.x - self._trigger_region_x,
                    self._trigger_location.x + self._trigger_region_x,
                    self._trigger_location.y - self._trigger_region_y,
                    self._trigger_location.y + self._trigger_region_y,
                    name="Trigger_CutInRegion"
                )

            return InTriggerDistanceToLocation(
                ego,
                self._trigger_location,
                self._trigger_distance,
                name="Trigger_CutInLocation"
            )

        return InTriggerDistanceToVehicle(
            cutin_vehicle,
            ego,
            distance=self._trigger_distance,
            name="Trigger_CutInStart"
        )

    def _initialize_actors(self, config):
        for actor_index, actor_conf in enumerate(config.other_actors):
            transform = actor_conf.transform
            if actor_index == 0:
                transform = self._spawn_transform_right_near_ego(transform)

            vehicle = CarlaDataProvider.request_new_actor(
                actor_conf.model,
                transform,
                rolename='scenario'
            )
            if vehicle is not None:
                self.other_actors.append(vehicle)
                vehicle.set_autopilot(False)
                if actor_index == 0:
                    self._set_initial_forward_speed(vehicle, transform, speed_mps=self._cutin_initial_speed)
            else:
                print(f"Spawn failed: {actor_conf.model} @ {transform.location}")



    def _spawn_transform_right_near_ego(self, transform):
        if not self.ego_vehicles:
            return transform

        ego_location = CarlaDataProvider.get_location(self.ego_vehicles[0])
        if ego_location is None:
            return transform

        world_map = CarlaDataProvider.get_map()
        reference_wp = world_map.get_waypoint(
            ego_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        if reference_wp is None:
            return transform

        right_wp = reference_wp.get_right_lane()
        while right_wp is not None and right_wp.lane_type != carla.LaneType.Driving:
            right_wp = right_wp.get_right_lane()

        if right_wp is None:
            return transform

        near_ahead = right_wp.next(self._spawn_ahead_distance)
        spawn_wp = near_ahead[0] if near_ahead else right_wp

        adjusted = carla.Transform(spawn_wp.transform.location, spawn_wp.transform.rotation)
        adjusted.location.z = max(transform.location.z, spawn_wp.transform.location.z + 0.2)
        return adjusted

    def _set_initial_forward_speed(self, vehicle, transform, speed_mps=6.0):
        forward = transform.get_forward_vector()
        vehicle.set_target_velocity(carla.Vector3D(
            x=forward.x * speed_mps,
            y=forward.y * speed_mps,
            z=0.0
        ))

    def _create_behavior(self):
        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
            name="root"
        )
        root_1 = py_trees.composites.Sequence(name="CutInCollisionBehavior")

        if not self.ego_vehicles or len(self.other_actors) < 2:
            return root_1

        ego = self.ego_vehicles[0]
        cutin_vehicle = self.other_actors[0]
        straight_vehicle = self.other_actors[1]

        trigger = self._create_space_trigger(ego, cutin_vehicle)

        main_actions = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
            name="MainActions"
        )

        # 2. 直行车辆保持本车道行驶
        main_actions.add_child(WaypointFollower(
            straight_vehicle,
            target_speed=self._straight_speed,
            name="Straight_KeepLane"
        ))

        # 3. 切入车辆行为序列
        cutin_seq = py_trees.composites.Sequence(name="CutIn_Sequence")

        phase1 = py_trees.composites.Parallel(
            "CutIn_Phase1_Drive",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        phase1.add_child(WaypointFollower(
            cutin_vehicle,
            target_speed=self._cutin_initial_speed,
            name="CutIn_Phase1_Straight"
        ))
        phase1.add_child(InTriggerDistanceToVehicle(
            cutin_vehicle,
            ego,
            distance=self._cutin_ready_distance,
            name="CutIn_Ready_LaneChange"
        ))
        cutin_seq.add_child(phase1)

        phase2_cutin = LaneChange(
            cutin_vehicle,
            speed=self._cutin_lane_change_speed,
            direction='left',
            distance_same_lane=self._distance_same_lane,
            distance_other_lane=self._distance_other_lane,
            distance_lane_change=self._distance_lane_change,
            lane_changes=1,
            name="CutIn_Phase2_LaneChange"
        )
        cutin_seq.add_child(phase2_cutin)

        phase3_continue = WaypointFollower(
            cutin_vehicle,
            target_speed=self._cutin_lane_change_speed,
            name="CutIn_Phase3_Continue"
        )
        cutin_seq.add_child(phase3_continue)

        main_actions.add_child(cutin_seq)

        # 4. 自车完成预设行驶距离后结束场景
        end_cond = DriveDistance(
            ego,
            distance=self._end_distance,
            name="End_Distance"
        )
        main_actions.add_child(end_cond)

        root_1.add_child(trigger)
        root_1.add_child(main_actions)
        root.add_child(EgoSpeedControl(ego, target_speed=self._straight_speed))
        root.add_child(root_1)
        return root

    def _create_test_criteria(self):
        if not self.ego_vehicles or len(self.other_actors) < 2:
            return []

        ego = self.ego_vehicles[0]
        cutin_vehicle = self.other_actors[0]
        straight_vehicle = self.other_actors[1]

        return [
            CutInBrakeResponseCriterion(
                actor=ego,
                hazard_actor=cutin_vehicle,
                trigger_distance=self._brake_trigger_distance,
                brake_threshold=self._brake_threshold,
                speed_drop_ratio=self._speed_drop_ratio,
                max_response_time=self._response_timeout,
            ),
            CutInSafeRecoveryCriterion(
                actor=ego,
                hazard_actor=cutin_vehicle,
                route_end_location=self._route_end_location,
                pass_distance=self._pass_distance,
                min_speed=self._recovery_min_speed,
                max_speed=self._recovery_max_speed,
                lane_center_tolerance=self._lane_center_tolerance,
                goal_distance_threshold=self._goal_distance_threshold,
            ),
            MinTTCAutoCriterion(
                actor=ego,
                other_actors=[straight_vehicle, cutin_vehicle],
                distance_threshold=max(self._trigger_distance + 10.0, 40.0),
                forward_angle_deg=140.0,
            ),
            WrongLaneTest(ego),
        ]

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()