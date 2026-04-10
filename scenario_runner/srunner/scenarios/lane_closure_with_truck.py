import math
import py_trees
import carla
import inspect

from agents.navigation.local_planner import RoadOption

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
    DecelerationForConstructionTest,
    RoutePassCompletionTest,
    MinTTCTest
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenarios.basic_scenario import BasicScenario

# Helper function to convert location to GPS coordinates (copied from route_manipulation)
def _location_to_gps(lat_ref, lon_ref, location):
    """Convert from world coordinates to GPS coordinates"""
    EARTH_RADIUS_EQUA = 6378137.0
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx = scale * location.x * math.pi / 180.0
    my = location.y * math.pi / 180.0

    lat = lat_ref + mx
    lon = lon_ref + my / scale
    z = location.z  # height

    return {'lat': lat, 'lon': lon, 'z': z}

# Helper function to get GPS reference (copied from route_manipulation)
def _get_latlon_ref(world):
    """Get GPS reference from world"""
    import xml.etree.ElementTree as ET

    xodr = world.get_map().to_opendrive()
    tree = ET.ElementTree(ET.fromstring(xodr))

    # default reference
    lat_ref = 42.0
    lon_ref = 2.0

    for opendrive in tree.iter("OpenDRIVE"):
        for header in opendrive.iter("header"):
            for georef in header.iter("geoReference"):
                if georef.text:
                    geo_ref = georef.text
                    # Extract lat and lon from the geo reference string
                    # Format: +42.0+2.0/
                    if '+' in geo_ref:
                        parts = geo_ref.split('+')
                        if len(parts) >= 3:
                            try:
                                lat_ref = float(parts[1])
                                lon_ref = float(parts[2])
                            except (ValueError, IndexError):
                                pass

    return lat_ref, lon_ref


class LaneClosureWithTruck(BasicScenario):
    """
    场景描述:
    车道封闭与卡车场景
    - 自车初始速度：130 km/h
    - 施工区域：30-45米为锥桶区域
    - 障碍物：55米处有停止的卡车
    - 评分标准：
      1. 识别并减速：40分（在30-55米内减速至少40km/h）
      2. 避撞：50分（无碰撞）
      3. 无碰撞通过：10分（通过125米）
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout

        # 纵向距离配置
        self._cone_start_distance = 30.0
        self._cone_end_distance = 45.0
        self._truck_distance = 55.0
        self._lane_width = 3.5

        # 初始速度配置 - 从XML读取，默认130 km/h
        self._initial_speed_kph = 130.0
        if hasattr(config, 'other_parameters') and 'init_speed' in config.other_parameters:
            self._initial_speed_kph = float(config.other_parameters['init_speed'].get('value', 130))
            # print(f"[DEBUG] Initial speed loaded from XML: {self._initial_speed_kph} km/h", flush=True)

        # print(f"[DEBUG] LaneClosureWithTruck.__init__ called, initial_speed={self._initial_speed_kph} km/h", flush=True)
        super(LaneClosureWithTruck, self).__init__("LaneClosureWithTruck",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)
        # print(f"[DEBUG] LaneClosureWithTruck.__init__ completed", flush=True)

    def _initialize_actors(self, config):
        # =======================================================
        # 1. 先让车辆静置，等待物理引擎稳定
        # =======================================================
        ego_velocity = carla.Vector3D(0, 0, 0)
        self.ego_vehicles[0].set_target_velocity(ego_velocity)
        # print(f"[DEBUG] Vehicle spawned at 0 km/h, will set to {self._initial_speed_kph} km/h after actors spawn", flush=True)

        # =======================================================
        # 2. 将车辆往后移动50米，给更多加速时间（障碍物位置不变）
        #    同时扩展route范围，包含车辆新位置
        # =======================================================
        carla_map = CarlaDataProvider.get_map()
        ego_location = self.ego_vehicles[0].get_location()
        ego_wp = carla_map.get_waypoint(ego_location)

        # 保存原始位置，用于后续生成route
        original_location = ego_location

        if ego_wp:
            # 往后找50米的waypoint
            previous_wps = ego_wp.previous(50.0)
            if previous_wps:
                new_wp = previous_wps[0]
                new_location = new_wp.transform.location
                new_location.z = ego_location.z  # 保持原有高度

                # 移动车辆到新位置，并向右偏6.5度
                original_rotation = new_wp.transform.rotation
                right_turned_rotation = carla.Rotation(
                    pitch=original_rotation.pitch,
                    yaw=original_rotation.yaw + 6.5,  # 向右偏6.5度
                    roll=original_rotation.roll
                )
                new_transform = carla.Transform(new_location, right_turned_rotation)
                self.ego_vehicles[0].set_transform(new_transform)
                # print(f"[DEBUG] Vehicle moved 50m back to: {new_location}, yaw rotated 6.5° right", flush=True)

                # =======================================================
                # 扩展route，添加从新位置回溯到原位置的waypoints
                # =======================================================
                if hasattr(config, 'route') and config.route:
                    # print(f"[DEBUG] Extending route to include vehicle's new position", flush=True)
                    # print(f"[DEBUG] Original route length: {len(config.route)}", flush=True)

                    # 获取GPS reference
                    world = CarlaDataProvider.get_world()
                    lat_ref, lon_ref = _get_latlon_ref(world)

                    # 先找到RouteScenario实例，以便后续获取原始gps_route
                    route_scenario_instance = None
                    for frame_info in inspect.stack():
                        frame = frame_info.frame
                        if 'self' in frame.f_locals:
                            frame_self = frame.f_locals['self']
                            if frame_self.__class__.__name__ == 'RouteScenario':
                                route_scenario_instance = frame_self
                                break

                    if not route_scenario_instance:
                        # print("[DEBUG] WARNING: Could not find RouteScenario instance!", flush=True)
                        pass
                    else:
                        # print(f"[DEBUG] Found RouteScenario with gps_route length: {len(route_scenario_instance.gps_route)}", flush=True)
                        pass

                    # 生成从新位置向前到原位置的waypoints
                    extended_waypoints = []
                    extended_gps_waypoints = []  # 同时生成GPS waypoints
                    current_wp = new_wp

                    # 向前追踪，直到到达或超过原始位置
                    while current_wp:
                        extended_waypoints.append((current_wp.transform, RoadOption.STRAIGHT))

                        # 生成对应的GPS坐标
                        gps_coord = _location_to_gps(lat_ref, lon_ref, current_wp.transform.location)
                        extended_gps_waypoints.append((gps_coord, RoadOption.STRAIGHT))

                        # 检查是否已经到达或超过原始位置
                        dist_to_original = current_wp.transform.location.distance(original_location)
                        # print(f"[DEBUG] Added waypoint at distance {dist_to_original:.2f}m from original location", flush=True)
                        if dist_to_original < 5.0:  # 接近原始位置
                            break

                        # 向前移动一段距离
                        next_wps = current_wp.next(10.0)  # 每次前进10米
                        if next_wps:
                            current_wp = next_wps[0]
                        else:
                            break

                    # 将扩展的waypoints插入到route开头
                    if extended_waypoints:
                        # 获取route的其余部分
                        original_route = config.route[1:]  # 跳过第一个waypoint（起点）

                        # 从RouteScenario实例获取原始gps_route
                        if route_scenario_instance and hasattr(route_scenario_instance, 'gps_route'):
                            original_gps_route = route_scenario_instance.gps_route[1:]
                            # print(f"[DEBUG] Original gps_route length: {len(route_scenario_instance.gps_route)}", flush=True)
                        else:
                            original_gps_route = []

                        # 组合新route：扩展部分 + 原始route
                        config.route = extended_waypoints + original_route
                        new_gps_route = extended_gps_waypoints + original_gps_route
                        # print(f"[DEBUG] Route extended: added {len(extended_waypoints)} waypoints from vehicle position", flush=True)
                        # print(f"[DEBUG] New route length: {len(config.route)}", flush=True)
                        # print(f"[DEBUG] New GPS route length: {len(new_gps_route)}", flush=True)

                        # =======================================================
                        # 关键修复：截断route到合理长度
                        # =======================================================
                        # 场景要求：车辆从spawn位置行驶95m（过卡车40m）
                        # 车辆后退了50m，所以route需要：50 + 95 + 余量 = 200m
                        # 注意：interpolated waypoints 现在是每2米一个点
                        # 增加余量以确保RouteCompletionTest不会在到达95m之前触发
                        target_distance = 200.0  # meters (50后退 + 95场景要求 + 55余量)
                        hop_resolution = 2.0  # interpolated waypoints的密度（每2米一个）
                        target_waypoints_count = int(target_distance / hop_resolution) + 10  # 多留一点余量

                        # print(f"[DEBUG] Route truncation check: current={len(config.route)}, target={target_waypoints_count}", flush=True)
                        # print(f"[DEBUG] Current route length: {len(config.route)} waypoints (~{len(config.route) * hop_resolution:.1f}m)", flush=True)

                        if len(config.route) > target_waypoints_count:
                            # print(f"[DEBUG] Truncating route from {len(config.route)} to {target_waypoints_count} waypoints (~{target_distance}m)", flush=True)
                            config.route = config.route[:target_waypoints_count]
                            new_gps_route = new_gps_route[:target_waypoints_count]
                            # print(f"[DEBUG] Truncated route length: {len(config.route)}", flush=True)
                            # print(f"[DEBUG] Truncated GPS route length: {len(new_gps_route)}", flush=True)
                        else:
                            # print(f"[DEBUG] Route is shorter than target, no truncation needed", flush=True)
                            # print(f"[DEBUG] Keeping route length: {len(config.route)} waypoints", flush=True)
                            pass

                        # 更新RouteScenario实例的route和gps_route
                        if route_scenario_instance:
                            # print(f"[DEBUG] Updating RouteScenario route and gps_route", flush=True)
                            route_scenario_instance.route = config.route
                            route_scenario_instance.gps_route = new_gps_route
                            # print(f"[DEBUG] RouteScenario.route updated to length {len(route_scenario_instance.route)}", flush=True)
                            # print(f"[DEBUG] RouteScenario.gps_route updated to length {len(route_scenario_instance.gps_route)}", flush=True)

                            # =======================================================
                            # 关键：更新RouteScenario中可能已经创建的criteria实例
                            # =======================================================
                            # 检查RouteScenario是否已经有criteria_tree
                            if hasattr(route_scenario_instance, 'criteria_tree') and route_scenario_instance.criteria_tree:
                                # print(f"[DEBUG] Searching for criteria instances to update", flush=True)
                                # 遍历criteria_tree找到所有需要更新route的criteria
                                import py_trees
                                for child in route_scenario_instance.criteria_tree.iterate():
                                    class_name = child.__class__.__name__ if hasattr(child, '__class__') else ""

                                    # 更新InRouteTest
                                    if class_name == 'InRouteTest':
                                        # print(f"[DEBUG] Found InRouteTest instance, updating its route data", flush=True)
                                        child._route = config.route
                                        child._route_transforms, _ = zip(*config.route)
                                        child._route_length = len(config.route)
                                        # 重新计算accum_meters
                                        child._accum_meters = []
                                        prev_loc = child._route_transforms[0].location
                                        for i, tran in enumerate(child._route_transforms):
                                            loc = tran.location
                                            d = loc.distance(prev_loc)
                                            accum = 0 if i == 0 else child._accum_meters[i - 1]
                                            child._accum_meters.append(d + accum)
                                            prev_loc = loc
                                        # print(f"[DEBUG] InRouteTest updated: route_length={child._route_length}", flush=True)

                                    # 更新RouteCompletionTest
                                    elif class_name == 'RouteCompletionTest':
                                        # print(f"[DEBUG] Found RouteCompletionTest instance, updating its route data", flush=True)
                                        child._route = config.route
                                        child._route_length = len(config.route)
                                        child._route_transforms, _ = zip(*config.route)
                                        # 重新计算accumulated percentages
                                        child._route_accum_perc = child._get_acummulated_percentages()
                                        child.target_location = child._route_transforms[-1].location
                                        # print(f"[DEBUG] RouteCompletionTest updated: route_length={child._route_length}, target_location={child.target_location}", flush=True)

                                    # 更新OutsideRouteLanesTest（如果它也缓存了route）
                                    elif class_name == 'OutsideRouteLanesTest':
                                        # print(f"[DEBUG] Found OutsideRouteLanesTest instance, updating its route data", flush=True)
                                        child._route = config.route
                                        child._route_length = len(config.route)
                                        child._route_transforms, _ = zip(*config.route)
                                        # print(f"[DEBUG] OutsideRouteLanesTest updated: route_length={child._route_length}", flush=True)
                    else:
                        # print(f"[DEBUG] No extended waypoints generated!", flush=True)
                        pass
                else:
                    # print(f"[DEBUG] No route found in config! hasattr={hasattr(config, 'route')}", flush=True)
                    pass

        # =======================================================
        # 3. 获取地图与基准 Waypoint（障碍物位置保持不变）
        # =======================================================
        # base_location 保持原位置，不移动障碍物
        base_location = carla.Location(x=173.22, y=207.91, z=0.0)

        # 获取起点的 Waypoint (车道中心点数据)
        base_wp = carla_map.get_waypoint(base_location)

        # =======================================================
        # 3. 生成大货车 (使用 waypoint.next 顺着弯道往前找位置)
        # =======================================================
        truck_wps = base_wp.next(self._truck_distance)
        if truck_wps:
            truck_wp = truck_wps[0]  # 获取前方 55米 处【弯道上】的 Waypoint
            truck_transform = truck_wp.transform
            truck_right_vec = truck_transform.get_right_vector()

            # 卡车向右微调 0.5m
            truck_lateral_offset = 0.5
            truck_location = truck_transform.location + truck_right_vec * truck_lateral_offset
            truck_location.z += 0.2

            final_truck_transform = carla.Transform(truck_location, truck_transform.rotation)
            truck = CarlaDataProvider.request_new_actor("vehicle.carlamotors.carlacola", final_truck_transform)
            if truck:
                truck.set_simulate_physics(True)
                truck.set_light_state(carla.VehicleLightState.Special1)
                self.other_actors.append(truck)

        # =======================================================
        # 4. 生成斜向引导锥桶
        # =======================================================
        num_cones = 8
        cone_model = "static.prop.constructioncone"
        cone_start_offset = -7.0
        cone_end_offset = -3.3

        for i in range(num_cones):
            fraction = i / float(num_cones - 1)
            forward_dist = self._cone_start_distance + (self._cone_end_distance - self._cone_start_distance) * fraction
            lateral_offset = cone_start_offset + (cone_end_offset - cone_start_offset) * fraction

            # 顺着弯道往前推演 forward_dist 米
            cone_wps = base_wp.next(forward_dist)
            if cone_wps:
                cone_wp = cone_wps[0]
                cone_transform = cone_wp.transform
                cone_right_vec = cone_transform.get_right_vector()

                # 基于弯道当前的法向量（右向）进行偏移
                cone_location = cone_transform.location + cone_right_vec * lateral_offset
                cone_location.z += 0.2
                final_cone_transform = carla.Transform(cone_location, cone_transform.rotation)

                cone = CarlaDataProvider.request_new_actor(cone_model, final_cone_transform)
                if cone:
                    cone.set_simulate_physics(True)
                    self.other_actors.append(cone)

        # =======================================================
        # 5. 生成直排隔离锥桶 (沿着曲线车道线平行延伸)
        # =======================================================
        num_straight_cones = 12
        straight_start_dist = self._cone_end_distance
        straight_end_dist = self._cone_end_distance + 35.0
        straight_lateral_offset = cone_end_offset

        for i in range(1, num_straight_cones):
            fraction = i / float(num_straight_cones - 1)
            forward_dist = straight_start_dist + (straight_end_dist - straight_start_dist) * fraction

            cone_wps = base_wp.next(forward_dist)
            if cone_wps:
                cone_wp = cone_wps[0]
                cone_transform = cone_wp.transform
                cone_right_vec = cone_transform.get_right_vector()

                # 即使是弯道，它也会和当前的车道中心线严格保持 -3.3m 的平行距离！
                cone_location = cone_transform.location + cone_right_vec * straight_lateral_offset
                cone_location.z += 0.2
                final_cone_transform = carla.Transform(cone_location, cone_transform.rotation)

                cone = CarlaDataProvider.request_new_actor(cone_model, final_cone_transform)
                if cone:
                    cone.set_simulate_physics(True)
                    self.other_actors.append(cone)

        # =======================================================
        # 6. 在所有actors创建完成后，立即设置车辆初始速度到130 km/h
        # =======================================================
        # 使用多个连续waypoint来计算更准确的行驶方向，特别是对于弯曲道路
        carla_map = CarlaDataProvider.get_map()
        ego_location = self.ego_vehicles[0].get_location()
        ego_wp = carla_map.get_waypoint(ego_location)

        if ego_wp:
            # 向前追踪多个waypoint来获得更好的方向估计
            # 对于130km/h的高速，需要更远的lookahead距离
            lookahead_distance = 50.0  # 向前看50米
            next_wps = ego_wp.next(lookahead_distance)

            if len(next_wps) > 0:
                # 使用最远的前向waypoint来计算方向
                next_wp = next_wps[-1]  # 使用最后一个（最远的）waypoint
                direction_vector = next_wp.transform.location - ego_wp.transform.location

                # 只在水平面上计算方向
                direction_vector.z = 0
                direction = direction_vector / (direction_vector.length() + 1e-6)

                distance = direction_vector.length()
                # print(f"[DEBUG] Using {distance:.1f}m waypoint lookahead direction", flush=True)
                # print(f"[DEBUG] From {ego_wp.transform.location} to {next_wp.transform.location}", flush=True)
            else:
                # 如果无法获取前向waypoint，使用当前waypoint的forward向量
                direction = ego_wp.transform.get_forward_vector()
                direction.z = 0
                direction = direction / (direction.length() + 1e-6)
                # print(f"[DEBUG] Using current waypoint forward direction", flush=True)
        else:
            # 回退方案：使用车辆当前的transform
            ego_transform = self.ego_vehicles[0].get_transform()
            direction = ego_transform.get_forward_vector()
            direction.z = 0
            direction = direction / (direction.length() + 1e-6)
            # print(f"[DEBUG] Using vehicle transform forward direction", flush=True)

        target_speed_mps = self._initial_speed_kph / 3.6
        ego_velocity = carla.Vector3D(
            direction.x * target_speed_mps,
            direction.y * target_speed_mps,
            0.0  # z方向速度设为0
        )

        # 设置目标速度到 130 km/h
        self.ego_vehicles[0].set_target_velocity(ego_velocity)

        # print(f"[DEBUG] Initial velocity set to {self._initial_speed_kph} km/h", flush=True)
        # print(f"[DEBUG] Velocity direction: ({direction.x:.3f}, {direction.y:.3f}, {direction.z:.3f})", flush=True)

    def _create_behavior(self):
        """
        创建场景行为树：
        1. 持续控制车辆沿着waypoint行驶，直到距离卡车30米
        2. 然后让agent接管
        """
        # print("[DEBUG] _create_behavior called, building behavior tree", flush=True)
        root = py_trees.composites.Sequence("LaneClosureBehavior")

        # 1. 持续控制车辆沿着waypoint行驶
        # 注释掉 ContinuousWaypointFollower - 使用 DecelerationForConstructionTest 的加速逻辑代替
        # class ContinuousWaypointFollower(py_trees.behaviour.Behaviour):
        #     def __init__(self, vehicle, target_speed_kph, stop_distance, truck_actor, name="ContinuousWaypointFollower"):
        #         super(ContinuousWaypointFollower, self).__init__(name)
        #         self.vehicle = vehicle
        #         self.target_speed_kph = target_speed_kph
        #         self.target_speed_mps = target_speed_kph / 3.6
        #         self.stop_distance = stop_distance  # 距离卡车多少米停止控制
        #         self.truck_actor = truck_actor
        #         self._started = False
        #
        #         print(f"[DEBUG] ContinuousWaypointFollower.__init__ called, target_speed={target_speed_kph} km/h, stop_distance={stop_distance}m", flush=True)
        #
        #     def update(self):
        #         if not self._started:
        #             print(f"[DEBUG] ContinuousWaypointFollower started, controlling vehicle at {self.target_speed_kph} km/h", flush=True)
        #             self._started = True
        #
        #         # 计算到卡车的距离
        #         truck_location = self.truck_actor.get_location()
        #         ego_location = self.vehicle.get_location()
        #         distance_to_truck = ego_location.distance(truck_location)
        #
        #         # 如果距离卡车小于设定距离，停止控制，让agent接管
        #         if distance_to_truck <= self.stop_distance:
        #             print(f"[DEBUG] Reached {distance_to_truck:.1f}m from truck (stop_distance={self.stop_distance}m), releasing control to agent", flush=True)
        #             return py_trees.common.Status.SUCCESS
        #
        #         # 获取当前waypoint和前方waypoint
        #         carla_map = CarlaDataProvider.get_map()
        #         ego_wp = carla_map.get_waypoint(ego_location)
        #
        #         if ego_wp:
        #             # 向前看更远的waypoint以获得更好的方向预判
        #             # 对于130km/h的速度，需要更远的lookahead
        #             lookahead_distance = 30.0  # 100米lookahead
        #             next_wps = ego_wp.next(lookahead_distance)
        #
        #             if len(next_wps) > 0:
        #                 # 使用最远的waypoint来计算方向
        #                 next_wp = next_wps[-1]
        #                 direction_vector = next_wp.transform.location - ego_wp.transform.location
        #
        #                 # 只在水平面上计算方向
        #                 direction_vector.z = 0
        #                 direction = direction_vector / (direction_vector.length() + 1e-6)
        #
        #                 # 设置速度向量
        #                 ego_velocity = carla.Vector3D(
        #                     direction.x * self.target_speed_mps,
        #                     direction.y * self.target_speed_mps,
        #                     0.0
        #                 )
        #
        #                 # 设置目标速度
        #                 self.vehicle.set_target_velocity(ego_velocity)
        #
        #                 # 设置控制量
        #                 control = carla.VehicleControl()
        #                 control.throttle = 1.0
        #                 control.brake = 0.0
        #                 control.steer = 0.0
        #                 control.hand_brake = False
        #                 control.manual_gear_shift = False
        #                 self.vehicle.apply_control(control)
        #
        #         return py_trees.common.Status.RUNNING
        #
        #
        # waypoint_follower = ContinuousWaypointFollower(
        #     self.ego_vehicles[0],
        #     self._initial_speed_kph,
        #     50.0,  # 距离卡车30米时停止控制
        #     self.other_actors[0]  # 卡车是第一个actor
        # )
        # print("[DEBUG] Adding waypoint follower behavior", flush=True)
        # root.add_child(waypoint_follower)

        # 2. 显示测试信息
        class ShowTestInfo(py_trees.behaviour.Behaviour):
            def __init__(self, target_speed_kph, name="ShowTestInfo"):
                super(ShowTestInfo, self).__init__(name)
                self.target_speed_kph = target_speed_kph
                self._shown = False

            def update(self):
                if not self._shown:
                    print("\n" + "="*60)
                    print("Lane Closure with Truck - Test Information")
                    print("="*60)
                    print(f"Target Speed: {self.target_speed_kph} km/h")
                    print(f"Test Area: 30-95m (Cone zone at 30-45m, Truck at 55m)")
                    print(f"Scoring:")
                    print(f"  - Deceleration (90 pts): Reduce speed by 30+ km/h in 30-55m zone")
                    print(f"  - Collision Free (Gate): No collisions")
                    print(f"  - Route Pass (10 pts): Complete 95m route (pass truck + 40m)")
                    print("="*60 + "\n", flush=True)
                    self._shown = True

                return py_trees.common.Status.SUCCESS

        show_info = ShowTestInfo(self._initial_speed_kph)
        root.add_child(show_info)

        # 3. 运行测试直到完成
        # print("[DEBUG] Adding end_condition behavior (DriveDistance)", flush=True)
        # 场景在过卡车40m后结束：55m (truck) + 40m (buffer) = 95m
        end_distance = self._truck_distance + 40.0
        end_condition = DriveDistance(self.ego_vehicles[0], end_distance)
        root.add_child(end_condition)
        # print(f"[DEBUG] Scene will end at {end_distance}m (truck position + 40m)", flush=True)

        # print(f"[DEBUG] Behavior tree built successfully", flush=True)
        return root

    def _create_test_criteria(self):
        criteria = []

        # 1. CollisionTest - 检测与所有障碍物（锥桶、卡车）的碰撞
        # 使用 "miscellaneous" 来检测所有静态对象（包括锥桶）
        # 同时也会检测卡车碰撞
        collision_static_criterion = CollisionTest(
            self.ego_vehicles[0],
            other_actor_type="miscellaneous",
            terminate_on_failure=False,
            name="CollisionTest"
        )
        criteria.append(collision_static_criterion)

        # 为了确保也检测卡车碰撞，再添加一个针对车辆的 CollisionTest
        collision_vehicle_criterion = CollisionTest(
            self.ego_vehicles[0],
            other_actor=self.other_actors[0],  # 卡车
            terminate_on_failure=False,
            name="CollisionTest"
        )
        criteria.append(collision_vehicle_criterion)

        # 2. 最小TTC计算 - 用于Penalty评分
        ttc_criterion = MinTTCTest(
            self.ego_vehicles[0],
            target_actors=self.other_actors,  # 计算与所有障碍物的TTC
            name="MinTTCTest"
        )
        criteria.append(ttc_criterion)

        # 3. 减速检测 - 检测在30-55米区域内是否减速至少40km/h
        deceleration_criterion = DecelerationForConstructionTest(
            self.ego_vehicles[0],
            start_distance=self._cone_start_distance,  # 30米
            end_distance=self._truck_distance,          # 55米（卡车位置）
            initial_speed_kmh=self._initial_speed_kph,  # 130 km/h
            target_speed_reduction=40.0                 # 需要减速至少40km/h
        )
        criteria.append(deceleration_criterion)

        # 4. 路段通过检测 - 检测是否通过卡车并再行驶40米
        pass_completion_criterion = RoutePassCompletionTest(
            self.ego_vehicles[0],
            pass_distance=self._truck_distance + 40.0  # 55 + 40 = 95米
        )
        criteria.append(pass_completion_criterion)
        # print(f"[DEBUG] Route pass will be checked at {self._truck_distance + 40.0}m", flush=True)

        return criteria

    def __del__(self):
        self.remove_all_actors()
