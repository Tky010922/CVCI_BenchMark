import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CollisionTest,
    DecelerationForConstructionTest,
    RoutePassCompletionTest,
    MinTTCTest
)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.traffic_events import TrafficEventType


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

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True, timeout=60):
        self.timeout = timeout

        # 1. 从 XML 动态读取参数（自带默认值兜底）
        params = getattr(config, 'other_parameters', {})
        self._initial_speed_kph = float(params.get('init_speed', {}).get('value', 130.0))
        self._truck_distance    = float(params.get('truck_distance', {}).get('value', 55.0))
        self._truck_offset      = float(params.get('truck_lateral_offset', {}).get('value', 0.5))
        self._cone_start_dist   = float(params.get('cone_start_distance', {}).get('value', 30.0))
        self._cone_end_dist     = float(params.get('cone_end_distance', {}).get('value', 45.0))
        self._cone_start_off    = float(params.get('cone_start_offset', {}).get('value', -7.0))
        self._cone_end_off      = float(params.get('cone_end_offset', {}).get('value', -3.3))

        super(LaneClosureWithTruck, self).__init__(
            "LaneClosureWithTruck", ego_vehicles, config, world, debug_mode, criteria_enable=criteria_enable)

    def _spawn_actor_on_route(self, base_wp, forward_dist, lateral_offset, model, is_vehicle=False):
        """通用辅助函数：沿着弯道前方生成 Actor"""
        wps = base_wp.next(forward_dist)
        if not wps: return None

        target_wp = wps[0]
        transform = target_wp.transform
        # 基于当前航向的右侧向量进行横向平移，完美适配弯道
        location = transform.location + transform.get_right_vector() * lateral_offset
        location.z += 0.2

        actor = CarlaDataProvider.request_new_actor(model, carla.Transform(location, transform.rotation))
        if actor:
            actor.set_simulate_physics(True)
            if is_vehicle:
                actor.set_light_state(carla.VehicleLightState.Special1)
            self.other_actors.append(actor)
        return actor

    def _initialize_actors(self, config):
        # =======================================================
        # 1. 先让车辆静置，等待物理引擎稳定
        # =======================================================
        ego_velocity = carla.Vector3D(0, 0, 0)
        self.ego_vehicles[0].set_target_velocity(ego_velocity)

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

                # =======================================================
                # 扩展route，添加从新位置回溯到原位置的waypoints
                # =======================================================
                if hasattr(config, 'route') and config.route:
                    # 获取GPS reference
                    world = CarlaDataProvider.get_world()
                    lat_ref, lon_ref = _get_latlon_ref(world)

                    # 先找到RouteScenario实例
                    route_scenario_instance = None
                    for frame in inspect.stack():
                        frame_self = frame.frame.f_locals.get('self')
                        if frame_self and frame_self.__class__.__name__ == 'RouteScenario':
                            route_scenario_instance = frame_self
                            break

                    # 生成从新位置向前到原位置的waypoints
                    extended_waypoints = []
                    extended_gps_waypoints = []
                    current_wp = new_wp

                    # 向前追踪，直到到达或超过原始位置
                    max_iterations = 100
                    iteration = 0
                    while current_wp and iteration < max_iterations:
                        iteration += 1
                        extended_waypoints.append((current_wp.transform, RoadOption.STRAIGHT))

                        # 生成对应的GPS坐标
                        gps_coord = _location_to_gps(lat_ref, lon_ref, current_wp.transform.location)
                        extended_gps_waypoints.append((gps_coord, RoadOption.STRAIGHT))

                        # 检查是否已经到达或超过原始位置
                        dist_to_original = current_wp.transform.location.distance(original_location)
                        if dist_to_original < 5.0:  # 接近原始位置
                            break

                        # 向前移动一段距离
                        next_wps = current_wp.next(2.0)  # 每2米一个点
                        if not next_wps:
                            break
                        current_wp = next_wps[0]

                    # 获取原始gps_route（保留完整，稍后会在agent中通过索引去重）
                    if route_scenario_instance and hasattr(route_scenario_instance, 'gps_route'):
                        original_gps_route = list(route_scenario_instance.gps_route)
                    else:
                        original_gps_route = []

                    # 组合新route：扩展部分 + 原始route
                    original_route = config.route

                    # 只有在成功扩展waypoints时才更新route
                    if len(extended_waypoints) > 0 and len(extended_gps_waypoints) > 0:
                        config.route = extended_waypoints + original_route
                        new_gps_route = extended_gps_waypoints + original_gps_route

                        # 更新RouteScenario实例
                        if route_scenario_instance:
                            route_scenario_instance.route = config.route
                            route_scenario_instance.gps_route = new_gps_route

                            # 关键：也需要更新已经创建的InRouteTest实例
                            if hasattr(route_scenario_instance, 'criteria_tree') and route_scenario_instance.criteria_tree:
                                import py_trees
                                for child in route_scenario_instance.criteria_tree.iterate():
                                    class_name = child.__class__.__name__ if hasattr(child, '__class__') else ""

                                    # 更新InRouteTest
                                    if class_name == 'InRouteTest':
                                        # 更新route数据
                                        child._route = config.route
                                        child._route_transforms, _ = zip(*config.route)
                                        child._route_length = len(config.route)
                                        # 重新计算accum_meters
                                        child._accum_meters = []
                                        prev_loc = child._route_transforms[0].location
                                        for i, transform in enumerate(child._route_transforms):
                                            # 关键修复：transform是Transform对象，需要用.location.distance()
                                            d = transform.location.distance(prev_loc)
                                            accum = 0 if i == 0 else child._accum_meters[i - 1]
                                            child._accum_meters.append(d + accum)
                                            prev_loc = transform.location
                                        # 重置current_index到0，避免索引越界
                                        child._current_index = 0
                                        # 关键：禁用terminate_on_failure，避免车辆不在route时立即终止
                                        child.terminate_on_failure = False

                                    # 更新RouteCompletionTest
                                    elif class_name == 'RouteCompletionTest':
                                        child._route = config.route
                                        child._route_length = len(config.route)
                                        child._route_transforms, _ = zip(*config.route)
                                        child._route_accum_perc = child._get_acummulated_percentages()
                                        child.target_location = child._route_transforms[-1].location

                                    # 更新OutsideRouteLanesTest
                                    elif class_name == 'OutsideRouteLanesTest':
                                        child._route = config.route
                                        child._route_length = len(config.route)
                                        child._route_transforms, _ = zip(*config.route)

                                    # 更新MinimumSpeedRouteTest
                                    elif class_name == 'MinimumSpeedRouteTest':
                                        child._route = config.route
                                        child._checkpoints = 4
                    else:
                        pass  # Route extension failed, but continue anyway

        # =======================================================
        # 3. 获取地图与基准 Waypoint（障碍物位置保持不变）
        # =======================================================
        # 使用XML中的trigger_point作为基准位置
        base_location = config.trigger_points[0].location

        # 获取起点的 Waypoint (车道中心点数据)
        base_wp = carla_map.get_waypoint(base_location)

        # =======================================================
        # 4. 生成障碍物（卡车和锥桶）
        # =======================================================

        try:
            # 1. 生成大货车
            self._spawn_actor_on_route(base_wp, self._truck_distance, self._truck_offset, "vehicle.carlamotors.carlacola", True)

            # 2. 生成斜向引导锥桶 (8个)
            num_cones = 8
            cone_model = "static.prop.constructioncone"
            for i in range(num_cones):
                fraction = i / float(num_cones - 1)
                forward_dist = self._cone_start_dist + (self._cone_end_dist - self._cone_start_dist) * fraction
                lateral_offset = self._cone_start_off + (self._cone_end_off - self._cone_start_off) * fraction
                self._spawn_actor_on_route(base_wp, forward_dist, lateral_offset, cone_model)

            # 3. 生成直排隔离锥桶 (12个)
            num_straight_cones = 12
            straight_end_dist = self._cone_end_dist + 35.0
            for i in range(1, num_straight_cones):
                fraction = i / float(num_straight_cones - 1)
                forward_dist = self._cone_end_dist + (straight_end_dist - self._cone_end_dist) * fraction
                self._spawn_actor_on_route(base_wp, forward_dist, self._cone_end_off, cone_model)
        except Exception as e:
            print(f"[ERROR] Failed to spawn actors: {e}", flush=True)
            import traceback
            traceback.print_exc()

        # =======================================================
        # 5. 在所有actors创建完成后，立即设置车辆初始速度
        # =======================================================
        # 使用多个连续waypoint来计算更准确的行驶方向，特别是对于弯曲道路
        ego_location = self.ego_vehicles[0].get_location()
        ego_wp = carla_map.get_waypoint(ego_location)

        if ego_wp:
            # 向前追踪多个waypoint来获得更好的方向估计
            # 对于高速，需要更远的lookahead距离
            lookahead_distance = 50.0  # 向前看50米
            next_wps = ego_wp.next(lookahead_distance)

            if len(next_wps) > 0:
                # 使用最远的前向waypoint来计算方向
                next_wp = next_wps[-1]  # 使用最后一个（最远的）waypoint
                direction_vector = next_wp.transform.location - ego_wp.transform.location

                # 只在水平面上计算方向
                direction_vector.z = 0
                direction = direction_vector / (direction_vector.length() + 1e-6)
            else:
                # 如果无法获取前向waypoint，使用当前waypoint的forward向量
                direction = ego_wp.transform.get_forward_vector()
                direction.z = 0
                direction = direction / (direction.length() + 1e-6)
        else:
            # 回退方案：使用车辆当前的transform
            ego_transform = self.ego_vehicles[0].get_transform()
            direction = ego_transform.get_forward_vector()
            direction.z = 0
            direction = direction / (direction.length() + 1e-6)

        # 4. 初始化自车速度
        target_speed_mps = self._initial_speed_kph / 3.6
        ego_transform = self.ego_vehicles[0].get_transform()
        direction = ego_transform.get_forward_vector()
        direction.z = 0.0 
        
        # 归一化后设置速度
        direction = direction / (direction.length() + 1e-6)
        self.ego_vehicles[0].set_target_velocity(
            carla.Vector3D(direction.x * target_speed_mps, direction.y * target_speed_mps, 0.0)
        )

        # 设置目标速度
        self.ego_vehicles[0].set_target_velocity(ego_velocity)

    def _create_behavior(self):
        """
        创建场景行为树：
        运行测试直到车辆通过卡车并行驶40米
        """
        root = py_trees.composites.Sequence("LaneClosureBehavior")

        # 场景在过卡车40m后结束：55m (truck) + 40m (buffer) = 95m
        end_distance = self._truck_distance + 40.0
        end_condition = DriveDistance(self.ego_vehicles[0], end_distance)
        root.add_child(end_condition)

        return root

    def _create_test_criteria(self):
        criteria = []

        # 1. CollisionTest - 检测与所有障碍物（锥桶、卡车）的碰撞
        # 使用 "miscellaneous" 来检测所有静态对象（包括锥桶）
        collision_static_criterion = CollisionTest(
            self.ego_vehicles[0],
            other_actor_type="miscellaneous",
            terminate_on_failure=False,
            name="CollisionTest"
        )
        criteria.append(collision_static_criterion)

        # 为了确保也检测卡车碰撞，再添加一个针对车辆的 CollisionTest
        if self.other_actors:
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
            start_distance=self._cone_start_dist,  # 30米
            end_distance=self._truck_distance,          # 55米（卡车位置）
            initial_speed_kmh=self._initial_speed_kph,  # 从XML读取
            target_speed_reduction=40.0                 # 需要减速至少40km/h
        )
        criteria.append(deceleration_criterion)

        # 4. 路段通过检测 - 检测是否通过卡车并再行驶40米
        pass_completion_criterion = RoutePassCompletionTest(
            self.ego_vehicles[0],
            pass_distance=self._truck_distance + 40.0  # 55 + 40 = 95米
        )
        criteria.append(pass_completion_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()