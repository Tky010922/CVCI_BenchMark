import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
<<<<<<< HEAD
=======
from srunner.scenariomanager.timer import GameTime
>>>>>>> dev
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    WaypointFollower,
    LaneChange,
)
<<<<<<< HEAD
from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
    CrazyBikeDecelerateCriterion,
    CrazyBikeNoCollisionCriterion,
    CrazyBikeResumeCriterion,
)
=======
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
>>>>>>> dev
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    InTriggerDistanceToVehicle,
    DriveDistance,
)
from srunner.scenarios.basic_scenario import BasicScenario


def get_value_parameter(config, name, p_type, default):
    parameter = getattr(config, "other_parameters", {}).get(name, {})
    if "value" not in parameter:
        return default
    return p_type(parameter["value"])


<<<<<<< HEAD
=======
class EgoStateLogger(py_trees.behaviour.Behaviour):
    """Print ego state to terminal at a fixed interval."""

    def __init__(self, actor, interval=0.2, name="EgoStateLogger"):
        super(EgoStateLogger, self).__init__(name)
        self.actor = actor
        self.interval = max(0.05, interval)
        self._last_print_time = -1.0

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if not self.actor:
            return new_status

        sim_time = GameTime.get_time()
        if self._last_print_time < 0.0 or (sim_time - self._last_print_time) >= self.interval:
            transform = self.actor.get_transform()
            location = transform.location
            yaw = transform.rotation.yaw
            vel = self.actor.get_velocity()
            speed = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
            print(
                "[CrazyBike][Ego] t={:.2f}s x={:.2f}, y={:.2f}, z={:.2f}, yaw={:.1f}, v={:.2f}m/s".format(
                    sim_time, location.x, location.y, location.z, yaw, speed
                )
            )
            self._last_print_time = sim_time

        return new_status


>>>>>>> dev
class PassiveEgoSpeedHold(py_trees.behaviour.Behaviour):
    """Keep ego speed only when the driver has not provided active control input."""

    def __init__(self, actor, target_speed, name="PassiveEgoSpeedHold"):
        super(PassiveEgoSpeedHold, self).__init__(name)
        self.actor = actor
        self.target_speed = target_speed
        self._manual_override_detected = False

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if not self.actor:
            return new_status

        control = self.actor.get_control()
        passive_input = (
            abs(control.steer) < 0.05 and
            control.brake < 0.05 and
            control.throttle < 0.05
        )

        if not passive_input:
            self._manual_override_detected = True

        if not self._manual_override_detected and passive_input:
            transform = self.actor.get_transform()
            forward_vector = transform.get_forward_vector()
            self.actor.set_target_velocity(carla.Vector3D(
                forward_vector.x * self.target_speed,
                forward_vector.y * self.target_speed,
                forward_vector.z * self.target_speed
            ))

        return new_status


class CrazyBikeScenario(BasicScenario):
    """
    Ego drives straight while a bike in the side-front lane suddenly cuts in.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=90):
        self.timeout = timeout
        self._bike_actor = None

        self._ego_initial_speed = get_value_parameter(config, "init_speed", float, 0.0)
        self._bike_speed_limit = get_value_parameter(config, "bike_speed_limit", float, 5.5)
        self._bike_speed = self._clamp_bike_speed(get_value_parameter(config, "bike_speed", float, 3.8))
        self._bike_cross_speed = self._clamp_bike_speed(get_value_parameter(config, "bike_cross_speed", float, 4.6))
        self._cut_in_trigger_distance = get_value_parameter(config, "cut_in_trigger_distance", float, 18.0)
        self._distance_same_lane = get_value_parameter(config, "distance_same_lane", float, 3.0)
        self._distance_other_lane = get_value_parameter(config, "distance_other_lane", float, 20.0)
        self._post_change_distance = get_value_parameter(config, "post_change_distance", float, 25.0)
<<<<<<< HEAD
        self._decel_trigger_distance = get_value_parameter(config, "decel_trigger_distance", float, 24.0)
        self._decel_latest_reaction_distance = get_value_parameter(
            config, "decel_latest_reaction_distance", float, 10.0)
        self._decel_min_speed_drop = get_value_parameter(config, "decel_min_speed_drop", float, 2.0)
        self._resume_escape_distance = get_value_parameter(config, "resume_escape_distance", float, 12.0)
        self._resume_speed = get_value_parameter(config, "resume_speed", float, 5.0)
        self._resume_min_duration = get_value_parameter(config, "resume_min_duration", float, 1.0)
        self._resume_lane_tolerance = get_value_parameter(config, "resume_lane_tolerance", float, 1.8)
=======
        self._ego_log_interval = get_value_parameter(config, "ego_log_interval", float, 0.2)
>>>>>>> dev

        lane_change_direction = get_value_parameter(config, "lane_change_direction", str, "left").lower()
        self._lane_change_direction = lane_change_direction if lane_change_direction in ("left", "right") else "left"

        super(CrazyBikeScenario, self).__init__(
            "CrazyBikeScenario",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable
        )

    def _clamp_bike_speed(self, value):
        return max(1.5, min(value, self._bike_speed_limit))

    def _get_route_anchor_locations(self):
        route_start_loc = self.config.trigger_points[0].location
<<<<<<< HEAD
        route_end_loc = carla.Location(x=110.0, y=1.8, z=0.5)
=======
        route_end_loc = carla.Location(x=95.0, y=1.8, z=0.5)
>>>>>>> dev

        if self.config.route:
            route_start_loc = self.config.route[0][0].location
            route_end_loc = self.config.route[-1][0].location

        return route_start_loc, route_end_loc

    def _get_route_length(self):
        route_start_loc, route_end_loc = self._get_route_anchor_locations()
        return route_start_loc.distance(route_end_loc)

    def _initialize_actors(self, config):
        ego = self.ego_vehicles[0]
        if self._ego_initial_speed > 0.1:
            forward_vector = ego.get_transform().get_forward_vector()
            ego.set_target_velocity(carla.Vector3D(
                forward_vector.x * self._ego_initial_speed,
                forward_vector.y * self._ego_initial_speed,
                forward_vector.z * self._ego_initial_speed
            ))

        for actor_config in config.other_actors:
            actor = self._spawn_actor_with_debug(actor_config)
            if actor:
                self.other_actors.append(actor)

        if self.other_actors:
            self._bike_actor = self.other_actors[0]
            bike_tf = self._bike_actor.get_transform()
            bike_forward = bike_tf.get_forward_vector()
            self._bike_actor.set_target_velocity(carla.Vector3D(
                bike_forward.x * self._bike_speed,
                bike_forward.y * self._bike_speed,
                bike_forward.z * self._bike_speed
            ))
<<<<<<< HEAD

    def _spawn_actor_with_debug(self, actor_config):
=======
            print(
                "[CrazyBike] Bike speed setup: cruise={:.2f}m/s, cross={:.2f}m/s, limit={:.2f}m/s".format(
                    self._bike_speed, self._bike_cross_speed, self._bike_speed_limit
                )
            )

    def _spawn_actor_with_debug(self, actor_config):
        world = CarlaDataProvider.get_world()
>>>>>>> dev
        spawn_transform = actor_config.transform
        spawn_loc = spawn_transform.location
        spawn_rot = spawn_transform.rotation

<<<<<<< HEAD
=======
        print(
            "[CrazyBike] Try spawn adversary model={} at x={:.2f}, y={:.2f}, z={:.2f}, yaw={:.2f}".format(
                actor_config.model, spawn_loc.x, spawn_loc.y, spawn_loc.z, spawn_rot.yaw
            )
        )
        if world:
            world.debug.draw_point(
                spawn_loc + carla.Location(z=0.4), size=0.12, color=carla.Color(255, 255, 0), life_time=20.0
            )
            world.debug.draw_string(
                spawn_loc + carla.Location(z=1.2), "Bike spawn try", draw_shadow=False,
                color=carla.Color(255, 255, 0), life_time=20.0, persistent_lines=False
            )

>>>>>>> dev
        candidate_models = []
        for model_name in [actor_config.model, "vehicle.vespa.zx125", "vehicle.gazelle.omafiets"]:
            if model_name not in candidate_models:
                candidate_models.append(model_name)

        longitudinal_offsets = [0.0, 1.0, -1.0, 2.0, -2.0]
        for model_name in candidate_models:
            for dx in longitudinal_offsets:
                transform = carla.Transform(
                    carla.Location(x=spawn_loc.x + dx, y=spawn_loc.y, z=spawn_loc.z),
                    spawn_rot
                )
                actor = CarlaDataProvider.request_new_actor(model_name, transform)
                if actor:
<<<<<<< HEAD
                    return actor

=======
                    actor_loc = actor.get_location()
                    print(
                        "[CrazyBike] Spawn success model={} id={} at x={:.2f}, y={:.2f}, z={:.2f}".format(
                            model_name, actor.id, actor_loc.x, actor_loc.y, actor_loc.z
                        )
                    )
                    if world:
                        world.debug.draw_point(
                            actor_loc + carla.Location(z=0.4), size=0.14, color=carla.Color(0, 255, 0), life_time=20.0
                        )
                        world.debug.draw_string(
                            actor_loc + carla.Location(z=1.2), "Bike spawn success", draw_shadow=False,
                            color=carla.Color(0, 255, 0), life_time=20.0, persistent_lines=False
                        )
                    return actor

        print(
            "[CrazyBike] Spawn failed for adversary near x={:.2f}, y={:.2f}, z={:.2f}".format(
                spawn_loc.x, spawn_loc.y, spawn_loc.z
            )
        )
        if world:
            world.debug.draw_point(
                spawn_loc + carla.Location(z=0.4), size=0.14, color=carla.Color(255, 0, 0), life_time=20.0
            )
            world.debug.draw_string(
                spawn_loc + carla.Location(z=1.2), "Bike spawn failed", draw_shadow=False,
                color=carla.Color(255, 0, 0), life_time=20.0, persistent_lines=False
            )
>>>>>>> dev
        return None

    def _resolve_lane_change_direction(self):
        if self._bike_actor is None:
            return self._lane_change_direction
        bike_y = self._bike_actor.get_location().y
        ego_y = self.ego_vehicles[0].get_location().y
        if bike_y > ego_y:
            resolved = "left"
        else:
            resolved = "right"
<<<<<<< HEAD
=======
        print("[CrazyBike] LaneChange direction resolved by y: bike_y={:.2f}, ego_y={:.2f}, direction={}".format(
            bike_y, ego_y, resolved
        ))
>>>>>>> dev
        return resolved

    def _create_behavior(self):
        root = py_trees.composites.Parallel(
            "CrazyBikeBehavior",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )

        scenario_flow = py_trees.composites.Parallel(
            "ScenarioFlow",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL
        )

<<<<<<< HEAD
=======
        scenario_flow.add_child(EgoStateLogger(self.ego_vehicles[0], interval=self._ego_log_interval))

>>>>>>> dev
        if self._ego_initial_speed > 0.1:
            scenario_flow.add_child(
                PassiveEgoSpeedHold(self.ego_vehicles[0], self._ego_initial_speed)
            )

        if self._bike_actor is not None:
            bike_cut_in = py_trees.composites.Sequence("BikeCutIn")

            approach_stage = py_trees.composites.Parallel(
                "BikeApproachAndTrigger",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
            )
            approach_stage.add_child(
                WaypointFollower(self._bike_actor, self._bike_speed, avoid_collision=False)
            )
            approach_stage.add_child(
                InTriggerDistanceToVehicle(
                    self.ego_vehicles[0],
                    self._bike_actor,
                    distance=self._cut_in_trigger_distance
                )
            )
            bike_cut_in.add_child(approach_stage)

            bike_cut_in.add_child(
                LaneChange(
                    self._bike_actor,
                    speed=self._bike_cross_speed,
                    direction=self._resolve_lane_change_direction(),
                    distance_same_lane=max(1.0, self._distance_same_lane),
                    distance_other_lane=max(10.0, self._distance_other_lane),
                    distance_lane_change=10.0,
                    lane_changes=1,
                    name="BikeLaneChangeByAPI"
                )
            )

            post_merge_straight = py_trees.composites.Parallel(
                "BikePostMergeStraight",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
            )
            post_merge_straight.add_child(
                WaypointFollower(
                    self._bike_actor,
                    target_speed=self._bike_speed,
                    avoid_collision=False,
                    name="BikeKeepStraightAfterMerge"
                )
            )
            post_merge_straight.add_child(DriveDistance(self._bike_actor, self._post_change_distance))
            bike_cut_in.add_child(post_merge_straight)
            scenario_flow.add_child(bike_cut_in)

        root.add_child(scenario_flow)
        root.add_child(DriveDistance(self.ego_vehicles[0], distance=max(45.0, self._get_route_length() + 5.0)))

        return root

    def _create_test_criteria(self):
        criteria = []
<<<<<<< HEAD
        route_start_loc, route_end_loc = self._get_route_anchor_locations()
        if self._bike_actor is None:
            return criteria

        criteria.append(CrazyBikeDecelerateCriterion(
            actor=self.ego_vehicles[0],
            bike_actor=self._bike_actor,
            route_start_location=route_start_loc,
            route_end_location=route_end_loc,
            trigger_distance=self._decel_trigger_distance,
            latest_reaction_distance=self._decel_latest_reaction_distance,
            min_speed_drop=self._decel_min_speed_drop
        ))
        criteria.append(CrazyBikeNoCollisionCriterion(
            actor=self.ego_vehicles[0],
            bike_actor=self._bike_actor
        ))
        criteria.append(CrazyBikeResumeCriterion(
            actor=self.ego_vehicles[0],
            bike_actor=self._bike_actor,
            route_start_location=route_start_loc,
            route_end_location=route_end_loc,
            escape_distance=self._resume_escape_distance,
            resume_speed=self._resume_speed,
            min_resume_duration=self._resume_min_duration,
            lane_tolerance=self._resume_lane_tolerance
        ))
=======
        criteria.append(CollisionTest(self.ego_vehicles[0]))
>>>>>>> dev
        return criteria

    def __del__(self):
        self.remove_all_actors()
