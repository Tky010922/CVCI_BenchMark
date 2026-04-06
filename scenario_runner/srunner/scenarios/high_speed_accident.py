import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario


def get_value_parameter(config, name, p_type, default):
    parameter = getattr(config, "other_parameters", {}).get(name, {})
    if "value" not in parameter:
        return default
    return p_type(parameter["value"])


class HighSpeedAccident(BasicScenario):
    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=120):
        self._init_speed = get_value_parameter(config, "init_speed", float, 10.0)
        self._accident_lateral_distance = get_value_parameter(config, "lateral_distance", float, 4.63)
        self._accident_longitudinal_distance = 30.0
        self._accident_yaw = -135.0

        super(HighSpeedAccident, self).__init__(
            "HighSpeedAccident",
            ego_vehicles,
            config,
            world,
            debug_mode=debug_mode,
            criteria_enable=criteria_enable
        )

    def _get_route_anchor_locations(self):
        """Use the route XML endpoints when they are available."""
        route_start_loc = self.config.trigger_points[0].location
        route_end_loc = carla.Location(x=287.36, y=159.47, z=0.5)

        if self.config.route:
            route_start_loc = self.config.route[0][0].location
            route_end_loc = self.config.route[-1][0].location

        return route_start_loc, route_end_loc

    @staticmethod
    def _normalize_vector_2d(x_value, y_value):
        magnitude = (x_value ** 2 + y_value ** 2) ** 0.5
        if magnitude < 1e-6:
            return 1.0, 0.0
        return x_value / magnitude, y_value / magnitude

    def _get_accident_transform(self):
        route_start_loc, route_end_loc = self._get_route_anchor_locations()
        forward_xy = self._normalize_vector_2d(
            route_end_loc.x - route_start_loc.x,
            route_end_loc.y - route_start_loc.y
        )
        right_xy = (-forward_xy[1], forward_xy[0])

        accident_loc = carla.Location(
            x=route_start_loc.x + forward_xy[0] * self._accident_longitudinal_distance
            + right_xy[0] * self._accident_lateral_distance,
            y=route_start_loc.y + forward_xy[1] * self._accident_longitudinal_distance
            + right_xy[1] * self._accident_lateral_distance,
            z=0.5
        )
        accident_rot = carla.Rotation(yaw=self._accident_yaw)
        return carla.Transform(accident_loc, accident_rot)

    def _initialize_actors(self, config):
        ego = self.ego_vehicles[0]
        forward_vector = ego.get_transform().get_forward_vector()

        ego.set_target_velocity(carla.Vector3D(
            forward_vector.x * self._init_speed,
            forward_vector.y * self._init_speed,
            forward_vector.z * self._init_speed
        ))

        transform = self._get_accident_transform()

        try:
            accident_car = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', transform)
            if accident_car:
                self.other_actors.append(accident_car)
                accident_car.set_light_state(carla.VehicleLightState(carla.VehicleLightState.All))
        except Exception as error:
            print(f"Actor creation failed: {error}")

    def _create_behavior(self):
        root = py_trees.composites.Parallel(
            "NightBehavior",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE
        )
        root.add_child(DriveDistance(self.ego_vehicles[0], 60))
        return root

    def _create_test_criteria(self):
        criteria = []
        ego = self.ego_vehicles[0]
        hazard_vehicle = self.other_actors[0]
        route_start_loc, route_end_loc = self._get_route_anchor_locations()

        from srunner.scenariomanager.scenarioatomics.atomic_criteria import (
            HighSpeedBrakeCriterion,
            HighSpeedBypassCriterion,
            HighSpeedResumeCriterion,
        )

        criteria.append(
            HighSpeedBrakeCriterion(
                actor=ego,
                hazard_actor=hazard_vehicle,
                route_start_location=route_start_loc,
                route_end_location=route_end_loc,
                trigger_distance=50.0,
                brake_threshold=0.3,
            )
        )

        criteria.append(
            HighSpeedBypassCriterion(
                actor=ego,
                hazard_actor=hazard_vehicle,
                route_start_location=route_start_loc,
                route_end_location=route_end_loc,
                safe_lateral_margin=2.4,
                danger_lateral_margin=1.9,
                passing_longitudinal_zone=7.5,
                min_route_offset=0.55,
            )
        )

        criteria.append(
            HighSpeedResumeCriterion(
                actor=ego,
                hazard_actor=hazard_vehicle,
                route_start_location=route_start_loc,
                route_end_location=route_end_loc,
                escape_distance=15.0,
                min_resume_speed=5.0,
            )
        )

        criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria
