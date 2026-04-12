def extract_common_facts(criteria_list):
    common_facts = {
        "collision": False,
        "min_ttc": None,
        "outside_route": False,
        "running_red_light": False,
        "running_stop": False,
        "agent_blocked": False,
        "route_completed": False,
    }
    for criterion in criteria_list:
        name = criterion.name
        if name == "CollisionTest":
            common_facts["collision"] = (criterion.test_status == "FAILURE" or len(criterion.events) > 0)

        elif name == "OutsideRouteLanesTest":
            common_facts["outside_route"] = (criterion.test_status == "FAILURE")

        elif name == "RunningRedLightTest":
            common_facts["running_red_light"] = (criterion.test_status == "FAILURE")

        elif name == "RunningStopTest":
            common_facts["running_stop"] = (criterion.test_status == "FAILURE")

        elif name == "AgentBlockedTest":
            common_facts["agent_blocked"] = (criterion.test_status == "FAILURE")

        elif name == "RouteCompletionTest":
            common_facts["route_completed"] = (criterion.test_status == "SUCCESS")

        elif name == "MinTTCAutoCriterion":
            print(criterion.actual_value)
            common_facts["min_ttc"] = float(criterion.actual_value)

    return common_facts

# missing car_private_facats extracts
def extract_private_facts_frontcar_disappearance(criteria_list):
    facts = {
        "slow_down": False,       # 条件1：自车成功减速
        "no_collision": False,    # 条件2：无碰撞
        "safe_bypass": False     # 条件3：安全变道通过故障车
    }

    for criterion in criteria_list:
        # 减速成功
        if criterion.name == "StaticObstacleSlowDownCriterion":
            facts["slow_down"] = (criterion.test_status == "SUCCESS")

        # 无碰撞成功
        if criterion.name == "StaticObstacleNoCollisionCriterion":
            facts["no_collision"] = (criterion.test_status == "SUCCESS")

        # 安全变道通过
        if criterion.name == "StaticObstacleSafePassCriterion":
            facts["safe_bypass"] = (criterion.test_status == "SUCCESS")

    return facts

# High speed temporary construction_private_facats extracts
def extract_private_facts_static_barrier(criteria_list):
    facts = {
        "slow_down": False,       # 成功减速
        "safe_bypass": False,     # 成功绕行
        "pass_barrier": False     # 成功通过路障
    }

    for criterion in criteria_list:
        if criterion.name == "BarrierSlowDownCriterion":
            facts["slow_down"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "BarrierPassByCriterion":
            facts["pass_barrier"] = (criterion.test_status == "SUCCESS")

        # 安全绕行 = 不碰撞 + 成功通过
        # 这里用 BarrierPassByCriterion 代表安全绕行完成
        if criterion.name == "BarrierPassByCriterion":
            facts["safe_bypass"] = (criterion.test_status == "SUCCESS")
    return facts

# High-speed reckless lane cutting_private_facats extracts
def extract_private_facts_high_speed_cutting(criteria_list):
    """提取高速切入场景的私有事实"""
    facts = {
        'brake_response': False,
        'safe_bypass': False,
    }

    for criterion in criteria_list:
        if criterion.name == 'CutInBrakeResponseCriterion':
            facts['brake_response'] = (criterion.brake_status == "SUCCESS")
        elif criterion.name == 'CutInSafeBypassCriterion':
            facts['safe_bypass'] = (criterion.safepass_status == "SUCCESS")

    return facts

# Highway accident vehicle_private_facats extracts
def extract_private_facts_high_speed_accident(criteria_list):
    """提取高速深夜事故场景的私有事实"""
    facts = {
        "brake_response": False,    # 识别事故车并减速
        "safe_bypass": False,       # 安全绕行
        "resume_route": False,      # 成功通过事故区域后恢复行驶
    }
    for criterion in criteria_list:
        if criterion.name == "HighSpeedBrakeCriterion":
            facts["brake_response"] = (criterion.test_status == "SUCCESS")
        elif criterion.name == "HighSpeedBypassCriterion":
            facts["safe_bypass"] = (criterion.test_status == "SUCCESS")
        elif criterion.name == "HighSpeedResumeCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")
    return facts
# Trucks encountered during construction_private_facats extracts
def extract_private_facts_lane_closure(criteria_list):
    """提取车道封闭场景的私有事实"""
    facts = {
        "deceleration_detected": False,  # 识别障碍并减速
        "speed_reduction": 0.0,          # 减速幅度
        "distance_traveled": 0.0,        # 行驶距离
    }
    for criterion in criteria_list:
        if criterion.name == "DecelerationForConstructionTest":

            speed_reduction = criterion.actual_value
            facts["speed_reduction"] = speed_reduction

            # 如果减速 ≥ 30 km/h（认为是有效的减速行为），或者 test_status 是 SUCCESS
            if speed_reduction >= 30.0 or criterion.test_status == "SUCCESS":
                facts["deceleration_detected"] = True

            # print(f"[DEBUG Facts Extract] DecelerationForConstructionTest: status={criterion.test_status}, actual_value={criterion.actual_value}, deceleration_detected={facts['deceleration_detected']}", flush=True)
        elif criterion.name == "RoutePassCompletionTest":
            facts["distance_traveled"] = criterion.actual_value
            # print(f"[DEBUG Facts Extract] RoutePassCompletionTest: status={criterion.test_status}, actual_value={criterion.actual_value}", flush=True)
    return facts
# Drive into the roundabout_private_facats extracts
def extract_private_facts_roundabout_merge_conflict(criteria_list):
    facts = {
        "decelerate_response": False,
        "yield_convoy": False,
        "safe_pass": False,
    }

    for criterion in criteria_list:
        if criterion.name == "RoundaboutDecelerateCriterion":
            facts["decelerate_response"] = (criterion.decelerate_status == "SUCCESS")

        elif criterion.name == "RoundaboutYieldConvoyCriterion":
            facts["yield_convoy"] = (criterion.yield_status == "SUCCESS")

        elif criterion.name == "RoundaboutSafePassCriterion":
            facts["safe_pass"] = (criterion.safe_pass_status == "SUCCESS")

    return facts
# Four students crossing the road_private_facats extracts
def extract_private_facts_ghost_probe(criteria_root):
    """提取鬼探头场景的私有事实"""
    facts = {
        "scooter_decelerate": False,    # 识别到电动车并成功减速
        "pedestrian_stop": False,       # 识别到行人并成功停车
        "pedestrian_resume": False,     # 待行人离开后恢复行驶
    }

    nodes = criteria_root.iterate() if hasattr(criteria_root, 'iterate') else criteria_root

    for criterion in nodes:
        if hasattr(criterion, 'name'):
            if criterion.name == "ScooterDecelerateCriterion":
                facts["scooter_decelerate"] = (criterion.test_status == "SUCCESS")
            elif criterion.name == "PedestrianStopCriterion":
                facts["pedestrian_stop"] = (criterion.test_status == "SUCCESS")
            elif criterion.name == "PedestrianResumeCriterion":
                facts["pedestrian_resume"] = (criterion.test_status == "SUCCESS")

    return facts

# avoid a disabled vehicle_private_facats extracts
def extract_private_facts_broken_down_vehicle(criteria_list):
    facts = {
        "brake_response": False,
        "safe_bypass": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "BrakeCriterion":
            facts["brake_response"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "BypassCriterion":
            facts["safe_bypass"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "ResumeCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")

    return facts

# Slanted motor and children_private_facats extracts
def extract_private_facts_ebike_pedestrian_cross(criteria_list):
    """
    提取 EbikeAndPedestrianCross 场景的私有事实
    """
    facts = {
        "ebike_decelerate": False,      # 识别电瓶车并减速
        "pedestrian_stop": False,       # 识别行人并刹车
        "resume_route": False,          # 离开风险区并恢复通行
    }

    for criterion in criteria_list:
        if criterion.name == "EbikeDetectionAndDecelerateCriterion":
            facts["ebike_decelerate"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "PedestrianDetectionAndStopCriterion":
            facts["pedestrian_stop"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "ResumeAfterPedestrianCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")

    return facts

# reverse vehicle_private_facats extracts
def extract_private_facts_reverse_vehicle(criteria_list):
    facts = {
        "brake_response": False,
        "safe_bypass": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "ReverseVehicleBrakeCriterion":
            facts["brake_response"] = (criterion.brake_status == "SUCCESS")

        elif criterion.name == "ReverseVehicleBypassCriterion":
            facts["safe_bypass"] = (criterion.bypass_status == "SUCCESS")

        elif criterion.name == "ReverseVehicleResumeCriterion":
            facts["resume_route"] = (criterion.resume_status == "SUCCESS")

    return facts

# crazy motor_private_facats extracts
def extract_private_facts_crazy_bike(criteria_list):
    facts = {
        "decelerate_response": False,
        "no_collision": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "CrazyBikeDecelerateCriterion":
            facts["decelerate_response"] = (criterion.test_status == "SUCCESS")
        elif criterion.name == "CrazyBikeNoCollisionCriterion":
            facts["no_collision"] = (criterion.test_status == "SUCCESS")
        elif criterion.name == "CrazyBikeResumeCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")

    return facts

# Blind spot hidden car_private_facats extracts
def extract_private_facts_left_turn(criteria_list):
    facts = {
        "brake_response": False,
        "safe_bypass": False,
        "resume_route": False,
    }

    for criterion in criteria_list:
        if criterion.name == "IntersectionCollisionLeftTurnBrakeCriterion":
            facts["brake_response"] = (criterion.test_status == "SUCCESS")

        elif criterion.name == "IntersectionCollisionLeftTurnResumeCriterion":
            facts["resume_route"] = (criterion.test_status == "SUCCESS")

    return facts


