def compute_gate(common_facts):
    if common_facts["collision"]:
        return 0.0
    return 1.0


def compute_penalty(common_facts):
    penalty = 1.0
    
    min_ttc = common_facts.get("min_ttc")
    if min_ttc is not None:
        if min_ttc >= 2.0:
            penalty *= 1.00
        elif min_ttc >= 1.5:
            penalty *= 0.95
        elif min_ttc >= 1.0:
            penalty *= 0.85
        elif min_ttc >= 0.5:
            penalty *= 0.65
        else:
            penalty *= 0.0

    if common_facts["outside_route"]:
        penalty *= 0.9

    return penalty

# scenario specific score calculate

# missing car

# High speed temporary construction

# High-speed reckless lane cutting

# Highway accident vehicle
def score_high_speed_accident(common_facts, private_facts):
    """计算高速深夜事故场景得分"""
    base_score = 0.0

    # 根据我们之前定义的权重分配
    if private_facts["brake_response"]:
        base_score += 55.0
    if private_facts["safe_bypass"]:
        base_score += 30.0
    if private_facts["resume_route"]:
        base_score += 15.0

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }
# Trucks encountered during construction

# Drive into the roundabout

# Four students crossing the road
def score_ghost_probe(common_facts, private_facts):
    """计算鬼探头场景得分"""
    base_score = 0.0

    if private_facts["scooter_decelerate"]:
        base_score += 25.0  # 识别遮挡物并减速
    if private_facts["pedestrian_stop"]:
        base_score += 55.0  # 彻底刹停让行
    if private_facts["pedestrian_resume"]:
        base_score += 20.0  # 安全起步

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }
# avoid a disabled vehicle

# Slanted motor and children

# reverse vehicle
def score_reverse_vehicle(common_facts, private_facts):
    base_score = 0.0
    # BaseScore: private fatcs calculate
    if private_facts["brake_response"]:
        base_score += 55.0
    if private_facts["safe_bypass"]:
        base_score += 20.0
    if private_facts["resume_route"]:
        base_score += 25.0
    # Gate: colision
    gate = compute_gate(common_facts)
    # Penalty: minttc and out_of_road
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }

# crazy motor

# Blind spot hidden car
def score_left_turn(common_facts, private_facts):
    base_score = 0.0

    if private_facts["brake_response"]:
        base_score += 70.0
    if private_facts["resume_route"]:
        base_score += 30.0

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }

# ========== 以下是 cpz 添加的 EbikeAndPedestrianCross 场景的评分函数 ==========

def score_ebike_pedestrian_cross(common_facts, private_facts):
    """
    EbikeAndPedestrianCross 场景的评分函数
    
    评分标准：
    - 识别电瓶车并减速: 25分
    - 识别行人并刹车: 50分
    - 离开风险区并恢复通行: 25分
    """
    base_score = 0.0

    # BaseScore: 根据私有事实计算
    if private_facts.get("ebike_decelerate", False):
        base_score += 25.0
    if private_facts.get("pedestrian_stop", False):
        base_score += 50.0
    if private_facts.get("resume_route", False):
        base_score += 25.0

    # Gate: 发生碰撞则分数归零
    gate = 1.0
    if common_facts.get("collision", False):
        gate = 0.0

    # Penalty: 基于最小TTC和越界
    penalty = 1.0
    min_ttc = common_facts.get("min_ttc")
    if min_ttc is not None:
        if min_ttc >= 2.0:
            penalty *= 1.00
        elif min_ttc >= 1.5:
            penalty *= 0.95
        elif min_ttc >= 1.0:
            penalty *= 0.85
        elif min_ttc >= 0.5:
            penalty *= 0.65
        else:
            penalty *= 0.0

    if common_facts.get("outside_route", False):
        penalty *= 0.9

    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
        "breakdown": {
            "ebike_decelerate": private_facts.get("ebike_decelerate", False),
            "pedestrian_stop": private_facts.get("pedestrian_stop", False),
            "resume_route": private_facts.get("resume_route", False),
            "collision": common_facts.get("collision", False),
            "min_ttc": min_ttc
        }
    }
