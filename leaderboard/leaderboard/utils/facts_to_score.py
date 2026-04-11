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
def score_frontcar_disappear_accident(common_facts, private_facts):
    base_score = 0.0

    # 条件1：自车减速 —— 50分
    if private_facts["slow_down"]:
        base_score += 50.0

    # 条件2：无碰撞 —— 20分
    if private_facts["no_collision"]:
        base_score += 20.0

    # 条件3：安全变道通过路段 —— 30分
    if private_facts["safe_bypass"]:
        base_score += 30.0

    # 碰撞门限（撞了直接 0 分）
    gate = compute_gate(common_facts)
    # 违规 penalty
    penalty = compute_penalty(common_facts)

    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": round(final_score, 2)
    }
# High speed temporary construction
def score_static_barrier(common_facts, private_facts):
    base_score = 0.0

    # 减速：55分
    if private_facts["slow_down"]:
        base_score += 55.0

    # 安全绕行：20分
    if private_facts["safe_bypass"]:
        base_score += 20.0

    # 成功通过路障：25分
    if private_facts["pass_barrier"]:
        base_score += 25.0

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }
# High-speed reckless lane cutting
def score_high_speed_cutting(common_facts, private_facts):
    base_score = 0.0
    has_safe_progress = private_facts.get('safe_bypass') and private_facts.get('resume_route')

    if private_facts.get('brake_response'):
        base_score += 40.0
    if not common_facts.get('collision'):
        base_score += 40.0
    if has_safe_progress:
        base_score += 20.0

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        'base_score': round(base_score, 6),
        'gate': round(gate, 6),
        'penalty': round(penalty, 6),
        'final_score': round(final_score, 6),
        'base_breakdown': {
            'brake_response': 40.0 if private_facts.get('brake_response') else 0.0,
            'collision_free': 40.0 if not common_facts.get('collision') else 0.0,
            'safe_bypass': 1.0 if private_facts.get('safe_bypass') else 0.0,
            'resume_route': 1.0 if private_facts.get('resume_route') else 0.0,
            'safe_progress': 20.0 if has_safe_progress else 0.0,
        }

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
def compute_lane_closure_score(common_facts, private_facts):
    base_score = 0.0

    if private_facts.get("deceleration_detected", False) and not common_facts.get("collision", False):
        base_score += 90.0  

    if common_facts.get("route_completed", False):
        base_score += 10.0 

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)

    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }
# Drive into the roundabout
def score_roundabout_merge_conflict(common_facts, private_facts):
    """
    计算大转盘交互场景得分:
    识别并减速: 55
    安全汇入: 20
    让行内圈车队: 25
    """
    base_score = 0.0

    if private_facts["decelerate_response"]:
        base_score += 55.0
    if private_facts["safe_merge"]:
        base_score += 20.0
    if private_facts["yield_convoy"]:
        base_score += 25.0

    # 获取通用的碰撞拦截(Gate)和惩罚(Penalty)
    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts) 
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }
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
def score_broken_down_vehicle(common_facts, private_facts):
    base_score = 0.0

    if private_facts["brake_response"]:
        base_score += 40.0
    if private_facts["safe_bypass"]:
        base_score += 40.0
    if private_facts["resume_route"]:
        base_score += 20.0

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }

# Slanted motor and children
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

    gate = compute_gate(common_facts)
    penalty = compute_penalty(common_facts)
    final_score = base_score * gate * penalty

    return {
        "base_score": base_score,
        "gate": gate,
        "penalty": penalty,
        "final_score": final_score,
    }


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


