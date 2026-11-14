import math

import pytest

from objectpilot.stepsim import Noise, evaluate, main, run_episode, stand_check
from objectpilot.stepwise import (StepConfig, StepObservation, StepPlanner, check_command,
                                  fit_wall_normal, summarize)


def obs(box=(.5, .5, .2, .4), tof=None, yaw=None, conf=None):
    return StepObservation(box, 1. if box else 0., tof, yaw, conf)


def test_commands_respect_sdk_ranges():
    assert check_command("forward 20") == ("forward", 20)
    assert check_command("ccw 360") == ("ccw", 360)
    for bad in ("forward 19", "left 501", "cw 0", "cw 361", "go 20 0 0 10", "rc 0 10 0 0", "back"):
        with pytest.raises(ValueError):
            check_command(bad)


def test_summarize_uses_medians_and_requires_half_the_frames():
    frames = [[("cup", .4, .5, .1, .2, .9), ("bag", .9, .5, .1, .2, .95)], [("cup", .6, .5, .1, .2, .8)],
              [("cup", .5, .5, .1, .4, .7)], []]
    o = summarize(frames, [1.02, None, 1.04, 1.06], "cup")
    assert o.box == pytest.approx((.5, .5, .1, .2)) and o.seen == .75
    assert o.tof == pytest.approx(1.02)                     # median 1.04 minus the 2 cm bench offset
    assert summarize(frames, [None, None, 1.0], "bottle").box is None
    assert summarize(frames, [None, None, 1.0], "cup").tof is None


def test_wall_normal_fit_recovers_heading_and_distance():
    points = [(h, .8/math.cos(math.radians(h-7))) for h in (-10, 0, 10, 20, 30)]
    normal, distance = fit_wall_normal(points, 10)
    assert normal == pytest.approx(7, abs=.3) and distance == pytest.approx(.8, abs=.005)


def test_camera_model_matches_the_cup_stand_clips():
    for row in stand_check():
        assert row["h_model"] == pytest.approx(row["h_measured"], rel=.06)
        assert row["w_model"] == pytest.approx(row["w_measured"], rel=.15)


def test_search_turns_in_overlapping_steps_then_gives_up():
    planner = StepPlanner()
    commands = [planner.decide(obs(box=None)).command for _ in range(12)]
    assert commands == ["cw 30"]*12              # 30 degree steps: a target is within 15 degrees of a view center
    for command in commands:
        planner.applied(command, "ok")
    assert planner.decide(obs(box=None)).outcome == "not_found"


def test_box_offset_becomes_a_yaw_angle():
    planner = StepPlanner()
    decision = planner.decide(obs(box=(.75, .5, .05, .1)))
    # ex = 0.25 with a 54 degree FOV: atan(2 * 0.25 * tan 27 deg) = 14 degrees clockwise.
    assert decision.command == "cw 14"


def test_far_target_is_approached_in_large_steps_and_close_one_in_small_steps():
    assert StepPlanner().decide(obs(box=(.5, .4, .05, .12))).command == "forward 50"
    assert StepPlanner().decide(obs(box=(.5, .4, .1, .24))).command == "forward 25"


def test_short_forward_correction_backs_off_first():
    planner = StepPlanner(StepConfig(square=False))
    planner.phase = "final"
    assert planner.decide(obs(tof=.75)).command == "back 20"
    assert planner.decide(obs(tof=.95)).command == "forward 30"


def test_too_close_always_backs_off():
    planner = StepPlanner()
    planner.phase = "final"
    assert planner.decide(obs(tof=.42)).command == "back 20"


def test_squared_planner_strafes_toward_the_box():
    planner = StepPlanner()
    planner.phase, planner.squared, planner.perp_m = "center", True, .9
    decision = planner.decide(obs(box=(.3, .5, .1, .3), tof=.9))
    assert decision.command.startswith("left ")
    assert int(decision.command.split()[1]) <= 40


def test_drift_past_tof_range_steps_closer_only_if_the_box_did_not_grow():
    def squared_final():
        planner = StepPlanner()
        planner.phase, planner.squared, planner.verified, planner.perp_m = "final", True, True, 1.1
        planner.decide(obs(box=(.5, .5, .1, .28), tof=1.10))      # last reading: 1.10 m, box 0.28 tall
        return planner

    planner = squared_final()
    assert planner.decide(obs(box=(.5, .5, .1, .27))).reason == "range_missing"
    assert planner.decide(obs(box=(.5, .5, .1, .27))).command == "forward 20"
    closer = squared_final()
    for _ in range(4):
        decision = closer.decide(obs(box=(.5, .5, .1, .34)))   # box grew: closer, not beyond range
        assert decision.command is None
    assert decision.outcome == "range_unavailable"


def test_target_lost_inside_the_stand_off_backs_off_before_turning():
    planner = StepPlanner()
    planner.phase, planner.squared, planner.verified, planner.perp_m = "final", True, True, .5
    assert planner.decide(obs(box=None, tof=.48)).command == "back 20"
    assert planner.decide(obs(box=None, tof=.62)).command == "back 20"
    third = planner.decide(obs(box=None, tof=.82))          # beyond the stand-off: the usual wait and turn
    assert third.command is None and third.reason == "target_missing"


def test_failed_command_is_sent_again():
    planner = StepPlanner()
    first = planner.decide(obs(box=(.75, .5, .05, .1))).command
    planner.applied(first, "error")
    assert planner.decide(obs(box=(.75, .5, .05, .1))).command == first


def test_noise_free_oblique_start_ends_square_in_front_of_the_print():
    quiet = Noise(move_scale_sd=0, move_cross_sd=0, yaw_scale_sd=0, yaw_sd_deg=0, move_heading_sd_deg=0,
                  fail_prob=0, hover_sd_m=0, pixel_sd=0, size_sd=0, tof_sd_m=0, tof_dropout=0, imu_drift_deg=0)
    for seed in (60002, 60024):            # 26 and 49 degrees off the wall normal
        result = run_episode(seed, noise=quiet)
        assert result["outcome"] == "arrived" and result["squared"]
        assert abs(result["heading_error_deg"]) < 3 and abs(result["lateral_m"]) < .05
        assert abs(result["perpendicular_m"]-.65) < .05


def test_square_up_beats_yaw_only_without_clearance_violations():
    seeds = range(60000, 60060)
    square, yaw_only = evaluate("square", seeds), evaluate("no_square", seeds)
    assert all(r["min_clearance_m"] >= .40 for r in square)
    assert sum(r["success"] for r in square)/len(seeds) >= .85
    assert sum(r["success"] for r in yaw_only) < sum(r["success"] for r in square)/2


def test_cli_refuses_controller_seed_ranges(tmp_path):
    with pytest.raises(SystemExit):
        main(["--episodes", "5", "--seed", "59998", "--out", str(tmp_path/"x")])


def test_weak_search_candidate_is_centered_then_confirmed_or_rejected():
    planner = StepPlanner()
    edge = obs(box=(.85, .3, .07, .12), conf=.30)           # small cup at the frame edge, below 0.45
    assert planner.decide(edge).reason == "search_candidate" and planner.decide(edge).command.startswith("cw ")
    centered_weak = obs(box=(.5, .3, .07, .12), conf=.30)
    assert planner.decide(centered_weak).reason == "search_candidate"        # first centered look: wait
    assert planner.decide(centered_weak).command == "cw 30"                  # second: rejected, search on
    confirmed = StepPlanner()
    assert confirmed.decide(obs(box=(.5, .3, .07, .12), conf=.6)).phase == "face"


def test_turn_shortfall_is_learned_from_the_state_yaw():
    planner = StepPlanner()
    yaw = 0.
    for _ in range(3):                                     # each 30 degree search turn achieves 28
        decision = planner.decide(obs(box=None, yaw=yaw))
        planner.applied(decision.command, "ok")
        yaw += int(decision.command.split()[1])-2
    assert planner.decide(obs(box=None, yaw=yaw)).command == "cw 32"


def test_hold_hysteresis_keeps_a_hold_that_drifts_slightly():
    planner = StepPlanner()
    planner.phase, planner.squared, planner.verified, planner.perp_m = "final", True, True, .65
    planner.normal_heading = planner.heading = 0.
    assert planner.decide(obs(box=(.5, .5, .2, .4), tof=.69, yaw=0.)).reason == "arrival_hold"
    # 6 cm long: outside 5 cm, inside 5 cm x 1.4 while holding.
    assert planner.decide(obs(box=(.5, .5, .2, .4), tof=.71, yaw=0.)).reason == "arrival_hold"


def test_no_strafe_inside_the_strafe_clearance():
    planner = StepPlanner()
    planner.phase, planner.squared, planner.perp_m = "center", True, .55
    decision = planner.decide(obs(box=(.6, .5, .2, .4), tof=.55))       # about 5.6 cm to the right
    assert decision.reason == "clear_before_strafe" and decision.command.startswith("back ")


def test_forward_step_stays_clear_even_if_it_overshoots_by_30_percent():
    planner = StepPlanner()
    step = planner._safe_forward_m(.94)
    assert .94-1.3*step >= planner.c.minimum_m+.03-1e-9
    planner.phase, planner.squared, planner.perp_m = "final", True, .94
    command = planner.decide(obs(box=(.5, .5, .2, .3), tof=.94)).command
    assert command.startswith("forward ") and int(command.split()[1])/100 <= step+.005
