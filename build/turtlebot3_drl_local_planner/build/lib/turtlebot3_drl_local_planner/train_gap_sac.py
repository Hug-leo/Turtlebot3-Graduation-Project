import argparse
import shutil
import csv
import os
import time
from datetime import datetime

import numpy as np
import yaml

from turtlebot3_drl_local_planner.utils.ros_gazebo import load_yaml


def parse_args():
    parser = argparse.ArgumentParser(description="Train GAP_SAC in Gazebo.")
    parser.add_argument("--config", required=True, help="Path to gap_sac.yaml")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument(
        "--resume-checkpoint",
        default="",
        help="Optional checkpoint path to load before continuing training.",
    )
    parser.add_argument("--curriculum", default="", help="Override curriculum mode: true or false.")
    parser.add_argument("--start-stage", default="", help="Curriculum stage name to start from.")
    parser.add_argument("--dry-run", action="store_true", help="Initialize components without running episodes.")
    return parser.parse_args()


def str_to_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def as_float(value, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def average(rows, key: str) -> float:
    if not rows:
        return 0.0
    return float(sum(as_float(row.get(key, 0.0)) for row in rows) / len(rows))


def minimum(rows, key: str) -> float:
    values = [as_float(row.get(key, np.inf), np.inf) for row in rows]
    values = [value for value in values if np.isfinite(value)]
    return float(min(values)) if values else float("inf")


def rate(rows, key: str) -> float:
    if not rows:
        return 0.0
    return float(sum(bool(row.get(key, False)) for row in rows) / len(rows))


def resolve_curriculum(args, config):
    training_cfg = config.get("training", {})
    if training_cfg.get("mode") == "my_map_hard_local_path_following" and training_cfg.get("hard_local_goal_curriculum", {}).get("enabled", False):
        curriculum_cfg = training_cfg.get("hard_local_goal_curriculum", {})
    elif training_cfg.get("mode") == "local_path_following" and training_cfg.get("local_goal_curriculum", {}).get("enabled", False):
        curriculum_cfg = training_cfg.get("local_goal_curriculum", {})
    else:
        curriculum_cfg = training_cfg.get("curriculum", {})
    enabled = bool(curriculum_cfg.get("enabled", False))
    if args.curriculum.strip():
        enabled = str_to_bool(args.curriculum)

    if enabled:
        stages = list(curriculum_cfg.get("stages", []))
    else:
        stages = [
            {
                "name": "fixed_goal",
                "goal": training_cfg.get("fixed_goal", {"x": 1.0, "y": 0.0, "yaw": 0.0}),
                "min_episodes": 0,
                "max_episodes": 10**9,
            }
        ]

    if not stages:
        raise ValueError("training.curriculum.stages must contain at least one stage when curriculum is enabled")

    if enabled:
        start_stage = args.start_stage.strip() or curriculum_cfg.get("start_stage", stages[0].get("name", "stage_0"))
    else:
        start_stage = args.start_stage.strip() or stages[0].get("name", "fixed_goal")
    stage_names = [stage.get("name", f"stage_{index}") for index, stage in enumerate(stages)]
    if start_stage not in stage_names:
        raise ValueError(f"Unknown start stage '{start_stage}'. Available stages: {', '.join(stage_names)}")

    return enabled, curriculum_cfg, stages, stage_names.index(start_stage)


def stage_goal(stage):
    sequence = stage.get("local_goal_sequence") or []
    if not sequence and stage.get("local_goal_sequences"):
        sequence = stage.get("local_goal_sequences", [[]])[0]
    if sequence:
        offset = sequence[-1]
        return {
            "x": float(offset.get("dx", 0.0)),
            "y": float(offset.get("dy", 0.0)),
            "yaw": float(offset.get("dyaw", 0.0)),
        }
    if stage.get("local_goal_offset"):
        offset = stage.get("local_goal_offset", {})
        return {
            "x": float(offset.get("dx", 0.0)),
            "y": float(offset.get("dy", 0.0)),
            "yaw": float(offset.get("dyaw", 0.0)),
        }
    goal = stage.get("goal", {})
    return {
        "x": float(goal.get("x", 0.0)),
        "y": float(goal.get("y", 0.0)),
        "yaw": float(goal.get("yaw", 0.0)),
    }


def summarize_stage(stage_index, stage, rows, start_episode, end_episode, status):
    goal = stage_goal(stage)
    episodes = len(rows)
    return {
        "stage_index": stage_index,
        "stage_name": stage.get("name", f"stage_{stage_index}"),
        "goal_x": average(rows, "goal_x") if rows else goal["x"],
        "goal_y": average(rows, "goal_y") if rows else goal["y"],
        "goal_yaw": average(rows, "goal_yaw") if rows else goal["yaw"],
        "start_episode": start_episode,
        "end_episode": end_episode,
        "episodes": episodes,
        "successes": sum(bool(row.get("success", False)) for row in rows),
        "collisions": sum(bool(row.get("collision", False)) for row in rows),
        "timeouts": sum(bool(row.get("timeout", False)) for row in rows),
        "success_rate": rate(rows, "success"),
        "collision_rate": rate(rows, "collision"),
        "avg_reward": average(rows, "episode_reward"),
        "avg_steps": average(rows, "steps"),
        "avg_final_DG": average(rows, "final_DG"),
        "avg_min_obstacle_distance": average(rows, "min_obstacle_distance"),
        "min_obstacle_distance": minimum(rows, "min_obstacle_distance"),
        "avg_close_obstacle_steps": average(rows, "close_obstacle_steps"),
        "avg_narrow_gap_steps": average(rows, "narrow_gap_steps"),
        "avg_waypoints_reached": average(rows, "waypoints_reached"),
        "exposure_rate": rate(rows, "obstacle_exposure_met"),
        "eval_success_rate": average(rows, "eval_success_rate"),
        "eval_collision_rate": average(rows, "eval_collision_rate"),
        "goal_mode": rows[-1].get("goal_mode", "") if rows else "",
        "status": status,
    }


def is_better_score(candidate, current) -> bool:
    if current is None:
        return True
    return candidate > current


def run_deterministic_eval(agent, env, stage, max_steps: int, episodes: int) -> dict:
    if episodes <= 0:
        return {"success_rate": 1.0, "collision_rate": 0.0, "avg_final_DG": 0.0}
    results = []
    previous_stage = env.current_stage
    stage_max_steps = int(stage.get("max_steps_per_episode", max_steps))
    for _ in range(episodes):
        env.set_stage(stage)
        state = env.reset()
        terminal_reason = "timeout"
        info = {"final_DG": np.inf}
        for _step in range(stage_max_steps):
            action = agent.select_action(state, deterministic=True)
            state, _reward, done, info = env.step(action)
            if done:
                terminal_reason = info.get("terminal_reason", "")
                break
        results.append(
            {
                "success": terminal_reason == "goal",
                "collision": terminal_reason == "collision",
                "final_DG": info.get("final_DG", np.inf),
            }
        )
    env.set_stage(previous_stage)
    return {
        "success_rate": rate(results, "success"),
        "collision_rate": rate(results, "collision"),
        "avg_final_DG": average(results, "final_DG"),
    }


def main():
    args = parse_args()
    config = load_yaml(args.config)
    try:
        import rclpy
        from turtlebot3_drl_local_planner.agents.gap_sac import GAPSACAgent, Transition
        from turtlebot3_drl_local_planner.envs.gazebo_nav_env import GazeboNavEnv
    except ImportError as exc:
        print(f"Missing dependency for training: {exc}")
        print("Install PyTorch before running GAP_SAC training.")
        return 2

    rclpy.init()
    node = rclpy.create_node("gap_sac_training")
    env = GazeboNavEnv(node, config)
    obs = env.reset()
    agent = GAPSACAgent(len(obs), config)
    if args.resume_checkpoint:
        agent.load_checkpoint(args.resume_checkpoint, map_location="cpu")
        print(f"[train] resumed checkpoint: {args.resume_checkpoint}")
    if args.dry_run:
        print(f"dry-run ok: state_dim={len(obs)}")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(config.get("training", {}).get("checkpoint_dir", "runs"), f"gap_sac_{timestamp}")
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, "metrics.csv")
    summary_path = os.path.join(run_dir, "stage_summary.csv")
    manifest_path = os.path.join(run_dir, "run_manifest.yaml")
    episodes = args.episodes or int(config.get("algorithm", {}).get("episodes", 1200))
    max_steps = int(config.get("algorithm", {}).get("max_steps_per_episode", 500))
    curriculum_enabled, curriculum_cfg, stages, stage_index = resolve_curriculum(args, config)
    promotion_window = int(curriculum_cfg.get("promotion_window", 20))
    min_success_rate = float(curriculum_cfg.get("min_success_rate", 0.8))
    max_collision_rate = float(curriculum_cfg.get("max_collision_rate", 0.05))
    default_min_exposure_rate = float(curriculum_cfg.get("min_obstacle_exposure_rate", 0.0))
    default_eval_episodes = int(curriculum_cfg.get("eval_episodes_on_promotion", 0))
    default_min_eval_success_rate = float(curriculum_cfg.get("min_eval_success_rate", min_success_rate))
    save_every = int(config.get("training", {}).get("save_every_episodes", 25))

    with open(manifest_path, "w", encoding="utf-8") as manifest:
        yaml.safe_dump(
            {
                "created_at": timestamp,
                "config": args.config,
                "episodes": episodes,
                "max_steps_per_episode": max_steps,
                "resume_checkpoint": args.resume_checkpoint,
                "curriculum_enabled": curriculum_enabled,
                "start_stage": stages[stage_index].get("name", f"stage_{stage_index}"),
                "promotion_window": promotion_window,
                "min_success_rate": min_success_rate,
                "max_collision_rate": max_collision_rate,
                "min_obstacle_exposure_rate": default_min_exposure_rate,
                "eval_episodes_on_promotion": default_eval_episodes,
                "min_eval_success_rate": default_min_eval_success_rate,
                "stages": stages,
            },
            manifest,
            sort_keys=False,
        )

    metrics_fields = [
        "episode",
        "stage_index",
        "stage_name",
        "goal_x",
        "goal_y",
        "goal_yaw",
        "goal_mode",
        "local_goal_dx",
        "local_goal_dy",
        "local_goal_dyaw",
        "start_x",
        "start_y",
        "start_yaw",
        "episode_reward",
        "success",
        "collision",
        "timeout",
        "path_length",
        "min_obstacle_distance",
        "initial_DG",
        "final_DG",
        "min_DG",
        "goal_progress",
        "close_obstacle_steps",
        "narrow_gap_steps",
        "waypoints_reached",
        "waypoint_count",
        "left_min_obstacle_distance",
        "right_min_obstacle_distance",
        "obstacle_exposure_met",
        "rolling_obstacle_exposure_rate",
        "eval_success_rate",
        "eval_collision_rate",
        "steps",
        "rolling_success_rate",
        "rolling_collision_rate",
        "rolling_avg_steps",
        "rolling_avg_reward",
        "stage_status",
        "actor_loss",
        "critic_loss",
        "alpha",
    ]
    summary_fields = [
        "stage_index",
        "stage_name",
        "goal_x",
        "goal_y",
        "goal_yaw",
        "start_episode",
        "end_episode",
        "episodes",
        "successes",
        "collisions",
        "timeouts",
        "success_rate",
        "collision_rate",
        "avg_reward",
        "avg_steps",
        "avg_final_DG",
        "avg_min_obstacle_distance",
        "min_obstacle_distance",
        "avg_close_obstacle_steps",
        "avg_narrow_gap_steps",
        "avg_waypoints_reached",
        "exposure_rate",
        "eval_success_rate",
        "eval_collision_rate",
        "goal_mode",
        "status",
    ]

    best_score = None
    stage_rows = []
    stage_start_episode = 0
    final_episode = -1
    stop_reason = "episode_limit"
    completed_curriculum = False

    with open(log_path, "w", newline="", encoding="utf-8") as stream, open(
        summary_path, "w", newline="", encoding="utf-8"
    ) as summary_stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=metrics_fields,
        )
        summary_writer = csv.DictWriter(summary_stream, fieldnames=summary_fields)
        writer.writeheader()
        summary_writer.writeheader()
        for episode in range(episodes):
            episode_start = time.time()
            stage = stages[stage_index]
            stage_max_steps = int(stage.get("max_steps_per_episode", max_steps))
            env.set_stage(stage)
            state = env.reset()
            goal = {
                "x": float(env.goal.get("x", 0.0)),
                "y": float(env.goal.get("y", 0.0)),
                "yaw": float(env.goal.get("yaw", 0.0)),
            }
            start_pose = env.current_start_pose or {}
            local_offset = env.last_local_goal_offset or {}
            total_reward = 0.0
            last_losses = {}
            terminal_reason = "timeout"
            info = {
                "path_length": 0.0,
                "min_obstacle_distance": np.inf,
                "initial_DG": np.inf,
                "final_DG": np.inf,
                "min_DG": np.inf,
            }
            for step in range(stage_max_steps):
                action = agent.select_action(state)
                next_state, reward, done, info = env.step(action)
                if step == stage_max_steps - 1 and not done:
                    terminal_reason = "timeout"
                    info["terminal_reason"] = "timeout"
                    if config.get("reward", {}).get("use_timeout_penalty", False):
                        reward += float(config.get("reward", {}).get("timeout_penalty", -40.0))
                    done = True
                agent.add_transition(Transition(state, action, reward, next_state, done))
                last_losses = agent.update() or last_losses
                total_reward += reward
                state = next_state
                if done:
                    terminal_reason = info.get("terminal_reason", "")
                    break
            rolling_rows = stage_rows[-max(promotion_window - 1, 0) :] if promotion_window > 1 else []
            row_seed = {
                "episode_reward": total_reward,
                "success": terminal_reason == "goal",
                "collision": terminal_reason == "collision",
                "timeout": terminal_reason == "timeout",
                "steps": step + 1,
                "final_DG": info.get("final_DG", np.inf),
                "obstacle_exposure_met": False,
                "waypoints_reached": info.get("waypoints_reached", 0),
                "waypoint_count": info.get("waypoint_count", 0),
            }
            exposure_distance = float(stage.get("exposure_distance", np.inf))
            min_exposure_steps = int(stage.get("min_exposure_steps", 0))
            min_narrow_gap_steps = int(stage.get("min_narrow_gap_steps", 0))
            min_waypoints_reached = int(stage.get("min_waypoints_reached", 0))
            exposure_met = True
            if min_exposure_steps > 0:
                exposure_met = exposure_met and int(info.get("close_obstacle_steps", 0)) >= min_exposure_steps
            if min_narrow_gap_steps > 0:
                exposure_met = exposure_met and int(info.get("narrow_gap_steps", 0)) >= min_narrow_gap_steps
            if min_waypoints_reached > 0:
                exposure_met = exposure_met and int(info.get("waypoints_reached", 0)) >= min_waypoints_reached
            if np.isfinite(exposure_distance):
                exposure_met = exposure_met and float(info.get("min_obstacle_distance", np.inf)) <= exposure_distance
            row_seed["obstacle_exposure_met"] = exposure_met
            rolling_rows = rolling_rows + [row_seed]
            row = {
                "episode": episode,
                "stage_index": stage_index,
                "stage_name": stage.get("name", f"stage_{stage_index}"),
                "goal_x": goal["x"],
                "goal_y": goal["y"],
                "goal_yaw": goal["yaw"],
                "goal_mode": env.last_goal_mode,
                "local_goal_dx": local_offset.get("dx", 0.0),
                "local_goal_dy": local_offset.get("dy", 0.0),
                "local_goal_dyaw": local_offset.get("dyaw", 0.0),
                "start_x": start_pose.get("x", ""),
                "start_y": start_pose.get("y", ""),
                "start_yaw": start_pose.get("yaw", ""),
                "episode_reward": total_reward,
                "success": terminal_reason == "goal",
                "collision": terminal_reason == "collision",
                "timeout": terminal_reason == "timeout",
                "path_length": info.get("path_length", 0.0),
                "min_obstacle_distance": info.get("min_obstacle_distance", np.inf),
                "initial_DG": info.get("initial_DG", np.inf),
                "final_DG": info.get("final_DG", np.inf),
                "min_DG": info.get("min_DG", np.inf),
                "goal_progress": info.get("initial_DG", np.inf) - info.get("final_DG", np.inf),
                "close_obstacle_steps": info.get("close_obstacle_steps", 0),
                "narrow_gap_steps": info.get("narrow_gap_steps", 0),
                "waypoints_reached": info.get("waypoints_reached", 0),
                "waypoint_count": info.get("waypoint_count", 0),
                "left_min_obstacle_distance": info.get("left_min_obstacle_distance", np.inf),
                "right_min_obstacle_distance": info.get("right_min_obstacle_distance", np.inf),
                "obstacle_exposure_met": exposure_met,
                "rolling_obstacle_exposure_rate": rate(rolling_rows, "obstacle_exposure_met"),
                "eval_success_rate": "",
                "eval_collision_rate": "",
                "steps": step + 1,
                "rolling_success_rate": rate(rolling_rows, "success"),
                "rolling_collision_rate": rate(rolling_rows, "collision"),
                "rolling_avg_steps": average(rolling_rows, "steps"),
                "rolling_avg_reward": average(rolling_rows, "episode_reward"),
                "stage_status": "running",
                "actor_loss": last_losses.get("actor_loss", ""),
                "critic_loss": last_losses.get("critic_loss", ""),
                "alpha": last_losses.get("alpha", ""),
            }
            stage_rows.append(row)
            final_episode = episode

            stage_episode_count = len(stage_rows)
            min_stage_episodes = int(stage.get("min_episodes", 0))
            max_stage_episodes = int(stage.get("max_episodes", 10**9))
            recent_rows = stage_rows[-promotion_window:] if promotion_window > 0 else stage_rows
            min_stage_exposure_rate = float(stage.get("min_obstacle_exposure_rate", default_min_exposure_rate))
            exposure_ready = rate(recent_rows, "obstacle_exposure_met") >= min_stage_exposure_rate
            promotion_ready = (
                curriculum_enabled
                and stage_episode_count >= min_stage_episodes
                and len(recent_rows) >= promotion_window
                and rate(recent_rows, "success") >= min_success_rate
                and rate(recent_rows, "collision") <= max_collision_rate
                and exposure_ready
            )
            if promotion_ready:
                eval_episodes = int(stage.get("eval_episodes_on_promotion", default_eval_episodes))
                eval_result = run_deterministic_eval(agent, env, stage, max_steps, eval_episodes)
                row["eval_success_rate"] = eval_result["success_rate"]
                row["eval_collision_rate"] = eval_result["collision_rate"]
                promotion_ready = (
                    eval_result["success_rate"] >= float(stage.get("min_eval_success_rate", default_min_eval_success_rate))
                    and eval_result["collision_rate"] <= max_collision_rate
                )
            stage_failed = curriculum_enabled and stage_episode_count >= max_stage_episodes and not promotion_ready

            if promotion_ready:
                row["stage_status"] = "completed" if stage_index == len(stages) - 1 else "promoted"
            elif stage_failed:
                row["stage_status"] = "failed"

            writer.writerow(row)
            stream.flush()
            duration = time.time() - episode_start
            print(
                f"[train] episode={episode} stage={row['stage_name']} reward={total_reward:.3f} "
                f"steps={step + 1} terminal={terminal_reason or 'running'} "
                f"path={row['path_length']:.3f} min_obs={row['min_obstacle_distance']:.3f} "
                f"DG={row['initial_DG']:.3f}->{row['final_DG']:.3f} min_DG={row['min_DG']:.3f} "
                f"waypoints={row['waypoints_reached']}/{row['waypoint_count']} "
                f"roll_success={row['rolling_success_rate']:.2f} "
                f"roll_exposure={row['rolling_obstacle_exposure_rate']:.2f} status={row['stage_status']} "
                f"elapsed={duration:.2f}s"
            )

            checkpoint_metadata = {
                "episode": episode,
                "stage_index": stage_index,
                "stage_name": row["stage_name"],
                "goal": goal,
                "rolling_success_rate": row["rolling_success_rate"],
                "rolling_obstacle_exposure_rate": row["rolling_obstacle_exposure_rate"],
                "rolling_avg_steps": row["rolling_avg_steps"],
                "rolling_avg_reward": row["rolling_avg_reward"],
                "hard_curriculum_complete": False,
            }
            score = (
                int(stage_index),
                float(row["rolling_success_rate"]),
                float(row["rolling_obstacle_exposure_rate"]),
                -float(row["rolling_avg_steps"]),
                float(row["rolling_avg_reward"]),
            )
            if is_better_score(score, best_score):
                best_score = score
                agent.save_checkpoint(os.path.join(run_dir, "best_checkpoint.pt"), metadata=checkpoint_metadata)

            if episode % save_every == 0:
                agent.save_checkpoint(os.path.join(run_dir, f"checkpoint_ep_{episode}.pt"), metadata=checkpoint_metadata)
                agent.save_checkpoint(os.path.join(run_dir, "latest_checkpoint.pt"), metadata=checkpoint_metadata)

            if promotion_ready or stage_failed:
                status = "completed" if promotion_ready and stage_index == len(stages) - 1 else row["stage_status"]
                summary_writer.writerow(
                    summarize_stage(stage_index, stage, stage_rows, stage_start_episode, episode, status)
                )
                summary_stream.flush()
                if stage_failed:
                    stop_reason = f"stage_failed:{row['stage_name']}"
                    print(f"[train] stage failed: {row['stage_name']} after {stage_episode_count} episodes")
                    stage_rows = []
                    break
                if stage_index == len(stages) - 1:
                    stop_reason = "curriculum_completed"
                    completed_curriculum = True
                    print(f"[train] curriculum completed at stage: {row['stage_name']}")
                    stage_rows = []
                    break
                print(f"[train] promoted {row['stage_name']} -> {stages[stage_index + 1].get('name', stage_index + 1)}")
                stage_index += 1
                stage_rows = []
                stage_start_episode = episode + 1

        if stage_rows:
            summary_writer.writerow(
                summarize_stage(stage_index, stages[stage_index], stage_rows, stage_start_episode, final_episode, stop_reason)
            )
            summary_stream.flush()

        final_checkpoint = os.path.join(run_dir, f"checkpoint_ep_{max(final_episode, 0)}.pt")
        final_metadata = {
            "episode": final_episode,
            "stop_reason": stop_reason,
            "hard_curriculum_complete": completed_curriculum,
            "deployment_ready": completed_curriculum,
        }
        agent.save_checkpoint(final_checkpoint, metadata=final_metadata)
        agent.save_checkpoint(os.path.join(run_dir, "latest_checkpoint.pt"), metadata=final_metadata)
        if completed_curriculum:
            shutil.copy2(final_checkpoint, os.path.join(run_dir, "deployment_checkpoint.pt"))
            print(f"[train] deployment checkpoint saved: {os.path.join(run_dir, 'deployment_checkpoint.pt')}")
        else:
            print("[train] warning: deployment_checkpoint.pt not saved because curriculum did not complete")
        print(f"[train] final checkpoint saved: {final_checkpoint}")
        print(f"[train] stop reason: {stop_reason}")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
