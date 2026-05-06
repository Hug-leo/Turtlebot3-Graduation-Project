import argparse
import csv
import time

from turtlebot3_drl_local_planner.utils.ros_gazebo import load_yaml


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate a trained GAP_SAC policy.")
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", default="evaluation_metrics.csv")
    return parser.parse_args()


def main():
    args = parse_args()
    config = load_yaml(args.config)
    try:
        import rclpy
        from turtlebot3_drl_local_planner.agents.gap_sac import GAPSACAgent
        from turtlebot3_drl_local_planner.envs.gazebo_nav_env import GazeboNavEnv
    except ImportError as exc:
        print(f"Missing dependency for evaluation: {exc}")
        return 2

    rclpy.init()
    node = rclpy.create_node("gap_sac_evaluation")
    env = GazeboNavEnv(node, config)
    obs = env.reset()
    agent = GAPSACAgent(len(obs), config)
    agent.load_checkpoint(args.checkpoint)
    max_steps = int(config.get("algorithm", {}).get("max_steps_per_episode", 500))

    benchmark_stages = []
    for goal in config.get("evaluation", {}).get("benchmark_goals", []):
        benchmark_stages.append({"goal": goal})
    for offset in config.get("evaluation", {}).get("benchmark_local_offsets", []):
        benchmark_stages.append({"local_goal_offset": offset})

    with open(args.output, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "goal_index",
                "goal_mode",
                "goal_x",
                "goal_y",
                "goal_yaw",
                "success",
                "collision",
                "timeout",
                "reward",
                "steps",
                "duration_sec",
            ],
        )
        writer.writeheader()
        for goal_index, stage in enumerate(benchmark_stages):
            env.set_stage(stage)
            state = env.reset()
            goal = env.goal
            total_reward = 0.0
            terminal_reason = "timeout"
            start = time.time()
            for step in range(max_steps):
                action = agent.select_action(state, deterministic=True)
                state, reward, done, info = env.step(action)
                total_reward += reward
                if done:
                    terminal_reason = info.get("terminal_reason", "")
                    break
            writer.writerow(
                {
                    "goal_index": goal_index,
                    "goal_mode": env.last_goal_mode,
                    "goal_x": goal.get("x", 0.0),
                    "goal_y": goal.get("y", 0.0),
                    "goal_yaw": goal.get("yaw", 0.0),
                    "success": terminal_reason == "goal",
                    "collision": terminal_reason == "collision",
                    "timeout": terminal_reason == "timeout",
                    "reward": total_reward,
                    "steps": step + 1,
                    "duration_sec": time.time() - start,
                }
            )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
