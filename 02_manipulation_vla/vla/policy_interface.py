"""TEAM-02: implement this protocol in a local model or remote inference client."""
from typing import Protocol, Sequence
from perception.observation import Observation
from vla.action_adapter import Action


class Policy(Protocol):
    def reset(self, instruction: str) -> None: ...
    def predict(self, observation: Observation) -> Sequence[Action]: ...


def run_policy(robot, policy: Policy, instruction: str, max_steps=1200):
    """One Action per control tick. Replan after every returned action chunk."""
    policy.reset(instruction)
    from vla.action_adapter import apply_action, validate_action
    completed = 0
    try:
        while completed < max_steps:
            chunk = list(policy.predict(robot.get_observation(instruction)))
            if not 1 <= len(chunk) <= robot.control_hz:
                raise ValueError("Policy chunk must contain 1..control_hz actions (<=1 s)")
            for action in chunk:
                validate_action(robot, action)
            for action in chunk:
                apply_action(robot, action)
                robot.step()
                completed += 1
                if completed >= max_steps:
                    break
    finally:
        robot.move_base(0, 0)
        robot.server.push_command(robot.command)
