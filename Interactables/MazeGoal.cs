// MazeGoal.cs
using UnityEngine;

public class MazeGoal : Interactable
{
    public override bool Interact(RobotAgent agent)
    {
        // If you want the agent to press a goal plate:
        agent.AddReward(+0.5f);
        return true;
    }

    void Reset() { type = InteractableType.Maze; }
}
