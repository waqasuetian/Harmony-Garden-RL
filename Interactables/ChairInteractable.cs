// ChairInteractable.cs
using UnityEngine;

public class ChairInteractable : Interactable
{
    bool occupied = false;

    public override bool Interact(RobotAgent agent)
    {
        occupied = !occupied;
        agent.SetSeated(occupied);
        agent.AddReward(+0.05f);
        return true;
    }

    void Reset() { type = InteractableType.Chair; }
}
