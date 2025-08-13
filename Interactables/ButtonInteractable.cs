// ButtonInteractable.cs
using UnityEngine;

public class ButtonInteractable : Interactable
{
    public Animator animator; // optional
    public override bool Interact(RobotAgent agent)
    {
        // play animation / toggle light / open gate etc.
        if (animator) animator.SetTrigger("Press");
        // tiny shaping reward to encourage “using” things
        agent.AddReward(+0.05f);
        return true;
    }

    void Reset() { type = InteractableType.Button; }
}
