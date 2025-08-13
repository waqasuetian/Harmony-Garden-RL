// AppleInteractable.cs
using UnityEngine;

public class AppleInteractable : Interactable
{
    bool taken = false;

    public override bool Interact(RobotAgent agent)
    {
        if (taken) return false;
        taken = true;
        agent.SetCarryingApple(true);
        agent.AddReward(+0.1f); // small extrinsic, curiosity will add +1 first time
        gameObject.SetActive(false); // “picked”
        return true;
    }

    void Reset() { type = InteractableType.Apple; }
}
