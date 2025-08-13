// WaterfallZone.cs
using UnityEngine;

[RequireComponent(typeof(Collider))]
public class WaterfallZone : Interactable
{
    Collider col;

    void Awake() { col = GetComponent<Collider>(); col.isTrigger = true; }
    void Reset() { type = InteractableType.Waterfall; }

    public override bool Interact(RobotAgent agent)
    {
        // Optional “splash” interaction when Space is pressed near it
        agent.SetPlayingWater(true);
        agent.AddReward(+0.05f);
        return true;
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<RobotAgent>(out var a))
        {
            a.SetPlayingWater(true);
            a.AddReward(+0.05f);
        }
    }
    void OnTriggerExit(Collider other)
    {
        if (other.TryGetComponent<RobotAgent>(out var a)) a.SetPlayingWater(false);
    }
}
