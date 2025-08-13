using UnityEngine;

public enum InteractableType { Button = 0, Apple = 1, Chair = 2, Maze = 3, Waterfall = 4 }

public abstract class Interactable : MonoBehaviour
{
    public InteractableType type;
    public abstract bool Interact(RobotAgent agent); // return true if it did something
}
