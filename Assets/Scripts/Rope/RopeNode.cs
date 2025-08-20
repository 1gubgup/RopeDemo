using UnityEngine;

public class RopeNode
{
    public Vector3 position;
    public Vector3 prevPosition;
    public Vector3 velocity;
    public Vector3 prevVelocity;
    public float mass;
    public float invMass;

    public RopeNode(Vector3 pos, float m)
    {
        position = pos;
        prevPosition = pos;
        velocity = Vector3.zero;
        prevVelocity = Vector3.zero;
        mass = m;
        invMass = m > 0f ? 1f / m : 0f; // 固定点 mass=0 => invMass=0
    }
}
