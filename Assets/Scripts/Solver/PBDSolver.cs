using System.Collections.Generic;
using UnityEngine;

public class PBDSolver : Solver
{
    public PBDSolver(List<DistanceConstraint> constraints) : base(constraints) { }

    public override void Solve(List<RopeNode> nodes, float dt)
    {
        Vector3 gravity = new Vector3(0, -9.81f, 0);

        // 1. 预测位置
        foreach (var node in nodes)
        {
            if (node.invMass == 0f) continue;
            node.prevPosition = node.position;
            node.velocity += gravity * dt;
            // TODO: dampVelocities
            node.position += node.velocity * dt;
        }

        // TODO: generateCollisionConstraints

        // 2. 迭代约束
        for (int iter = 0; iter < iterations; iter++)
        {
            for (int i = 0; i < constraints.Count; i++)
            {
                var c = constraints[i];
                RopeNode nodeA = nodes[c.indexA];
                RopeNode nodeB = nodes[c.indexB];

                Vector3 delta = nodeA.position - nodeB.position;
                float dist = delta.magnitude;
                if (dist < Mathf.Epsilon) continue;

                float C = dist - c.restLength;
                Vector3 n = delta / dist;

                float wA = nodeA.invMass;
                float wB = nodeB.invMass;
                float wSum = wA + wB;
                if (wSum == 0f) continue;

                // stiffness compensation
                float stiffness = c.stiffnessPBD; // [0,1]
                float kPrime = 1f - Mathf.Pow(1f - stiffness, 1f / iterations);

                float corr = (C / wSum) * kPrime;
                nodeA.position -= wA * corr * n;
                nodeB.position += wB * corr * n;
            }
        }

        // 3. 更新速度
        foreach (var node in nodes)
        {
            if (node.invMass == 0f) continue;
            node.velocity = (node.position - node.prevPosition) / dt;
        }

        // TODO: velocityUpdate
    }
}
