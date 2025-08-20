using System.Collections.Generic;
using UnityEngine;

public class XPBDSolver : Solver
{
    private float[] lambdas;

    public XPBDSolver(List<DistanceConstraint> constraints) : base(constraints)
    {
        lambdas = new float[constraints.Count];
    }

    public override void Solve(List<RopeNode> nodes, float dt)
    {
        Vector3 gravity = new Vector3(0, -9.81f, 0);

        // 1. 预测位置
        foreach (var node in nodes)
        {
            if (node.invMass == 0f) continue;
            node.prevPosition = node.position;
            node.velocity += gravity * dt;
            node.position += node.velocity * dt;
        }

        for (int i = 0; i < lambdas.Length; i++) lambdas[i] = 0f;

        // 2. 迭代约束
        float dt2 = dt * dt;
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

                // XPBD 核心公式 Eq.(18)
                float alphaTilde = c.complianceXPBD / dt2;
                float denom = wSum + alphaTilde;
                float deltaLambda = (-C - alphaTilde * lambdas[i]) / denom;

                lambdas[i] += deltaLambda;

                // 位置修正 Eq.(17)
                Vector3 correction = deltaLambda * n;
                nodeA.position += wA * correction;
                nodeB.position -= wB * correction;
            }
        }

        // 3. 更新速度
        foreach (var node in nodes)
        {
            if (node.invMass == 0f) continue;
            node.velocity = (node.position - node.prevPosition) / dt;
        }
    }
}
