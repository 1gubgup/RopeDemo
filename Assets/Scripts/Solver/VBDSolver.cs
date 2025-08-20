using System.Collections.Generic;
using Unity.VisualScripting.Dependencies.Sqlite;
using UnityEngine;

public class VBDSolver : Solver
{
    private Vector3[] y; // 预测位置

    public bool damping = false;
    public float dampingCoefficient = 0.0001f;

    public VBDSolver(List<DistanceConstraint> constraints) : base(constraints)
    {
        BuildVertexToConstraintMap();
    }


    public override void Solve(List<RopeNode> nodes, float dt)
    {
        Vector3 gravity = new Vector3(0, -9.81f, 0);
        if (y == null || y.Length < nodes.Count)
            y = new Vector3[nodes.Count];

        float dt2 = dt * dt;
        float invdt2 = 1f / dt2;

        // 1. 预测位置
        for (int i = 0; i < nodes.Count; i++)
        {
            var node = nodes[i];
            if (node.invMass == 0f)
            {
                y[i] = node.position;
                continue;
            }

            Vector3 a_ext = gravity;

            node.prevPosition = node.position;
            y[i] = node.position + node.velocity * dt + a_ext * dt2;

            // TODO: DCD using x_t

            // Adaptive acceleration
            Vector3 a_prev = (node.velocity - node.prevVelocity) / dt;
            float a_ext_mag = a_ext.magnitude;
            float a_proj = Vector3.Dot(a_prev, a_ext / a_ext_mag);
            float scale = Mathf.Clamp(a_proj / a_ext_mag, 0.0f, 1.0f);
            scale = scale > Mathf.Epsilon ? scale : 0.0f;

            node.position = node.position + node.velocity * dt + scale * a_ext * dt2;
        }

        // 2. 迭代
        for (int iter = 0; iter < iterations; iter++)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                var node = nodes[i];
                if (node.invMass == 0f)
                    continue;

                // 1. 惯性项
                Vector3 grad = -invdt2 * node.mass * (node.position - y[i]);
                Matrix3x3 hessian = invdt2 * node.mass * Matrix3x3.Identity;

                // 2. 约束项
                if (vertexToConstraints.TryGetValue(i, out var constraintIndices))
                {
                    foreach (int j in constraintIndices)
                    {                        
                        var c = constraints[j];
                        RopeNode nodeA = nodes[c.indexA];
                        RopeNode nodeB = nodes[c.indexB];

                        Vector3 delta = nodeA.position - nodeB.position;
                        float dist = delta.magnitude;
                        if (dist < Mathf.Epsilon) continue;

                        float C = dist - c.restLength;
                        Vector3 n = delta / dist;

                        int s = (i == c.indexA) ? +1 : -1;

                        // grad = -∂E/∂xᵢ : - k C n
                        grad += -s * c.stiffnessVBD * C * n;

                        // Hessian = k ( n nᵀ + C / dist * ( I - n nᵀ ) )
                        Matrix3x3 nnT = MatrixUtil.OuterProduct(n);
                        float t = Mathf.Max(C / dist, 0f);   // 这里为了防止 H 非正定，当 C<0 的时候，不计算二阶项，如果强行加上这一项可能会能量爆炸
                        Matrix3x3 Hi = c.stiffnessVBD * (nnT + t * (Matrix3x3.Identity - nnT));

                        hessian += Hi;

                        // 阻尼项（可选）
                        if (damping)
                        {
                            float kd = dampingCoefficient;
                            grad += -(kd / dt) * Hi * (node.position - node.prevPosition);
                            hessian += (kd / dt) * Hi;
                        }
                    }
                }

                // 3. 求解 dx = H⁻¹ * grad
                if (!hessian.TryInverse(out Matrix3x3 hessianInv))
                    continue; // 行列式太小，跳过该粒子的更新

                node.position += hessianInv * grad;
            }

            // TODO: accelerated iteration
        }

        // 3. 更新速度
        for (int i = 0; i < nodes.Count; i++)
        {
            var node = nodes[i];
            if (node.invMass == 0f) continue;
            node.prevVelocity = node.velocity;
            node.velocity = (node.position - node.prevPosition) / dt;
        }
    }
}
