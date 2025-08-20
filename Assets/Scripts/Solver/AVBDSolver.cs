using System;
using System.Collections.Generic;
using UnityEngine;

public class AVBDSolver : Solver
{
    private Vector3[] y; // 预测位置

    public float alpha = 0.75f;
    public float beta = 100000f;
    public float gamma = 0.99f;

    private float[] lambdas;
    private float[] k;
    private float[] C0;

    private float kmin = 1000f;
    private float kmax = 1000000000f;

    public bool damping = false;
    public float dampingCoefficient = 0.0001f;

    public bool postStabilize = false; // TODO

    public AVBDSolver(List<DistanceConstraint> constraints) : base(constraints)
    {
        BuildVertexToConstraintMap();

        lambdas = new float[constraints.Count];
        k = new float[constraints.Count];
        C0 = new float[constraints.Count];

        for (int i = 0; i < constraints.Count; i++)
        {
            lambdas[i] = 0f;
            k[i] = 1000f;
            C0[i] = 0f;
        }
    }

    public override void Solve(List<RopeNode> nodes, float dt)
    {
        // 更新 C0
        for (int j = 0; j < constraints.Count; j++)
        {
            var c = constraints[j];
            RopeNode nodeA = nodes[c.indexA];
            RopeNode nodeB = nodes[c.indexB];

            Vector3 delta = nodeA.position - nodeB.position;
            float dist = delta.magnitude;

            C0[j] = dist - c.restLength;
        }

        Vector3 gravity = new Vector3(0, -9.81f, 0);
        if (y == null || y.Length < nodes.Count)
            y = new Vector3[nodes.Count];

        float dt2 = dt * dt;
        float invdt2 = 1f / dt2;

        // TODO: collision detection / update colorization

        // 1. warm start
        // 预测位置
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

            // Adaptive acceleration
            Vector3 a_prev = (node.velocity - node.prevVelocity) / dt;
            float a_ext_mag = a_ext.magnitude;
            float a_proj = Vector3.Dot(a_prev, a_ext / a_ext_mag);
            float scale = Mathf.Clamp(a_proj / a_ext_mag, 0.0f, 1.0f);
            scale = scale > Mathf.Epsilon ? scale : 0.0f;

            node.position = node.position + node.velocity * dt + scale * a_ext * dt2;
        }

        // 初始化 lambda 和 k
        for (int j = 0; j < constraints.Count; j++)
        {            
            lambdas[j] = lambdas[j] * alpha * gamma;
            k[j] = Mathf.Clamp(k[j] * gamma, kmin, kmax);
            k[j] = Mathf.Min(k[j], constraints[j].stiffnessAVBD);
        }

        // 2. 迭代
        for (int iter = 0; iter < iterations; iter++)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                var node = nodes[i];
                if (node.invMass == 0f)
                    continue;

                // Primal update

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

                        C = c.hard ? C - alpha * C0[j] : C;   // 硬约束的话更新 C TODO: clamp lambda

                        float lambdaPlus = k[j] * C + lambdas[j];

                        // grad = -∂E/∂xᵢ : - ( k C + lambda ) n
                        grad += -s * lambdaPlus * n;

                        // Hessian = k n nᵀ + Gi
                        Matrix3x3 nnT = MatrixUtil.OuterProduct(n);
                        Matrix3x3 Gi = ComputeGi(lambdaPlus, dist, n);
                        Matrix3x3 Hi = k[j] * nnT + Gi;

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
                Vector3 dx = MatrixUtil.LDLSolve(hessian, grad);
                node.position += dx;
            }

            // Dual update

            for (int j = 0; j < constraints.Count; j++)
            {
                var c = constraints[j];
                RopeNode nodeA = nodes[c.indexA];
                RopeNode nodeB = nodes[c.indexB];

                Vector3 delta = nodeA.position - nodeB.position;
                float dist = delta.magnitude;

                float C = dist - c.restLength;

                if (c.hard)
                {
                    C = C - alpha * C0[j];

                    lambdas[j] = k[j] * C + lambdas[j];   // TODO: clamp lambda
                    k[j] = k[j] + beta * Mathf.Abs(C);
                }
                else
                {
                    k[j] = Mathf.Min(c.stiffnessAVBD, k[j] + beta * Mathf.Abs(C));
                }
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

    // AVBDSolver 内部：用 |lambdaPlus| 的对角近似，保证 SPD
    private Matrix3x3 ComputeGi(float lambdaPlus, float dist, Vector3 n)
    {
        float invDist = 1f / Mathf.Max(dist, 1e-8f);   // 这个地方如果 dist 太小能量会巨大 TODO
        // 列范数闭式：|| (I - n n^T)[:,c] || = sqrt(1 - n_c^2)
        float nx2 = n.x * n.x, ny2 = n.y * n.y, nz2 = n.z * n.z;

        float sx = Mathf.Sqrt(Mathf.Max(0f, 1f - nx2));
        float sy = Mathf.Sqrt(Mathf.Max(0f, 1f - ny2));
        float sz = Mathf.Sqrt(Mathf.Max(0f, 1f - nz2));

        float s = Mathf.Abs(lambdaPlus) * invDist; // 一定要取绝对值（或等价）→ SPD
        return new Matrix3x3(
            new Vector3(s * sx, 0f, 0f),
            new Vector3(0f, s * sy, 0f),
            new Vector3(0f, 0f, s * sz)
        );
    }

}
