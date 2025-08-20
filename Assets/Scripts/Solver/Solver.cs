using System.Collections.Generic;

public abstract class Solver
{
    public int iterations = 10;
    protected List<DistanceConstraint> constraints;

    protected Dictionary<int, List<int>> vertexToConstraints; // 每个粒子对应的约束索引

    public Solver(List<DistanceConstraint> constraints)
    {
        this.constraints = constraints;
    }

    public List<DistanceConstraint> GetConstraints() => constraints;

    public abstract void Solve(List<RopeNode> nodes, float dt);

    public void BuildVertexToConstraintMap()
    {
        vertexToConstraints = new Dictionary<int, List<int>>();
        for (int j = 0; j < constraints.Count; j++)
        {
            var c = constraints[j];
            if (!vertexToConstraints.ContainsKey(c.indexA))
                vertexToConstraints[c.indexA] = new List<int>();
            if (!vertexToConstraints.ContainsKey(c.indexB))
                vertexToConstraints[c.indexB] = new List<int>();

            vertexToConstraints[c.indexA].Add(j);
            vertexToConstraints[c.indexB].Add(j);
        }
    }
}
