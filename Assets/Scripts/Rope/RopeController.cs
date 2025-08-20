using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Android;

public class RopeController : MonoBehaviour
{
    [Header("Rope Settings")]
    [Range(1f, 200f)]
    public int iterations = 10;
    private int nodeCount = 20;
    private float segmentLength = 0.1f;
    [Range(0.01f, 1000f)]
    public float heavyMass = 10f; // 最下面的粒子质量
    public GameObject nodePrefab;
    public LineRenderer lineRenderer;

    [Header("PBD")]
    [Range(0f, 1f)]
    public float PBDStiffness = 1f;

    [Header("XPBD")]
    [Range(0f, 0.001f)]
    public float XPBDCompliance = 0f;

    [Header("VBD")]
    [Range(1000f, 100000f)]
    public float VBDStiffness = 5000f;
    public bool VBDDamping = false;

    [Header("AVBD")]
    public bool hard = true;
    [Range(1000f, 100000f)]
    public float AVBDStiffness = 5000f;
    public bool AVBDDamping = false;
    [Range(0f, 1f)]
    public float alpha = 0.75f;
    [Range(10f, 1000000f)]
    public float beta = 100000f;
    [Range(0f, 1f)]
    public float gamma = 0.99f;

    private List<RopeNode> nodes;
    private List<GameObject> nodeObjects;
    private Solver solver;

    public Camera mainCamera;
    private float dragDepth = 5f; // 鼠标投射深度

    void Start()
    {
        nodes = new List<RopeNode>();
        nodeObjects = new List<GameObject>();

        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 pos = transform.position + new Vector3(0, -i * segmentLength, 0);

            float mass;
            if (i == 0) mass = 0f; // 顶端固定
            else if (i == nodeCount - 1) mass = heavyMass; // 最底端重
            else mass = 1f;

            nodes.Add(new RopeNode(pos, mass));

            if (nodePrefab)
            {
                GameObject obj = Instantiate(nodePrefab, pos, Quaternion.identity, transform);

                // 如果是最后一个粒子 -> 放大 + 换颜色
                if (i == nodeCount - 1)
                {
                    obj.transform.localScale *= 2f; // 放大 2 倍
                    Renderer rend = obj.GetComponent<Renderer>();
                    if (rend != null)
                    {
                        rend.material.color = Color.red; // 变红色
                    }
                }

                nodeObjects.Add(obj);
            }
        }

        //solver = new PBDSolver(BuildConstraints());
        //solver = new XPBDSolver(BuildConstraints());
        //solver = new VBDSolver(BuildConstraints());
        solver = new AVBDSolver(BuildConstraints());
    }

    void Update()
    {
        HandleMouseDrag(); // 添加拖拽逻辑

        solver.iterations = iterations;
        nodes[nodeCount - 1].mass = heavyMass;
        nodes[nodeCount - 1].invMass = 1f / heavyMass;

        if (solver != null)
        {
            foreach (var c in solver.GetConstraints())
            {
                c.stiffnessPBD = PBDStiffness;
                c.complianceXPBD = XPBDCompliance;
                c.stiffnessVBD = VBDStiffness;
                c.hard = hard;
                if (hard)
                {
                    c.stiffnessAVBD = 10000000000f;
                }
                else
                {
                    c.stiffnessAVBD = AVBDStiffness;
                }
            }
        }
        if (solver is VBDSolver vbd)
        {
            vbd.damping = VBDDamping;
        }
        else if (solver is AVBDSolver avbd)
        {
            avbd.damping = AVBDDamping;
            avbd.alpha = alpha;
            avbd.beta = beta;
            avbd.gamma = gamma;
        }
    }

    void FixedUpdate()
    {
        solver.Solve(nodes, Time.fixedDeltaTime);
        UpdateVisuals();
    }

    private void UpdateVisuals()
    {
        for (int i = 0; i < nodes.Count; i++)
        {
            if (i < nodeObjects.Count)
                nodeObjects[i].transform.position = nodes[i].position;
        }

        if (lineRenderer != null)
        {
            lineRenderer.positionCount = nodes.Count;
            for (int i = 0; i < nodes.Count; i++)
                lineRenderer.SetPosition(i, nodes[i].position);
        }
    }

    private void HandleMouseDrag()
    {
        if (Input.GetMouseButton(0))
        {
            Vector3 mousePos = Input.mousePosition;
            mousePos.z = dragDepth;
            Vector3 worldPos = mainCamera.ScreenToWorldPoint(mousePos);

            nodes[0].position = worldPos; // 直接修改固定点位置
            nodeObjects[0].transform.position = worldPos; // 更新 prefab 位置
        }
    }

    private List<DistanceConstraint> BuildConstraints()
    {
        var list = new List<DistanceConstraint>();
        for (int i = 0; i < nodeCount - 1; i++)
        {
            list.Add(new DistanceConstraint(i, i + 1, segmentLength));
        }
        return list;
    }
}
