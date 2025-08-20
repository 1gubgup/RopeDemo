using UnityEngine;

public struct Matrix3x3
{
    public Vector3 col0, col1, col2;

    public static Matrix3x3 Identity => new Matrix3x3(
        new Vector3(1, 0, 0),
        new Vector3(0, 1, 0),
        new Vector3(0, 0, 1)
    );

    public Matrix3x3(Vector3 c0, Vector3 c1, Vector3 c2)
    {
        col0 = c0; col1 = c1; col2 = c2;
    }

    public static Vector3 operator *(Matrix3x3 m, Vector3 v)
    {
        return m.col0 * v.x + m.col1 * v.y + m.col2 * v.z;
    }

    public static Matrix3x3 operator +(Matrix3x3 a, Matrix3x3 b)
    {
        return new Matrix3x3(a.col0 + b.col0, a.col1 + b.col1, a.col2 + b.col2);
    }

    public static Matrix3x3 operator -(Matrix3x3 a, Matrix3x3 b)
    {
        return new Matrix3x3(a.col0 - b.col0, a.col1 - b.col1, a.col2 - b.col2);
    }

    public static Matrix3x3 operator *(float s, Matrix3x3 m)
    {
        return new Matrix3x3(s * m.col0, s * m.col1, s * m.col2);
    }

    public bool TryInverse(out Matrix3x3 inv)
    {
        Vector3 a = col0, b = col1, c = col2;

        Vector3 r0 = Vector3.Cross(b, c);
        Vector3 r1 = Vector3.Cross(c, a);
        Vector3 r2 = Vector3.Cross(a, b);

        float det = Vector3.Dot(a, r0);

        // 更符合实际的行列式大小判断
        float det2 = det * det;
        float adj2 = Mathf.Max(r0.sqrMagnitude, Mathf.Max(r1.sqrMagnitude, r2.sqrMagnitude));
        const float eps = 2e-7f;
        if (adj2 == 0f || det2 <= (eps * eps) * adj2) { inv = Identity; return false; }

        float invDet = 1f / det;
        inv = new Matrix3x3(
            r0 * invDet,
            r1 * invDet,
            r2 * invDet
        );
        return true;
    }
}

public static class MatrixUtil
{
    public static Matrix3x3 OuterProduct(Vector3 n)
    {
        return new Matrix3x3(
            new Vector3(n.x * n.x, n.y * n.x, n.z * n.x), // col0: n * n.x
            new Vector3(n.x * n.y, n.y * n.y, n.z * n.y), // col1: n * n.y
            new Vector3(n.x * n.z, n.y * n.z, n.z * n.z)  // col2: n * n.z
        );
    }

    public static Vector3 LDLSolve(Matrix3x3 A, Vector3 b)
    {
        float D1 = A.col0[0];
        float L21 = A.col1[0] / A.col0[0];
        float L31 = A.col2[0] / A.col0[0];
        float D2 = A.col1[1] - L21 * L21 * D1;
        float L32 = (A.col2[1] - L21 * L31 * D1) / D2;
        float D3 = A.col2[2] - (L31 * L31 * D1 + L32 * L32 * D2);

        float y1 = b.x;
        float y2 = b.y - L21 * y1;
        float y3 = b.z - L31 * y1 - L32 * y2;

        float z1 = y1 / D1;
        float z2 = y2 / D2;
        float z3 = y3 / D3;

        Vector3 x = new Vector3();
        x[2] = z3;
        x[1] = z2 - L32 * x[2];
        x[0] = z1 - L21 * x[1] - L31 * x[2];

        return x;
    }
}
