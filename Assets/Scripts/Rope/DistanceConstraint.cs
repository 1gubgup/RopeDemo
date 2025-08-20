using System.Collections.Generic;
using UnityEngine;

public class DistanceConstraint
{
    public int indexA;
    public int indexB;
    public float restLength;

    // PBD
    public float stiffnessPBD;
    
    //XPBD
    public float complianceXPBD;

    //VBD
    public float stiffnessVBD;

    //AVBD
    public bool hard;
    public float stiffnessAVBD;

    public DistanceConstraint(int a, int b, float length, float stiffnessPBD = 1f, float complianceXPBD = 0f, float stiffnessVBD = 5000f, 
        bool hard = true, float stiffnessAVBD = 5000f)
    {
        indexA = a;
        indexB = b;
        restLength = length;
        this.stiffnessPBD = stiffnessPBD;
        this.complianceXPBD = complianceXPBD;
        this.stiffnessVBD = stiffnessVBD;
        this.hard = hard;
        this.stiffnessAVBD = stiffnessAVBD;
    }
}
