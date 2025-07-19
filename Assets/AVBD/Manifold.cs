// This file is the C# translation of avbd-demo2d-repo/source/manifold.cpp and collide.cpp
// It implements contact manifold and collision logic for the AVBD engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using UnityEngine;
    using System;

    /// <summary>
    /// Implements contact manifold and collision logic (see manifold.cpp, collide.cpp in C++).
    /// </summary>
    public class Manifold : Force
    {
        private const float STICK_THRESH = 0.02f;
        public struct Contact
        {
            public Vector2 normal;
            public Vector2 rA;
            public Vector2 rB;
            public float separation;
            public FeaturePair feature;

            // Extra data from C++ version for Taylor expansion & friction rows
            public Vector3 JAn, JBn; // Jacobians (normal)
            public Vector3 JAt, JBt; // Jacobians (tangent)
            public Vector2 C0;       // Cached constraint values at x- (used for stabilisation)
            public bool stick;       // Was the contact sticking last frame?
        }

        public struct FeaturePair
        {
            public byte inEdge1;
            public byte inEdge2;
            public byte outEdge1;
            public byte outEdge2;

            public byte value;
        }

        public struct ClipVertex
        {
            public Vector2 v;
            public FeaturePair fp;
        }

        public Contact[] contacts = new Contact[2];
        // Cached previous-frame data for warmstarting/static friction
        private Contact[] oldContacts = new Contact[2];
        private float[] oldPenalty = new float[4];
        private float[] oldLambda = new float[4];
        private bool[] oldStick = new bool[2];
        private int oldNumContacts = 0;
        public int numContacts;
        public float friction;

        // Jacobians are stored in base Force.J array

        /// <summary>
        /// Constructs a manifold between two bodies. Mirrors C++ constructor.
        /// </summary>
        public Manifold(Solver solver, RigidBody bodyA, RigidBody bodyB)
            : base(solver, bodyA, bodyB)
        {
            // Initialize force bounds like C++ constructor
            fmax[0] = fmax[2] = 0.0f;
            fmin[0] = fmin[2] = float.NegativeInfinity;
        }

        /// <summary>
        /// Number of constraint rows (matches C++ logic).
        /// </summary>
        public override int Rows() { return numContacts * 2; }

        /// <summary>
        /// Initializes the contact manifold. Mirrors C++ logic.
        /// </summary>
        public override bool Initialize()
        {
            // Compute combined friction coefficient (match C++)
            friction = Mathf.Sqrt(bodyA.friction * bodyB.friction);

            // Store previous frame data locally then we will merge after new collision
            Contact[] prevContacts = new Contact[oldContacts.Length];
            Array.Copy(oldContacts, prevContacts, oldContacts.Length);
            float[] prevPenalty = new float[oldPenalty.Length];
            Array.Copy(oldPenalty, prevPenalty, oldPenalty.Length);
            float[] prevLambda = new float[oldLambda.Length];
            Array.Copy(oldLambda, prevLambda, oldLambda.Length);
            bool[] prevStick = new bool[oldStick.Length];
            Array.Copy(oldStick, prevStick, oldStick.Length);
            int prevNum = oldNumContacts;

            numContacts = Collide(bodyA, bodyB, contacts);
            // Merge old contact data with new contacts (match C++)
            for (int i = 0; i < numContacts; i++)
            {
                int rowN = i * 2;
                int rowT = rowN + 1;
                penalty[rowN] = penalty[rowT] = 0.0f;
                lambda[rowN] = lambda[rowT] = 0.0f;

                for (int j = 0; j < prevNum; j++)
                {
                    if (contacts[i].feature.value == prevContacts[j].feature.value)
                    {
                        // Copy previous penalty/lambda for both rows
                        int prevRowN = j * 2;
                        int prevRowT = prevRowN + 1;
                        penalty[rowN] = prevPenalty[prevRowN];
                        penalty[rowT] = prevPenalty[prevRowT];
                        lambda[rowN] = prevLambda[prevRowN];
                        lambda[rowT] = prevLambda[prevRowT];
                        contacts[i].stick = prevStick[j];
                        // If sticking, reuse rA/rB to reduce jitter
                        if (prevStick[j])
                        { 
                            contacts[i].rA = prevContacts[j].rA;
                            contacts[i].rB = prevContacts[j].rB;
                        }
                    }
                }
            }

            const float COLLISION_MARGIN = 0.01f;

            for (int i = 0; i < numContacts; i++)
            {
                int rowN = i * 2;
                int rowT = rowN + 1;

                stiffness[rowN] = stiffness[rowT] = float.PositiveInfinity;

                // Normal row only pushes (negative in our sign convention)
                fmin[rowN] = -Mathf.Infinity;
                fmax[rowN] = 0.0f;

                // Tangent row bounds set later each step from friction
                fmin[rowT] = -Mathf.Infinity;
                fmax[rowT] = Mathf.Infinity;

                // Pre-compute Jacobians (Eq.15 in paper) identical to C++
                Vector2 n = contacts[i].normal;
                
                Vector2 t = new Vector2(n.y, -n.x);
                // World-space contact offsets
                Matrix2x2 rotA = new Matrix2x2(bodyA.position.z);
                Matrix2x2 rotB = new Matrix2x2(bodyB.position.z);
                Vector2 rAW = rotA * contacts[i].rA;
                Vector2 rBW = rotB * contacts[i].rB;

                // Helper for 2D cross product (scalar)
                float Cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;

                // Precompute Jacobians
                contacts[i].JAn = new Vector3(n.x, n.y, Cross(rAW, n));
                contacts[i].JBn = new Vector3(-n.x, -n.y, -Cross(rBW, n));
                contacts[i].JAt = new Vector3(t.x, t.y, Cross(rAW, t));
                contacts[i].JBt = new Vector3(-t.x, -t.y, -Cross(rBW, t));

                Matrix2x2 basis = new Matrix2x2();
                basis.m00 = n.x; basis.m01 = n.y;
                basis.m10 = t.x; basis.m11 = t.y;

                Vector2 posA = new Vector2(bodyA.position.x, bodyA.position.y);
                Vector2 posB = new Vector2(bodyB.position.x, bodyB.position.y);
                Vector2 delta = posA + rAW - posB - rBW;
                contacts[i].C0 = basis * delta + new Vector2(COLLISION_MARGIN, 0);

                // Save for next frame
                oldContacts[i] = contacts[i];
            }

            // Save per-row arrays for next frame (only up to 4 rows)
            for (int k = 0; k < 4; k++)
            {
                oldPenalty[k] = penalty[k];
                oldLambda[k] = lambda[k];
            }
            for (int k = 0; k < 2; k++) oldStick[k] = contacts.Length>k ? contacts[k].stick : false;
            oldNumContacts = numContacts;

            if (numContacts > 0)
            {
                if (!bodyA.forces.Contains(this)) bodyA.forces.Add(this);
                if (!bodyB.forces.Contains(this)) bodyB.forces.Add(this);
            }

            return numContacts > 0;
        }

        /// <summary>
        /// Computes the constraint value for the manifold. Mirrors C++ logic.
        /// </summary>
        public override void ComputeConstraint(float alpha)
        {
            for (int i = 0; i < numContacts; i++)
            {
                int rowN = i * 2;
                int rowT = rowN + 1;

                // Compute dp of bodies since beginning of step (same as C++)
                Vector3 dpA = bodyA.position - bodyA.initial;
                Vector3 dpB = bodyB.position - bodyB.initial;

                // Normal row (penetration + Baumgarte style stabilization)
                C[rowN] = contacts[i].C0.x * (1 - alpha) + Vector3.Dot(contacts[i].JAn, dpA) + Vector3.Dot(contacts[i].JBn, dpB);

                // Tangent row (relative tangential motion)
                C[rowT] = contacts[i].C0.y * (1 - alpha) + Vector3.Dot(contacts[i].JAt, dpA) + Vector3.Dot(contacts[i].JBt, dpB);

                // Update friction limits from latest normal lambda (static/kinetic friction)
                float frictionBound = Mathf.Abs(lambda[rowN]) * friction;
                fmax[rowT] = frictionBound;
                fmin[rowT] = -frictionBound;

                // Determine if contact is sticking for next frame cache
                contacts[i].stick = Mathf.Abs(lambda[rowT]) < frictionBound && Mathf.Abs(contacts[i].C0.y) < STICK_THRESH;
            }
        }

        /// <summary>
        /// Stores precomputed Jacobians in base Force.J array for the specified body (matches C++ logic).
        /// </summary>
        public override void ComputeDerivatives(RigidBody body)
        {
            // Clear Jacobians first
            for (int i = 0; i < Rows(); i++)
            {
                J[i] = Vector3.zero;
            }

            // Compute Jacobians for this body
            for (int i = 0; i < numContacts; i++)
            {
                if (body == bodyA)
                {
                    J[i * 2 + 0] = contacts[i].JAn;
                    J[i * 2 + 1] = contacts[i].JAt;
                }
                else if (body == bodyB)
                {
                    J[i * 2 + 0] = contacts[i].JBn;
                    J[i * 2 + 1] = contacts[i].JBt;
                }
            }
        }

        // Jacobians are now stored in base Force.J array via ComputeDerivatives

        

        /// <summary>
        /// Flips the feature pair (utility function, see C++).
        /// </summary>
        private static void Flip(ref FeaturePair fp)
        {
            byte temp = fp.inEdge1;
            fp.inEdge1 = fp.inEdge2;
            fp.inEdge2 = temp;

            temp = fp.outEdge1;
            fp.outEdge1 = fp.outEdge2;
            fp.outEdge2 = temp;
        }

        /// <summary>
        /// Clips a segment to a line (utility function, see C++).
        /// </summary>
        private static int ClipSegmentToLine(ClipVertex[] vOut, ClipVertex[] vIn,
            Vector2 normal, float offset, byte clipEdge)
        {
            // Start with no output points
            int numOut = 0;

            // Calculate the distance of end points to the line
            float distance0 = Vector2.Dot(normal, vIn[0].v) - offset;
            float distance1 = Vector2.Dot(normal, vIn[1].v) - offset;

            // If the points are behind the plane
            if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
            if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

            // If the points are on different sides of the plane
            if (distance0 * distance1 < 0.0f)
            {
                // Find intersection point of edge and plane
                float interp = distance0 / (distance0 - distance1);
                vOut[numOut].v = vIn[0].v + (vIn[1].v - vIn[0].v) * interp;
                if (distance0 > 0.0f)
                {
                    vOut[numOut].fp = vIn[0].fp;
                    vOut[numOut].fp.inEdge1 = clipEdge;
                    vOut[numOut].fp.inEdge2 = 0; // NO_EDGE
                }
                else
                {
                    vOut[numOut].fp = vIn[1].fp;
                    vOut[numOut].fp.outEdge1 = clipEdge;
                    vOut[numOut].fp.outEdge2 = 0; // NO_EDGE
                }
                ++numOut;
            }

            return numOut;
        }

        /// <summary>
        /// Computes the incident edge for collision (utility function, see C++).
        /// </summary>
        private static void ComputeIncidentEdge(ClipVertex[] c, Vector2 h, Vector2 pos,
            Matrix2x2 Rot, Vector2 normal)
        {
            // The normal is from the reference box. Convert it
            // to the incident boxe's frame and flip sign.
            Matrix2x2 RotT = Matrix2x2.Transpose(Rot);
            Vector2 n = -(RotT * normal);
            Vector2 nAbs = new Vector2(Mathf.Abs(n.x), Mathf.Abs(n.y));

            if (nAbs.x > nAbs.y)
            {
                if (Mathf.Sign(n.x) > 0.0f)
                {
                    c[0].v = new Vector2(h.x, -h.y);
                    c[0].fp.inEdge2 = 3; // EDGE3
                    c[0].fp.outEdge2 = 4; // EDGE4

                    c[1].v = new Vector2(h.x, h.y);
                    c[1].fp.inEdge2 = 4; // EDGE4
                    c[1].fp.outEdge2 = 1; // EDGE1
                }
                else
                {
                    c[0].v = new Vector2(-h.x, h.y);
                    c[0].fp.inEdge2 = 1; // EDGE1
                    c[0].fp.outEdge2 = 2; // EDGE2

                    c[1].v = new Vector2(-h.x, -h.y);
                    c[1].fp.inEdge2 = 2; // EDGE2
                    c[1].fp.outEdge2 = 3; // EDGE3
                }
            }
            else
            {
                if (Mathf.Sign(n.y) > 0.0f)
                {
                    c[0].v = new Vector2(h.x, h.y);
                    c[0].fp.inEdge2 = 4; // EDGE4
                    c[0].fp.outEdge2 = 1; // EDGE1

                    c[1].v = new Vector2(-h.x, h.y);
                    c[1].fp.inEdge2 = 1; // EDGE1
                    c[1].fp.outEdge2 = 2; // EDGE2
                }
                else
                {
                    c[0].v = new Vector2(-h.x, -h.y);
                    c[0].fp.inEdge2 = 2; // EDGE2
                    c[0].fp.outEdge2 = 3; // EDGE3

                    c[1].v = new Vector2(h.x, -h.y);
                    c[1].fp.inEdge2 = 3; // EDGE3
                    c[1].fp.outEdge2 = 4; // EDGE4
                }
            }

            c[0].v = pos + Rot * c[0].v;
            c[1].v = pos + Rot * c[1].v;
        }

        /// <summary>
        /// Computes collision contacts between two bodies. Mirrors C++ logic in collide.cpp.
        /// </summary>
        public static int Collide(RigidBody bodyA, RigidBody bodyB, Contact[] contacts)
        {
            Vector2 normal;

            // Setup
            Vector2 hA = bodyA.size * 0.5f;
            Vector2 hB = bodyB.size * 0.5f;
            // Debug.Log($"Manifold.Collide: hA: {hA}, hB: {hB}");

            Vector2 posA = new Vector2(bodyA.position.x, bodyA.position.y);
            Vector2 posB = new Vector2(bodyB.position.x, bodyB.position.y);

            Matrix2x2 RotA = new Matrix2x2(bodyA.position.z);
            Matrix2x2 RotB = new Matrix2x2(bodyB.position.z);

            Matrix2x2 RotAT = Matrix2x2.Transpose(RotA);
            Matrix2x2 RotBT = Matrix2x2.Transpose(RotB);

            Vector2 dp = posB - posA;
            Vector2 dA = RotAT * dp;
            Vector2 dB = RotBT * dp;
            // Debug.Log($"Manifold.Collide: dA: {dA}, dB: {dB}");

            Matrix2x2 C = RotAT * RotB;
            Matrix2x2 absC = new Matrix2x2(0);
            absC.m00 = Mathf.Abs(C.m00); absC.m01 = Mathf.Abs(C.m01);
            absC.m10 = Mathf.Abs(C.m10); absC.m11 = Mathf.Abs(C.m11);
            // Debug.Log($"Manifold.Collide: absC: {absC.m00}, {absC.m01}, {absC.m10}, {absC.m11}");

            Matrix2x2 absCT = Matrix2x2.Transpose(absC);
            // Debug.Log($"Manifold.Collide: absCT: {absCT.m00}, {absCT.m01}, {absCT.m10}, {absCT.m11}");

            Vector2 absCHB = absC * hB;
            // Debug.Log($"Manifold.Collide: absC * hB: {absCHB}");

            Vector2 absCTHA = absCT * hA;
            // Debug.Log($"Manifold.Collide: absCT * hA: {absCTHA}");

            // Box A faces
            Vector2 faceA = new Vector2(
                Mathf.Abs(dA.x) - hA.x - absCHB.x,
                Mathf.Abs(dA.y) - hA.y - absCHB.y);

            // Debug.Log($"Manifold.Collide: faceA: {faceA}");
            if (faceA.x > 0.0f || faceA.y > 0.0f)
            {
                // Debug.Log($"Manifold.Collide: faceA check failed. faceA: {faceA}");
                return 0;
            }

            // Box B faces
            Vector2 faceB = new Vector2(
                Mathf.Abs(dB.x) - absCTHA.x - hB.x,
                Mathf.Abs(dB.y) - absCTHA.y - hB.y);

            // Debug.Log($"Manifold.Collide: faceB: {faceB}");
            if (faceB.x > 0.0f || faceB.y > 0.0f)
            {
                // Debug.Log($"Manifold.Collide: faceB check failed. faceB: {faceB}");
                return 0;
            }

            // Find best axis
            Axis axis;
            float separation;

            // Box A faces
            axis = Axis.FACE_A_X;
            separation = faceA.x;
            if (dA.x > 0.0f) normal = RotA.GetColumn(0);
            else normal = -RotA.GetColumn(0);

            const float relativeTol = 0.95f;
            const float absoluteTol = 0.01f;

            if (faceA.y > relativeTol * separation + absoluteTol * hA.y)
            {
                axis = Axis.FACE_A_Y;
                separation = faceA.y;
                if (dA.y > 0.0f) normal = RotA.GetColumn(1);
                else normal = -RotA.GetColumn(1);
            }

            // Box B faces
            if (faceB.x > relativeTol * separation + absoluteTol * hB.x)
            {
                axis = Axis.FACE_B_X;
                separation = faceB.x;
                if (dB.x > 0.0f) normal = RotB.GetColumn(0);
                else normal = -RotB.GetColumn(0);
            }

            if (faceB.y > relativeTol * separation + absoluteTol * hB.y)
            {
                axis = Axis.FACE_B_Y;
                separation = faceB.y;
                if (dB.y > 0.0f) normal = RotB.GetColumn(1);
                else normal = -RotB.GetColumn(1);
            }

            // Setup clipping plane data based on the separating axis
            Vector2 frontNormal = Vector2.zero;
            Vector2 sideNormal = Vector2.zero;
            ClipVertex[] incidentEdge = new ClipVertex[2];
            float front = 0.0f, negSide = 0.0f, posSide = 0.0f;
            byte negEdge = 0, posEdge = 0;

            // Compute the clipping lines and the line segment to be clipped.
            switch (axis)
            {
                case Axis.FACE_A_X:
                    frontNormal = (dA.x > 0.0f) ? RotA.GetColumn(0) : -RotA.GetColumn(0);
                    front = Vector2.Dot(posA, frontNormal) + hA.x;
                    sideNormal = RotA.GetColumn(1);
                    float sideA = Vector2.Dot(posA, sideNormal);
                    negSide = -sideA + hA.y;
                    posSide = sideA + hA.y;
                    negEdge = 3; // EDGE3
                    posEdge = 1; // EDGE1
                    ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
                    break;

                case Axis.FACE_A_Y:
                    frontNormal = (dA.y > 0.0f) ? RotA.GetColumn(1) : -RotA.GetColumn(1);
                    front = Vector2.Dot(posA, frontNormal) + hA.y;
                    sideNormal = RotA.GetColumn(0);
                    float sideA_Y = Vector2.Dot(posA, sideNormal);
                    negSide = -sideA_Y + hA.x;
                    posSide = sideA_Y + hA.x;
                    negEdge = 2; // EDGE2
                    posEdge = 4; // EDGE4
                    ComputeIncidentEdge(incidentEdge, hB, posB, RotB, frontNormal);
                    break;

                case Axis.FACE_B_X:
                    frontNormal = (dB.x > 0.0f) ? -RotB.GetColumn(0) : RotB.GetColumn(0);
                    front = Vector2.Dot(posB, frontNormal) + hB.x;
                    sideNormal = RotB.GetColumn(1);
                    float sideB = Vector2.Dot(posB, sideNormal);
                    negSide = -sideB + hB.y;
                    posSide = sideB + hB.y;
                    negEdge = 3; // EDGE3
                    posEdge = 1; // EDGE1
                    ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
                    break;

                case Axis.FACE_B_Y:
                    frontNormal = (dB.y > 0.0f) ? -RotB.GetColumn(1) : RotB.GetColumn(1);
                    front = Vector2.Dot(posB, frontNormal) + hB.y;
                    sideNormal = RotB.GetColumn(0);
                    float sideB_Y = Vector2.Dot(posB, sideNormal);
                    negSide = -sideB_Y + hB.x;
                    posSide = sideB_Y + hB.x;
                    negEdge = 2; // EDGE2
                    posEdge = 4; // EDGE4
                    ComputeIncidentEdge(incidentEdge, hA, posA, RotA, frontNormal);
                    break;
            }

            // clip other face with 5 box planes (1 face plane, 4 edge planes)

            ClipVertex[] clipPoints1 = new ClipVertex[2];
            ClipVertex[] clipPoints2 = new ClipVertex[2];
            int np;

            // Debug.Log($"Manifold.Collide: incidentEdge[0].v: {incidentEdge[0].v}, incidentEdge[1].v: {incidentEdge[1].v}");
            // Debug.Log($"Manifold.Collide: -sideNormal: {-sideNormal}, negSide: {negSide}, negEdge: {negEdge}");

            // Clip to box side 1
            np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);
            // Debug.Log($"Manifold.Collide: np after first clip: {np}");

            if (np < 2)
            {
                // Debug.Log($"Manifold.Collide: First clip returned less than 2 points.");
                return 0;
            }

            // Debug.Log($"Manifold.Collide: clipPoints1[0].v: {clipPoints1[0].v}, clipPoints1[1].v: {clipPoints1[1].v}");
            // Debug.Log($"Manifold.Collide: sideNormal: {sideNormal}, posSide: {posSide}, posEdge: {posEdge}");

            // Clip to negative box side 1
            np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);
            // Debug.Log($"Manifold.Collide: np after second clip: {np}");

            if (np < 2)
            {
                // Debug.Log($"Manifold.Collide: Second clip returned less than 2 points.");
                return 0;
            }

            // Now clipPoints2 contains the clipping points.
            // Due to roundoff, it is possible that clipping removes all points.

            int numContacts = 0;
            for (int i = 0; i < 2; ++i)
            {
                float sep = Vector2.Dot(frontNormal, clipPoints2[i].v) - front;

                if (sep <= 0)
                {
                    contacts[numContacts].normal = -normal;
                    contacts[numContacts].separation = sep;

                    // slide contact point onto reference face (easy to cull)
                    contacts[numContacts].rA = Matrix2x2.Transpose(RotA) * (clipPoints2[i].v - frontNormal * sep - posA);
                    contacts[numContacts].rB = Matrix2x2.Transpose(RotB) * (clipPoints2[i].v - posB);
                    contacts[numContacts].feature = clipPoints2[i].fp;

                    if (axis == Axis.FACE_B_X || axis == Axis.FACE_B_Y)
                    {
                        Flip(ref contacts[numContacts].feature);
                        contacts[numContacts].rA = Matrix2x2.Transpose(RotA) * (clipPoints2[i].v - posA);
                        contacts[numContacts].rB = Matrix2x2.Transpose(RotB) * (clipPoints2[i].v - frontNormal * sep - posB);
                    }
                    ++numContacts;
                }
            }
            // Debug.Log($"Manifold.Collide: Final numContacts: {numContacts}");
            return numContacts;
        }
    }

    

    public enum Axis
    {
        FACE_A_X,
        FACE_A_Y,
        FACE_B_X,
        FACE_B_Y
    }

    public enum EdgeNumbers
    {
        NO_EDGE = 0,
        EDGE1,
        EDGE2,
        EDGE3,
        EDGE4
    }
}