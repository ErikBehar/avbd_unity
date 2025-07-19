// This file is the C# translation of avbd-demo2d-repo/source/spring.cpp
// It implements spring constraints for the AVBD engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using UnityEngine;

    /// <summary>
    /// Implements spring constraints (see spring.cpp in C++).
    /// </summary>
    public class Spring : Force
    {
        public Vector2 rA, rB;
        public float rest;

        /// <summary>
        /// Constructs a spring between two bodies. Mirrors C++ constructor in spring.cpp.
        /// </summary>
        public Spring(Solver solver, RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, float stiffness, float rest = -1)
            : base(solver, bodyA, bodyB)
        {
            this.rA = rA;
            this.rB = rB;
            this.rest = rest;
            this.stiffness[0] = stiffness;
        }

        /// <summary>
        /// Number of constraint rows (matches C++ logic).
        /// </summary>
        public override int Rows() { return 1; }

        /// <summary>
        /// Initializes the spring constraint. Mirrors C++ logic.
        /// </summary>
        public override bool Initialize()
        {
            if (rest < 0)
                rest = (bodyA.Transform(rA) - bodyB.Transform(rB)).magnitude;
            return true;
        }

        /// <summary>
        /// Computes the constraint value for the spring. Mirrors C++ logic.
        /// </summary>
        public override void ComputeConstraint(float alpha)
        {
            C[0] = (bodyA.Transform(rA) - bodyB.Transform(rB)).magnitude - rest;
        }

        /// <summary>
        /// Computes the derivatives for the spring constraint. Mirrors C++ logic.
        /// </summary>
        public override void ComputeDerivatives(RigidBody body)
        {
            Matrix2x2 S = new Matrix2x2(Mathf.PI / 2.0f); // Rotation by 90 degrees
            Matrix2x2 I = new Matrix2x2(0); // Identity matrix
            I.m00 = 1; I.m11 = 1;

            Vector2 d = bodyA.Transform(rA) - bodyB.Transform(rB);
            float dlen2 = Vector2.Dot(d, d);
            if (dlen2 == 0)
                return;
            float dlen = Mathf.Sqrt(dlen2);
            Vector2 n = d / dlen;

            Matrix2x2 dxx = (I - Matrix2x2.Outer(n, n) / dlen2) / dlen;

            if (body == bodyA)
            {
                Vector2 Sr = Maths.Rotate(S * rA, bodyA.position.z);
                Vector2 r = Maths.Rotate(rA, bodyA.position.z);
                Vector2 dxr = dxx * Sr;
                float drr = Vector2.Dot(Sr, dxr) - Vector2.Dot(n, r);

                J[0] = new Vector3(n.x, n.y, Vector2.Dot(n, Sr));
                H[0] = new Matrix3x3(
                    dxx.m00, dxx.m01, dxr.x,
                    dxx.m10, dxx.m11, dxr.y,
                    dxr.x, dxr.y, drr
                );
            }
            else
            {
                Vector2 Sr = Maths.Rotate(S * rB, bodyB.position.z);
                Vector2 r = Maths.Rotate(rB, bodyB.position.z);
                Vector2 dxr = dxx * -Sr;
                float drr = Vector2.Dot(Sr, dxr) + Vector2.Dot(n, r);

                J[0] = new Vector3(-n.x, -n.y, Vector2.Dot(n, -Sr));
                H[0] = new Matrix3x3(
                    dxx.m00, dxx.m01, dxr.x,
                    dxx.m10, dxx.m11, dxr.y,
                    dxr.x, dxr.y, drr
                );
            }
        }
    }
}