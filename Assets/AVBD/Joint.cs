// This file is the C# translation of avbd-demo2d-repo/source/joint.cpp
// It implements joint constraints for the AVBD engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using UnityEngine;

    /// <summary>
    /// Implements joint constraints (see joint.cpp in C++).
    /// </summary>
    public class Joint : Force
    {
        public Vector2 rA, rB;
        public Vector3 C0; // Cached C(x-) at start of step for Baumgarte stabilisation
        public float torqueArm; // Squared length of effective arm used to scale torque row
        public float restAngle; // Relative rest orientation between bodies

        /// <summary>
        /// Constructs a joint between two bodies. Mirrors C++ constructor in joint.cpp.
        /// </summary>
        /// <summary>
        /// Constructs a joint between two bodies. Mirrors Joint constructor in C++.
        /// If <paramref name="bodyA"/> is <c>null</c> the joint is treated as being
        /// anchored in world space at <paramref name="rA"/>.
        /// </summary>
        /// <param name="stiffness">Per–row stiffness (N/m) where <see cref="float.PositiveInfinity"/> encodes a hard constraint.</param>
        /// <param name="motor">Motor target torque applied to the angular row.</param>
        /// <param name="fracture">Torque limit that disables the joint if surpassed.</param>
        // Convenience overload replicating default C++ parameters (linear hard constraint, free angle)
        public Joint(Solver solver, RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB)
            : this(solver, bodyA, bodyB, rA, rB, new Vector3(float.PositiveInfinity, float.PositiveInfinity, 0.0f)) { }

        public Joint(Solver solver, RigidBody bodyA, RigidBody bodyB, Vector2 rA, Vector2 rB, Vector3 stiffness, float motor = 0.0f, float fracture = float.PositiveInfinity)
            : base(solver, bodyA, bodyB)
        {
            this.rA = rA;
            this.rB = rB;
            // Copy stiffness/motor/fracture into base arrays (match C++)
            this.stiffness[0] = stiffness.x;
            this.stiffness[1] = stiffness.y;
            this.stiffness[2] = stiffness.z;
            this.motor[2] = motor;
            this.fmax[2] = fracture;
            this.fmin[2] = -fracture;
            this.fracture[2] = fracture;
            this.restAngle = (bodyA != null ? bodyA.position.z : 0.0f) - bodyB.position.z;
            float ta = (((bodyA != null) ? bodyA.size : Vector2.zero) + bodyB.size).sqrMagnitude;
            // Clamp to keep motor row well-conditioned (matches guidance from C++ sample)
            this.torqueArm = Mathf.Clamp(ta, 0.01f, 10.0f);
        }

        /// <summary>
        /// Number of constraint rows (matches C++ logic).
        /// </summary>
        public override int Rows() { return 3; }

        /// <summary>
        /// Initializes the joint constraint. Mirrors C++ logic.
        /// </summary>
        public override bool Initialize()
        {
            // Store C(x-) at start of step for Baumgarte stabilisation
            Vector2 worldA = (bodyA != null) ? bodyA.Transform(rA) : rA;
            Vector2 worldB = bodyB.Transform(rB);
            C0 = new Vector3(worldA.x - worldB.x,
                             worldA.y - worldB.y,
                             ((bodyA != null ? bodyA.position.z : 0) - bodyB.position.z - restAngle) * torqueArm);
            return stiffness[0] != 0 || stiffness[1] != 0 || stiffness[2] != 0 || motor[0] != 0 || motor[1] != 0 || motor[2] != 0;
        }

        /// <summary>
        /// Computes the constraint value for the joint. Mirrors C++ logic.
        /// </summary>
        public override void ComputeConstraint(float alpha)
        {
            // Current constraint value C(x)
            Vector2 worldA = (bodyA != null) ? bodyA.Transform(rA) : rA;
            Vector2 worldB = bodyB.Transform(rB);
            Vector3 Cn = new Vector3(worldA.x - worldB.x,
                                     worldA.y - worldB.y,
                                     ((bodyA != null ? bodyA.position.z : 0) - bodyB.position.z - restAngle) * torqueArm);

            for (int i = 0; i < Rows(); i++)
            {
                // For hard rows (infinite stiffness) apply Baumgarte stabilisation (Eq. 18 in paper)
                if (float.IsInfinity(stiffness[i]))
                    C[i] = Cn[i] - C0[i] * alpha;
                else
                    C[i] = Cn[i];
            }
        }

        /// <summary>
        /// Computes the derivatives for the joint constraint. Mirrors C++ logic.
        /// </summary>
        public override void ComputeDerivatives(RigidBody body)
        {
            if (body == bodyA)
            {
                Vector2 r = Maths.Rotate(rA, bodyA.position.z);
                J[0] = new Vector3(1.0f, 0.0f, -r.y);
                J[1] = new Vector3(0.0f, 1.0f, r.x);
                J[2] = new Vector3(0.0f, 0.0f, torqueArm);
                H[0] = new Matrix3x3(0,0,0, 0,0,0, 0,0,-r.x);
                H[1] = new Matrix3x3(0,0,0, 0,0,0, 0,0,-r.y);
                H[2] = new Matrix3x3(); // zero
            }
            else
            {
                Vector2 r = Maths.Rotate(rB, bodyB.position.z);
                J[0] = new Vector3(-1.0f, 0.0f, r.y);
                J[1] = new Vector3(0.0f, -1.0f, -r.x);
                J[2] = new Vector3(0.0f, 0.0f, -torqueArm);
                H[0] = new Matrix3x3(0,0,0, 0,0,0, 0,0,r.x);
                H[1] = new Matrix3x3(0,0,0, 0,0,0, 0,0,r.y);
                H[2] = new Matrix3x3();
            }
        }
    }
}