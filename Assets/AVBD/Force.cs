// This file is the C# translation of avbd-demo2d-repo/source/force.cpp
// It implements the abstract force/constraint base for the AVBD engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using UnityEngine;

    /// <summary>
    /// Abstract base class for forces and constraints (see force.cpp in C++).
    /// </summary>
    public abstract class Force
    {
        public Solver solver;
        public RigidBody bodyA;
        public RigidBody bodyB;

        public Vector3[] J = new Vector3[4];
        public Matrix3x3[] H = new Matrix3x3[4];
        public float[] C = new float[4];
        public float[] fmin = new float[4];
        public float[] fmax = new float[4];
        public float[] stiffness = new float[4];
        public float[] motor = new float[4];
        public float[] fracture = new float[4];
        public float[] penalty = new float[4];
        public float[] lambda = new float[4];

        public Force(Solver solver, RigidBody bodyA, RigidBody bodyB)
        {
            this.solver = solver;
            this.bodyA = bodyA;
            this.bodyB = bodyB;

            // Register with solver
            solver.forces.Add(this);

            // Register with bodies
            if (bodyA != null) bodyA.forces.Add(this);
            if (bodyB != null) bodyB.forces.Add(this);

            // Initialize default values
            for (int i = 0; i < 4; i++)
            {
                J[i] = Vector3.zero;
                H[i] = Matrix3x3.zero;
                C[i] = 0.0f;
                motor[i] = 0.0f;
                stiffness[i] = float.PositiveInfinity;
                fmax[i] = float.PositiveInfinity;
                fmin[i] = float.NegativeInfinity;
                fracture[i] = float.PositiveInfinity;
                penalty[i] = 0.0f;
                lambda[i] = 0.0f;
            }
        }

        public abstract int Rows();
        public abstract bool Initialize();
        public abstract void ComputeConstraint(float alpha);
        public abstract void ComputeDerivatives(RigidBody body);

        /// <summary>
        /// Disables the force/constraint by zeroing all relevant arrays.
        /// This method should only be called internally by the Solver.
        /// </summary>
        public void Disable()
        {
            // Clear force data
            for (int i = 0; i < Rows(); i++)
            {
                stiffness[i] = 0;
                penalty[i] = 0;
                lambda[i] = 0;
            }
        }
    }
}