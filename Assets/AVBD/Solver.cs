// This file is the C# translation of avbd-demo2d-repo/source/solver.cpp
// It implements the main physics solver for the AVBD engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using System.Collections.Generic;
    using UnityEngine;

    /// <summary>
    /// Central physics integrator responsible for stepping the AVBD simulation.
    /// Performs broad-phase collision, constructs <see cref="Force"/> objects
    /// (e.g., <see cref="Manifold"/>, <see cref="Spring"/>) and iteratively solves
    /// the resulting constrained system every fixed-time-step.
    /// Attach one instance per scene.
    /// </summary>
    public class Solver : MonoBehaviour
    {
        public float dt = 1.0f / 60.0f;
        public float gravity = -10.0f;
        public int iterations = 10;

        public float alpha = 0.99f;
        public float beta = 100000.0f;
        public float gamma = 0.99f;

        public List<RigidBody> bodies = new List<RigidBody>();
        public List<Force> forces = new List<Force>();

        /// <summary>
        /// Validates body inertia values and, if safe, advances the simulation by
        /// calling <see cref="Step"/> once per fixed-time-step.
        /// </summary>
        void FixedUpdate()
        {
            // Guard: Only step if all bodies are initialized (moment > 0 for dynamic bodies)
            foreach (var body in bodies)
            {
                if (body.mass > 0 && body.moment <= 0)
                {
                    Debug.LogWarning($"[Solver] Skipping Step: Body '{body.name}' not fully initialized (mass={body.mass}, moment={body.moment})");
                    return;
                }
            }
            Step();
        }

        /// <summary>
        /// Main physics solver step. This is a direct port of Solver::step() from solver.cpp (C++).
        /// Performs collision detection, constraint setup, primal/dual updates, and velocity integration.
        /// </summary>
        public void Step()
        {
            // --- Broadphase collision detection (matches C++ O(n²) approach) ---
            for (int i = 0; i < bodies.Count; i++)
            {
                for (int j = i + 1; j < bodies.Count; j++)
                {
                    Vector2 dp = new Vector2(bodies[i].position.x - bodies[j].position.x, bodies[i].position.y - bodies[j].position.y);
                    float r = bodies[i].radius + bodies[j].radius;
                    if (Vector2.Dot(dp, dp) <= r * r && !bodies[i].IsConstrainedTo(bodies[j]))
                    {
                       forces.Add(new Manifold(this, bodies[i], bodies[j]));
                    }
                }
            }

            // Initialize and warmstart forces
            for (int i = forces.Count - 1; i >= 0; i--)
            {
                if (!forces[i].Initialize())
                {
                    // Disable and remove force
                    forces[i].Disable();
                    if (forces[i].bodyA != null) forces[i].bodyA.forces.Remove(forces[i]);
                    if (forces[i].bodyB != null) forces[i].bodyB.forces.Remove(forces[i]);
                    forces.RemoveAt(i);
                }
                else
                {
                    // Warmstart the force
                    for (int j = 0; j < forces[i].Rows(); j++)
                    {
                        // Warmstart the dual variables and penalty parameters (Eq. 19)
                        // Penalty is safely clamped to a minimum and maximum value
                        forces[i].lambda[j] = forces[i].lambda[j] * alpha * gamma;
                        forces[i].penalty[j] = Mathf.Clamp(forces[i].penalty[j] * gamma, 10000.0f, 1000000000.0f);

                        // If it's not a hard constraint, we don't let the penalty exceed the material stiffness
                        if (!float.IsInfinity(forces[i].stiffness[j]))
                            forces[i].penalty[j] = Mathf.Min(forces[i].penalty[j], forces[i].stiffness[j]);
                    }
                }
            }

            // Initialize and warmstart bodies
            for (int i = 0; i < bodies.Count; i++)
            {
                bodies[i].velocity.z = Mathf.Clamp(bodies[i].velocity.z, -10.0f, 10.0f);
                // Calculate inertial position without gravity
                // Compute inertial position (x~) matching C++: include gravity only for dynamic bodies
                bodies[i].inertial = bodies[i].position + bodies[i].velocity * dt;
                if (bodies[i].mass > 0)
                    bodies[i].inertial += new Vector3(0, gravity, 0) * (dt * dt);

                // Calculate vertical acceleration
                Vector3 accel = (bodies[i].velocity - bodies[i].prevVelocity) / dt;
                float accelExt = accel.y * Mathf.Sign(gravity);
                float accelWeight = Mathf.Clamp(accelExt / Mathf.Abs(gravity), 0.0f, 1.0f);
                if (float.IsNaN(accelWeight)) accelWeight = 0.0f;

                bodies[i].initial = bodies[i].position;
                if (bodies[i].mass > 0)
                {
                    // Warmstart position using adaptive gravity scaling (accelWeight) per original VBD paper
                    bodies[i].position = bodies[i].position + bodies[i].velocity * dt + new Vector3(0, gravity, 0) * (accelWeight * dt * dt);
                }
            }

            // Main solver loop
            for (int it = 0; it < iterations; it++)
            {
                // Primal update
                for (int i = 0; i < bodies.Count; i++)
                {
                    // Skip static bodies (mass == 0): they should not be included in the matrix solve step
                    if (bodies[i].mass <= 0)
                        continue;
                    // Debug.Log($"Solver Step - Processing Body: {body.name}, Mass: {body.mass}, Moment: {body.moment}");
                    // Debug.Log($"Solver Step - Processing Body: {body.name}, Mass: {body.mass}, Moment: {body.moment}");

                    Matrix3x3 M = new Matrix3x3(bodies[i].mass, 0, 0, 0, bodies[i].mass, 0, 0, 0, bodies[i].moment);
                    Matrix3x3 lhs = M * (1.0f / (dt * dt));
                    Vector3 rhs = lhs * (bodies[i].position - bodies[i].inertial);

                    // Iterate over all forces acting on the body
                    for ( int fit=0; fit< bodies[i].forces.Count; fit++)
                    {
                        // Compute constraint and its derivatives
                        bodies[i].forces[fit].ComputeConstraint(alpha);
                        bodies[i].forces[fit].ComputeDerivatives(bodies[i]);

                        for (int j = 0; j < bodies[i].forces[fit].Rows(); j++)
                        {
                            // Use lambda as 0 if it's not a hard constraint
                            float lambda = float.IsInfinity(bodies[i].forces[fit].stiffness[j]) ? bodies[i].forces[fit].lambda[j] : 0.0f;

                            // Compute the clamped force magnitude (Sec 3.2)
                            float f = Mathf.Clamp(bodies[i].forces[fit].penalty[j] * bodies[i].forces[fit].C[j] + lambda + bodies[i].forces[fit].motor[j], bodies[i].forces[fit].fmin[j], bodies[i].forces[fit].fmax[j]);

                            // Compute the diagonally lumped geometric stiffness term (Sec 3.5)
                            Matrix3x3 G = new Matrix3x3(bodies[i].forces[fit].H[j].GetColumn(0).magnitude, 0, 0, 0, bodies[i].forces[fit].H[j].GetColumn(1).magnitude, 0, 0, 0, bodies[i].forces[fit].H[j].GetColumn(2).magnitude) * Mathf.Abs(f);

                            // Accumulate force (Eq. 13) and hessian (Eq. 17)
                            rhs += bodies[i].forces[fit].J[j] * f; 
                            lhs += Matrix3x3.Outer(bodies[i].forces[fit].J[j], bodies[i].forces[fit].J[j] * bodies[i].forces[fit].penalty[j]) + G;
                        }
                    }


                    // Solve the SPD linear system using LDL and apply the update (Eq. 4)
                    bodies[i].position -= Maths.Solve(lhs, rhs);
                }

                // Dual update
                for (int i = 0; i < forces.Count; i++)
                {
                    // Compute constraint
                    forces[i].ComputeConstraint(alpha);

                    for (int j = 0; j < forces[i].Rows(); j++)
                    {
                        // Use lambda as 0 if it's not a hard constraint
                        float lambda = float.IsInfinity(forces[i].stiffness[j]) ? forces[i].lambda[j] : 0.0f;

                        // Update lambda (Eq 11)
                        // Note that we don't include non-conservative forces (ie motors) in the lambda update, as they are not part of the dual problem.
                        forces[i].lambda[j] = Mathf.Clamp(forces[i].penalty[j] * forces[i].C[j] + lambda, forces[i].fmin[j], forces[i].fmax[j]);

                        // Disable the force if it has exceeded its fracture threshold
                        if (Mathf.Abs(forces[i].lambda[j]) >= forces[i].fracture[j])
                        {
                            forces[i].Disable();
                        }

                        // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                        if (forces[i].lambda[j] > forces[i].fmin[j] && forces[i].lambda[j] < forces[i].fmax[j])
                        {
                            float maxPenalty = float.IsInfinity(forces[i].stiffness[j]) ? 1000000000.0f : forces[i].stiffness[j];
                            forces[i].penalty[j] = Mathf.Min(forces[i].penalty[j] + beta * Mathf.Abs(forces[i].C[j]), maxPenalty);
                        }
                    }
                }
            }

            // Compute velocities (BDF1)
            for (int i = 0; i < bodies.Count; i++)
            {
                bodies[i].prevVelocity = bodies[i].velocity;
                if (bodies[i].mass > 0)
                {
                    bodies[i].velocity = (bodies[i].position - bodies[i].initial) / dt;
                }
            }
        }
    public RigidBody Pick(Vector2 at, out Vector2 local)
        {
            local = Vector2.zero;
            foreach (var body in bodies)
            {
                // Convert world position to body's local space
                Matrix2x2 rotT = new Matrix2x2(-body.position.z);
                Vector2 localPos = rotT * (at - new Vector2(body.position.x, body.position.y));

                // Check if the local position is within the body's bounds
                if (localPos.x >= -body.size.x * 0.5f && localPos.x <= body.size.x * 0.5f &&
                    localPos.y >= -body.size.y * 0.5f && localPos.y <= body.size.y * 0.5f)
                {
                    local = localPos;
                    return body;
                }
            }
            return null;
        }
    }
}