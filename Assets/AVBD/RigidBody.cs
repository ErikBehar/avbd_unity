// This file is the C# translation of avbd-demo2d-repo/source/rigid.cpp
// It implements the rigid body logic for the AVBD physics engine.
// Keep in sync with the C++ original for correctness!

namespace avbd
{
    using UnityEngine;
    using System.Collections.Generic;
    using Unity.VisualScripting;

    /// <summary>
    /// Represents a 2-D rigid body that is simulated by the AVBD solver.
    /// Stores kinematic state (position, velocity), mass/inertia, and a list of
    /// active <see cref="Force"/> constraints. At runtime the <see cref="Solver"/>
    /// updates these fields every physics step while <see cref="FixedUpdate"/>
    /// commits the simulated pose back to the Unity <see cref="Transform"/>.
    /// </summary>
    public class RigidBody : MonoBehaviour
    {
        public Solver solver;
        public List<Force> forces = new List<Force>();
        public Vector3 position;
        public Vector3 initial;
        public Vector3 inertial;
        public Vector3 velocity;
        public Vector3 prevVelocity;
        public Vector2 size;
        public float mass;
        public float invMass;
        public float moment;
        public float invInertia;
        [SerializeField]
        [Tooltip("Coefficient of friction (0 = no friction, 1 = high).")]
        public float friction = 0.3f;
        [Tooltip("Density used to compute mass (mass = area * density).")]
        public float density = 0.0f; // default 0 keeps existing mass for static/inspector bodies // New: matches C++ density param
        public float radius;

        /// <summary>
        /// Initializes the rigid body state from the Unity transform.
        /// Mirrors logic in rigid.cpp (C++).
        /// </summary>
        public void InitializeBody(Vector3 initialVelocity = default)
        {
            //solver.bodies.Add(this);
            position = new Vector3(transform.position.x, transform.position.y, transform.rotation.eulerAngles.z * Mathf.Deg2Rad);
            initial = position;
            velocity = initialVelocity;
            prevVelocity = initialVelocity;
            size = new Vector2(transform.localScale.x, transform.localScale.y);
            
            // Compute mass from density only when density > 0, otherwise keep inspector value (allows static ground bodies).
            if (density > 0.0f)
                mass = size.x * size.y * density;

            // Moment of inertia for rectangle around centre
            moment = mass * (size.x * size.x + size.y * size.y) / 12.0f;
            radius = Mathf.Sqrt(size.x * size.x + size.y * size.y) * 0.5f;
            Debug.Log($"[RIGIDBODY INIT] Name: {gameObject.name}, localScale: {transform.localScale}, Mass: {mass}, Size: {size}, Moment: {moment}");

            if (mass > 0.0f && moment <= 0.0f)
            {
                Debug.LogError($"RigidBody Start - WARNING: Dynamic body '{gameObject.name}' has zero or negative moment of inertia! Mass: {mass}, Size: {size}, Moment: {moment}");
            }

            if (mass == 0.0f)
            {
                invMass = 0.0f;
                invInertia = 0.0f;
            }
            else
            {
                invMass = 1.0f / mass;
                invInertia = 1.0f / moment;
            }
        }

        /// <summary>
        /// Returns <c>true</c> if this body is already constrained to <paramref name="other"/>
        /// by an existing <see cref="Manifold"/> contact. This prevents duplicate
        /// constraints from being generated during broad-phase collision checks.
        /// </summary>
        public bool IsConstrainedTo(RigidBody other)
        {
            foreach (Force f in forces)
            {
                if ((f is Manifold || f is IgnoreCollision) && ((f.bodyA == this && f.bodyB == other) || (f.bodyA == other && f.bodyB == this)))
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Converts a vector from this body's local space to world space using the
        /// simulated pose stored in <see cref="position"/> (rather than the current
        /// Unity transform, which may be a frame behind).
        /// </summary>
        public Vector2 Transform(Vector2 v)
        {
            float c = Mathf.Cos(position.z);
            float s = Mathf.Sin(position.z);
            return new Vector2(position.x + v.x * c - v.y * s, position.y + v.x * s + v.y * c);
        }

        /// <summary>
        /// Pushes the simulated position/rotation to the Unity transform.
        /// Defensive checks prevent propagation of NaN values.
        /// Mirrors logic in rigid.cpp (C++).
        /// </summary>
        void FixedUpdate()
        {
            if (float.IsNaN(position.x) || float.IsNaN(position.y) || float.IsNaN(position.z))
            {
                Debug.LogError($"{name}: Position is NaN! {position}");
                return; // Prevents the transform from being set to NaN
            }
            transform.position = new Vector3(position.x, position.y, 0);
            if (float.IsNaN(position.z))
            {
                Debug.LogError($"{name}: Rotation is NaN! {position.z}");
                return;
            }
            transform.rotation = Quaternion.Euler(0, 0, position.z * Mathf.Rad2Deg);
        }
    }
}