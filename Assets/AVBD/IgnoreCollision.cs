// This file is a C# translation of the IgnoreCollision force from the C++ demo.
// It's a placeholder force used to prevent collision checks between specific bodies.

namespace avbd
{
    /// <summary>
    /// A dummy force used to signal that collisions between two bodies should be ignored.
    /// </summary>
    public class IgnoreCollision : Force
    {
        public IgnoreCollision(Solver solver, RigidBody bodyA, RigidBody bodyB)
            : base(solver, bodyA, bodyB) { }

        public override int Rows() { return 0; }
        public override bool Initialize() { return true; }
        public override void ComputeConstraint(float alpha) { }
        public override void ComputeDerivatives(RigidBody body) { }
    }
}
