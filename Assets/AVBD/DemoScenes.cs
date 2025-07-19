using UnityEngine;
using System.Collections.Generic;

namespace avbd
{
    /// <summary>
    /// Re-creates the demo scenes defined in the original C++ <c>scenes.h</c> file.
    /// Call <see cref="Run"/> with an index from 0-17.
    /// </summary>
    public static class DemoScenes
    {
        private static readonly System.Action<Solver>[] scenes =
        {
            SceneEmpty,
            SceneGround,
            SceneDynamicFriction,
            SceneStaticFriction,
            ScenePyramid,
            SceneRope,
            SceneHeavyRope,
            SceneHangingRope,
            SceneSpring,
            SceneSpringRatio,
            SceneStack,
            SceneStackRatio,
            SceneRod,
            SceneSoftBody,
            SceneJointGrid,
            SceneNet,
            SceneMotor,
            SceneFracture
        };

        public static void Run(int index)
        {
            if (index < 0 || index >= scenes.Length)
            {
                Debug.LogError($"[DemoScenes] Invalid scene index {index}");
                return;
            }
            var solvers = Object.FindObjectsByType<Solver>(FindObjectsSortMode.None);
            Solver solver = null;
            if (solvers.Length > 0)
            {
                solver = solvers[0];
                // Destroy any additional Solver duplicates that may have persisted
                for (int i = 1; i < solvers.Length; i++)
                    Object.Destroy(solvers[i].gameObject);
            }
            if (solver == null)
            {
                solver = new GameObject("Solver").AddComponent<Solver>();
            }
            // Clear previous scene objects (destroy all rigid bodies + forces)
            foreach (var body in solver.bodies.ToArray())
            {
                if (body != null)
                    Object.Destroy(body.gameObject);
            }
            solver.bodies.Clear();
            solver.forces.Clear();

            scenes[index](solver);
            Debug.Log($"[DemoScenes] Loaded demo #{index}");
        }

        #region Scene Implementations (subset)
        private static void SceneEmpty(Solver solver) { /* nothing */ }

        private static void SceneGround(Solver solver)
        {
            CreateBox(solver, new Vector2(100f, 1f), 0f, 0.5f, new Vector3(0, 0, 0), color: Color.green);
        }

        private static void SceneDynamicFriction(Solver solver)
        {
            SceneGround(solver);
            for (int x = 0; x <= 10; x++)
            {
                float mu = 5.0f - (x / 10f * 5f);
                CreateBox(solver, new Vector2(1f, 0.5f), 1f, mu, new Vector3(-30f + x * 2f, 0.75f, 0), new Vector3(10, 0, 0));
            }
        }

        private static void SceneStaticFriction(Solver solver)
        {
            CreateBox(solver, new Vector2(100f, 1f), 0f, 1f, new Vector3(0, 0, Mathf.PI / 6f), color: Color.green);
            for (int y = 0; y <= 10; y++)
            {
                CreateBox(solver, new Vector2(5f, 0.5f), 1f, 1f, new Vector3(0, y * 1f + 1f, Mathf.PI / 6f));
            }
        }

        private static void ScenePyramid(Solver solver)
        {
            int SIZE = 20;
            CreateBox(solver, new Vector2(100, 0.5f), 0f, 0.5f, new Vector3(0, -2.5f, 0), color: Color.green);
            for (int y = 0; y < SIZE; y++)
                for (int x = 0; x < SIZE - y; x++)
                    CreateBox(solver, new Vector2(1f, 0.5f), 1f, 0.5f, new Vector3(x * 1.1f + y * 0.5f - SIZE / 2f, y * 0.85f, 0));
        }

        private static void SceneRope(Solver solver)
        {
            RigidBody prev = null;
            for (int i = 0; i < 20; i++)
            {
                RigidBody curr = CreateBox(solver, new Vector2(1f, 0.5f), i == 0 ? 0f : 1f, 0.5f, new Vector3(i, 10f, 0));
                if (prev) new Joint(solver, prev, curr, new Vector2(0.5f, 0), new Vector2(-0.5f, 0), new Vector3(float.PositiveInfinity, float.PositiveInfinity, 0));
                prev = curr;
            }
        }

        private static void SceneHeavyRope(Solver solver)
        {
            const int N = 20;
            const float SIZE = 30f;
            RigidBody prev = null;
            for (int i = 0; i < N; i++)
            {
                Vector2 dims = (i == N - 1) ? new Vector2(SIZE, SIZE) : new Vector2(1f, 0.5f);
                float mass = i == 0 ? 0f : 1f;
                RigidBody curr = CreateBox(solver, dims, mass, 0.5f, new Vector3(i + (i == N - 1 ? SIZE / 2f : 0), 10f, 0));
                if (prev)
                    new Joint(solver, prev, curr, new Vector2(0.5f, 0), (i == N - 1) ? new Vector2(-SIZE / 2f, 0) : new Vector2(-0.5f, 0), new Vector3(float.PositiveInfinity, float.PositiveInfinity, 0));
                prev = curr;
            }
        }

        private static void SceneHangingRope(Solver solver)
        {
            const int N = 50;
            const float SIZE = 10f;
            RigidBody prev = null;
            for (int i = 0; i < N; i++)
            {
                Vector2 dims = (i == N - 1) ? new Vector2(SIZE, SIZE) : new Vector2(0.5f, 1f);
                float mass = i == 0 ? 0f : 1f;
                RigidBody curr = CreateBox(solver, dims, mass, 0.5f, new Vector3(0, 10f - (i + (i == N - 1 ? SIZE / 2f : 0)), 0));
                if (prev)
                    new Joint(solver, prev, curr, new Vector2(0, -0.5f), (i == N - 1) ? new Vector2(0, SIZE / 2f) : new Vector2(0, 0.5f), new Vector3(float.PositiveInfinity, float.PositiveInfinity, 0));
                prev = curr;
            }
        }

        private static void SceneSpring(Solver solver)
        {
            RigidBody anchor = CreateBox(solver, new Vector2(1, 1), 0f, 0.5f, new Vector3(0, 0, 0));
            RigidBody block = CreateBox(solver, new Vector2(4, 4), 1f, 0.5f, new Vector3(0, -8, 0));
            new Spring(solver, anchor, block, Vector2.zero, Vector2.zero, 100.0f, 4.0f);
        }

        private static void SceneSpringRatio(Solver solver)
        {
            const int N = 8;
            RigidBody prev = null;
            for (int i = 0; i < N; i++)
            {
                float mass = (i == 0 || i == N - 1) ? 0f : 1f;
                RigidBody curr = CreateBox(solver, new Vector2(1f, 0.5f), mass, 0.5f, new Vector3(i * 4f, 10f, 0));
                if (prev)
                {
                    float rest = 0.1f;
                    new Spring(solver, prev, curr, new Vector2(0.5f, 0), new Vector2(-0.5f, 0), i % 2 == 0 ? 1000.0f : 1000000.0f, rest);
                }
                prev = curr;
            }
        }

        private static void SceneStack(Solver solver)
        {
            CreateBox(solver, new Vector2(100, 1), 0f, 0.5f, new Vector3(0, -1f, 0), color: Color.green);
            for (int i = 0; i < 20; i++)
                CreateBox(solver, new Vector2(1, 1), 1f, 0.5f, new Vector3(0, i * 2f + 1f, 0));
        }

        private static void SceneStackRatio(Solver solver)
        {
            CreateBox(solver, new Vector2(100, 1), 0f, 0.5f, Vector3.zero, color: Color.green);
            int y = 1;
            int s = 1;
            for (int i = 0; i < 6; i++)
            {
                CreateBox(solver, new Vector2(s, s), 1f, 0.5f, new Vector3(0, y, 0));
                y += s * 3 / 2;
                s *= 2;
            }
        }

        private static void SceneRod(Solver solver)
        {
            RigidBody prev = null;
            for (int i = 0; i < 20; i++)
            {
                float mass = i == 0 ? 0f : 1f;
                RigidBody curr = CreateBox(solver, new Vector2(1f, 0.5f), mass, 0.5f, new Vector3(i, 10, 0));
                if (prev)
                    new Joint(solver, prev, curr, new Vector2(0.5f, 0), new Vector2(-0.5f, 0), new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity));
                prev = curr;
            }
        }

        private static void SceneSoftBody(Solver solver)
        {
            CreateBox(solver, new Vector2(100, 0.5f), 0f, 0.5f, new Vector3(0, 0, 0), color: Color.green);

            const float Klin = 1000.0f;
            const float Kang = 100.0f;
            const int W = 15, H = 5;
            const int N = 2;
            for (int i = 0; i < N; i++)
            {
                RigidBody[,] grid = new RigidBody[W, H];
                for (int x = 0; x < W; x++)
                    for (int y = 0; y < H; y++)
                        grid[x, y] = CreateBox(solver, new Vector2(1, 1), 1f, 0.5f, new Vector3(x, y + H * i * 2.0f + 5.0f, 0));

                for (int x = 1; x < W; x++)
                    for (int y = 0; y < H; y++)
                        new Joint(solver, grid[x - 1, y], grid[x, y], new Vector2(0.5f, 0), new Vector2(-0.5f, 0), new Vector3(Klin, Klin, Kang));

                for (int x = 0; x < W; x++)
                    for (int y = 1; y < H; y++)
                        new Joint(solver, grid[x, y - 1], grid[x, y], new Vector2(0, 0.5f), new Vector2(0, -0.5f), new Vector3(Klin, Klin, Kang));

                for (int x = 1; x < W; x++)
                {
                    for (int y = 1; y < H; y++)
                    {
                        new IgnoreCollision(solver, grid[x - 1, y - 1], grid[x, y]);
                        new IgnoreCollision(solver, grid[x, y - 1], grid[x - 1, y]);
                    }
                }
            }
        }

        private static void SceneJointGrid(Solver solver)
        {
            const int W = 30, H = 30;
            RigidBody[,] grid = new RigidBody[W, H];
            for (int x = 0; x < W; x++)
                for (int y = 0; y < H; y++)
                    grid[x, y] = CreateBox(solver, new Vector2(1, 1), (y == H - 1 && (x == 0 || x == W - 1)) ? 0f : 1f, 0.5f, new Vector3(x, y, 0));

            for (int x = 1; x < W; x++)
                for (int y = 0; y < H; y++)
                    new Joint(solver, grid[x - 1, y], grid[x, y], new Vector2(0.5f, 0), new Vector2(-0.5f, 0));
            for (int x = 0; x < W; x++)
                for (int y = 1; y < H; y++)
                    new Joint(solver, grid[x, y - 1], grid[x, y], new Vector2(0, 0.5f), new Vector2(0, -0.5f));
        }

        private static void SceneNet(Solver solver)
        {
            const int N = 40;
            CreateBox(solver, new Vector2(100, 0.5f), 0f, 0.5f, Vector3.zero, color: Color.green);
            RigidBody prev = null;
            for (int i = 0; i < N; i++)
            {
                float mass = (i == 0 || i == N - 1) ? 0f : 1f;
                RigidBody curr = CreateBox(solver, new Vector2(1, 0.5f), mass, 0.5f, new Vector3(i - N / 2f, 10, 0));
                if (prev)
                    new Joint(solver, prev, curr, new Vector2(0.5f, 0), new Vector2(-0.5f, 0), new Vector3(float.PositiveInfinity, float.PositiveInfinity, 0));
                prev = curr;
            }
            for (int x = 0; x < N / 4; x++)
                for (int y = 0; y < N / 8; y++)
                    CreateBox(solver, new Vector2(1, 1), 1f, 0.5f, new Vector3(x - N / 8f, y + 15f, 0));
        }

        private static void SceneMotor(Solver solver)
        {
            CreateBox(solver, new Vector2(100, 0.5f), 0f, 0.5f, new Vector3(0, -10, 0), color: Color.green);
            RigidBody a0 = CreateBox(solver, new Vector2(5, 0.5f), 1f, 0.5f, Vector3.zero);
            RigidBody a1 = CreateBox(solver, new Vector2(5, 0.5f), 1f, 0.5f, new Vector3(0, 0, Mathf.PI / 2f));
            new Joint(solver, a0, a1, Vector2.zero, Vector2.zero, new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity));
            new Joint(solver, null, a0, Vector2.zero, Vector2.zero, new Vector3(float.PositiveInfinity, float.PositiveInfinity, 0f), -250f);
        }

        private static void SceneFracture(Solver solver)
        {
            const int N = 10;
            const int M = 10;
            CreateBox(solver, new Vector2(100, 0.5f), 0f, 0.5f, Vector3.zero, color: Color.green);
            RigidBody prev = null;
            for (int i = 0; i <= N; i++)
            {
                RigidBody curr = CreateBox(solver, new Vector2(1, 0.5f), 0.5f, 0.5f, new Vector3(i - N / 2f, 6, 0));
                if (prev)
                    new Joint(solver, prev, curr, new Vector2(0.5f, 0), new Vector2(-0.5f, 0), new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity), 0f, 500f);
                prev = curr;
            }
            CreateBox(solver, new Vector2(1, 5), 5f, 0.5f, new Vector3(-N / 2f, 2.5f, 0));
            CreateBox(solver, new Vector2(1, 5), 5f, 0.5f, new Vector3(N / 2f, 2.5f, 0));
            for (int i = 0; i < M; i++)
                CreateBox(solver, new Vector2(2, 1), 2f, 0.5f, new Vector3(0, i * 2f + 8f, 0));
        }
        #endregion

        public static RigidBody CreateBox(Solver solver, Vector2 size, float mass, float friction, Vector3 position, Vector3 initialVelocity = default, Color? color = null)
        {
            GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.transform.localScale = new Vector3(size.x, size.y, 1);
            box.transform.position = new Vector3(position.x, position.y, 0);
            box.transform.rotation = Quaternion.Euler(0, 0, position.z * Mathf.Rad2Deg);
            box.name = $"Box_{solver.bodies.Count}";
            if (color.HasValue)
            {
                var renderer = box.GetComponent<Renderer>();
                if (renderer != null)
                {
                    Debug.Log("here");
                    renderer.material.color = color.Value;
                }
            }
            var rb = box.AddComponent<RigidBody>();
            rb.solver = solver;
            rb.mass = mass;
            rb.friction = friction;
            rb.InitializeBody(initialVelocity); // Initialize body state immediately
            solver.bodies.Add(rb);
            return rb;
        }
    }
}
