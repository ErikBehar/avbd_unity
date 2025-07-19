using UnityEngine;
using UnityEngine.SceneManagement;

namespace avbd
{
    /// <summary>
    /// Listens for number-key input and recreates one of the predefined demo scenes
    /// from the original C++ sample set.  The mapping is logged at start-up.
    /// Attach this once in an empty bootstrap scene.
    /// </summary>
    public class DemoManager : MonoBehaviour
    {
        private static DemoManager instance;

        private Solver solver;
        private Joint dragJoint;
        private Vector2 boxSize = new Vector2(1, 1);
        private float boxFriction = 0.5f;
        private float boxDensity = 1.0f;

        private Camera cam;

        private void Awake()
        {
            if (instance != null && instance != this)
            {
                Destroy(gameObject);
                return; // another instance already exists
            }
            instance = this;
            DontDestroyOnLoad(gameObject);
            Debug.Log("[DemoManager] Press keys 0-9 / F1-F8 to load AVBD demo scenes\n" +
                      " 0: Empty, 1: Ground, 2: DynFriction, 3: StatFriction, 4: Pyramid, 5: Rope, 6: HeavyRope, 7: HangRope, 8: Spring, 9: SpringRatio, F1: Stack, F2: StackRatio, F3: Rod, F4: SoftBody, F5: JointGrid, F6: Net, F7: Motor, F8: Fracture");
        }

        private void Update()
        {
            process_scene_loads();

            // Allow interaction if we have the solver instance in the scene
            if (solver == null)
            {
                Solver[] solvers = FindObjectsByType<Solver>(FindObjectsSortMode.None);

                cam = FindObjectsByType<Camera>(FindObjectsSortMode.None)[0];

                if (solvers.Length == 0) return; // No solver yet, wait for it to be created

                solver = solvers[0];
            }
            else
            {
                process_interaction();
            }
        }

        private void process_scene_loads()
        {
            // Scene loading logic (existing)
            int? index = null;
            if (Input.GetKeyDown(KeyCode.Alpha0)) index = 0;
            else if (Input.GetKeyDown(KeyCode.Alpha1)) index = 1;
            else if (Input.GetKeyDown(KeyCode.Alpha2)) index = 2;
            else if (Input.GetKeyDown(KeyCode.Alpha3)) index = 3;
            else if (Input.GetKeyDown(KeyCode.Alpha4)) index = 4;
            else if (Input.GetKeyDown(KeyCode.Alpha5)) index = 5;
            else if (Input.GetKeyDown(KeyCode.Alpha6)) index = 6;
            else if (Input.GetKeyDown(KeyCode.Alpha7)) index = 7;
            else if (Input.GetKeyDown(KeyCode.Alpha8)) index = 8;
            else if (Input.GetKeyDown(KeyCode.Alpha9)) index = 9;
            else if (Input.GetKeyDown(KeyCode.F1)) index = 10;
            else if (Input.GetKeyDown(KeyCode.F2)) index = 11;
            else if (Input.GetKeyDown(KeyCode.F3)) index = 12;
            else if (Input.GetKeyDown(KeyCode.F4)) index = 13;
            else if (Input.GetKeyDown(KeyCode.F5)) index = 14;
            else if (Input.GetKeyDown(KeyCode.F6)) index = 15;
            else if (Input.GetKeyDown(KeyCode.F7)) index = 16;
            else if (Input.GetKeyDown(KeyCode.F8)) index = 17;

            if (index.HasValue)
            {
                Debug.Log("Picked: " + index.Value.ToString());
                StartCoroutine(ReloadAndRun(index.Value));
            }
        }

        private void process_interaction()
        {
            // Convert mouse position to world coordinates
            Vector3 mouseScreenPos = Input.mousePosition;
            mouseScreenPos.z = Mathf.Abs(cam.transform.position.z);
            Vector3 mouseWorldPos = cam.ScreenToWorldPoint(mouseScreenPos);
            Vector2 mouseWorldPos2D = new Vector2(mouseWorldPos.x, mouseWorldPos.y);

            // Left mouse button: Drag body
            if (Input.GetMouseButtonDown(0))
            {
                Vector2 localOffset;
                RigidBody pickedBody = solver.Pick(mouseWorldPos2D, out localOffset);
                if (pickedBody != null)
                {
                    dragJoint = new Joint(solver, null, pickedBody, mouseWorldPos2D, localOffset, new Vector3(1000.0f, 1000.0f, 0.0f));
                }
            }
            else if (Input.GetMouseButton(0))
            {
                if (dragJoint != null)
                {
                    dragJoint.rA = mouseWorldPos2D;
                }
            }
            else if (Input.GetMouseButtonUp(0))
            {
                if (dragJoint != null)
                {
                    // Remove the drag joint from the solver's forces list
                    solver.forces.Remove(dragJoint);
                    // Also remove it from the body's forces list if it was added
                    if (dragJoint.bodyA != null) dragJoint.bodyA.forces.Remove(dragJoint);
                    if (dragJoint.bodyB != null) dragJoint.bodyB.forces.Remove(dragJoint);
                    dragJoint = null;
                }
            }

            // Right mouse button: Spawn box
            if (Input.GetMouseButtonDown(1))
            {
                Debug.Log( "mouse pos:" + Input.mousePosition + " world pos: " + mouseWorldPos);
                //zero z since its rotation 
                mouseWorldPos.z = 0; 

                // Create a new box at the mouse position
                DemoScenes.CreateBox(solver, boxSize, boxDensity, boxFriction, mouseWorldPos);
            }
        }

        private System.Collections.IEnumerator ReloadAndRun(int idx)
        {
            // Simple scene reload to clear objects – this guarantees a clean slate.
            var scene = SceneManager.GetActiveScene();
            yield return SceneManager.LoadSceneAsync(scene.name, LoadSceneMode.Single);
            DemoScenes.Run(idx);
            // Wait one frame for objects to initialise.
            yield return null;
        }
    }
}