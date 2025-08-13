using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RobotAgent : Agent
{
    // ----------------- Tunables -----------------
    [Header("Movement")]
    [Tooltip("Cruise speed on the ground (m/s).")]
    public float moveSpeed = 2.0f;
    [Tooltip("Acceleration toward target speed (m/s^2).")]
    public float accel = 8f;
    [Tooltip("Hard cap on horizontal speed (m/s).")]
    public float maxSpeed = 2.4f;
    [Tooltip("Yaw degrees/second when turning.")]
    public float turnSpeed = 100f;

    [Header("Animation")]
    [Tooltip("Animator that controls the robot's animations.")]
    public Animator animator;
    [Tooltip("Bool parameter that is true while the agent is walking.")]
    public string walkParam = "isWalking";

    [Header("World")]
    [Tooltip("BoxCollider (NOT trigger) that encloses the allowed play area.")]
    public Collider gardenBounds;
    [Tooltip("Layers considered ground (TerrainCollider / MeshCollider / BoxCollider).")]
    public LayerMask groundMask = ~0;
    [Tooltip("Sphere probe radius for ground snap (m).")]
    public float groundProbeRadius = 0.25f;
    [Tooltip("How far to search downward for ground each step (m).")]
    public float groundProbeDistance = 3.0f;
    [Tooltip("Keep feet slightly above ground to avoid jitter (m).")]
    public float hoverOffset = 0.02f;

    [Header("Sensing (Observations & Interactions)")]
    public float senseRadius = 8f;
    public int maxObjectsObserved = 6;
    public LayerMask interactableMask = ~0;

    [Header("Debug / Logging")]
    [Tooltip("Print a compact step log every N decisions (0 = off).")]
    public int printEverySteps = 25;
    [Tooltip("Print Collision/Trigger logs to the Console.")]
    public bool logCollisions = true;

    // ----------------- Runtime -----------------
    Rigidbody rb;
    readonly Collider[] hits = new Collider[32];
    readonly HashSet<int> seenThisEpisode = new();

    Vector3 desiredXZ = Vector3.zero; // target planar velocity
    int stepCount = 0;

    // Optional state flags some interactables might toggle
    public bool carryingApple { get; private set; }
    public bool isSeated { get; private set; }
    public bool isPlayingWater { get; private set; }

    // ----------------- Init -----------------
    void Awake()
    {
        // Disable CharacterController if present (fights Rigidbody).
        if (TryGetComponent<CharacterController>(out var cc))
        {
            cc.enabled = false;
            Debug.Log("[RobotAgent] Disabled CharacterController for training (Rigidbody controls motion).");
        }

        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        if (!TryGetComponent<Collider>(out _))
            Debug.LogWarning("[RobotAgent] No collider on the Agent. Add a BoxCollider or a Convex MeshCollider.");

        // If there’s no DecisionRequester, request actions on a timer.
        if (!TryGetComponent<DecisionRequester>(out _))
            InvokeRepeating(nameof(RequestDecision), 0.1f, 0.1f);
    }

    void Start()
    {
        SnapToGroundImmediate(); // place feet on the ground at startup
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        desiredXZ = Vector3.zero;
        stepCount = 0;

        carryingApple = false;
        isSeated = false;
        isPlayingWater = false;
        seenThisEpisode.Clear();

        SnapToGroundImmediate();

        Debug.Log($"[EP START] {name} pos={transform.position} groundMask={(int)groundMask}");
    }

    // ----------------- Grounding -----------------
    void SnapToGroundImmediate()
    {
        // Cast down from a little above the robot
        Vector3 start = transform.position + Vector3.up * 0.2f;

        if (Physics.SphereCast(start, groundProbeRadius, Vector3.down,
                               out var hit, groundProbeDistance + 2f,
                               groundMask, QueryTriggerInteraction.Ignore))
        {
            float y = hit.point.y + hoverOffset;
            var p = transform.position; p.y = y; transform.position = p;

            // Stop downward velocity so we don't re-penetrate
            var v = rb.linearVelocity;
            if (v.y < 0f) rb.linearVelocity = new Vector3(v.x, 0f, v.z);
        }
        else
        {
            Debug.LogWarning($"[Snap] No ground under {name}. Check groundMask & ground colliders.");
        }
    }

    void StickToGroundStep()
    {
        Vector3 origin = transform.position + Vector3.up * 0.2f;

        if (Physics.SphereCast(origin, groundProbeRadius, Vector3.down,
                               out var hit, groundProbeDistance,
                               groundMask, QueryTriggerInteraction.Ignore))
        {
            float targetY = hit.point.y + hoverOffset;
            float deltaY = targetY - transform.position.y;

            if (Mathf.Abs(deltaY) > 0.002f)
            {
                var p = transform.position;
                p.y += Mathf.Clamp(deltaY, -0.05f, 0.05f);   // smooth adjust
                transform.position = p;

                if (deltaY > 0f && rb.linearVelocity.y < 0f)
                {
                    var v = rb.linearVelocity; v.y = 0f; rb.linearVelocity = v;
                }
            }
        }
    }

    // ----------------- Observations -----------------
    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 vLocal = transform.InverseTransformDirection(rb.linearVelocity);
        sensor.AddObservation(vLocal.x);
        sensor.AddObservation(vLocal.z);

        sensor.AddObservation(carryingApple ? 1f : 0f);
        sensor.AddObservation(isSeated ? 1f : 0f);
        sensor.AddObservation(isPlayingWater ? 1f : 0f);

        // Nearby interactables (angle + normalized distance + one-hot type)
        int count = Physics.OverlapSphereNonAlloc(transform.position, senseRadius, hits, interactableMask);
        var list = new List<(float dist, Interactable it, Vector3 dir)>(count);
        for (int i = 0; i < count; i++)
            if (hits[i] && hits[i].TryGetComponent(out Interactable it))
            {
                Vector3 d = hits[i].transform.position - transform.position;
                list.Add((d.magnitude, it, d));
            }
        list.Sort((a, b) => a.dist.CompareTo(b.dist));

        for (int i = 0; i < maxObjectsObserved; i++)
        {
            if (i < list.Count)
            {
                var (dist, it, dir) = list[i];
                Vector3 dirLocal = transform.InverseTransformDirection(dir.normalized);
                OneHotType(sensor, it.type);                                  // 5 floats
                sensor.AddObservation(Mathf.Atan2(dirLocal.x, dirLocal.z));   // bearing
                sensor.AddObservation(Mathf.Clamp01(dist / senseRadius));     // 0..1
            }
            else
            {
                OneHotZeros(sensor);  // 5 zeros
                sensor.AddObservation(0f);
                sensor.AddObservation(1f);
            }
        }
    }
    void OneHotType(VectorSensor s, InteractableType t) { for (int k = 0; k < 5; k++) s.AddObservation(k == (int)t ? 1f : 0f); }
    void OneHotZeros(VectorSensor s) { for (int k = 0; k < 5; k++) s.AddObservation(0f); }

    // ----------------- Actions -----------------
    // Branch 0: Move  (0 none, 1 fwd, 2 back, 3 left, 4 right)
    // Branch 1: Turn  (0 none, 1 left, 2 right)
    // Branch 2: Use   (0 none, 1 interact)
    public override void OnActionReceived(ActionBuffers actions)
    {
        stepCount++;

        // Planar target velocity from discrete actions
        Vector3 target = Vector3.zero;
        switch (actions.DiscreteActions[0])
        {
            case 1: target = transform.forward * moveSpeed; break;
            case 2: target = -transform.forward * moveSpeed; break;
            case 3: target = -transform.right * moveSpeed; break;
            case 4: target = transform.right * moveSpeed; break;
        }
        desiredXZ = new Vector3(target.x, 0f, target.z);

        // Turn
        int turn = actions.DiscreteActions[1];
        if (turn != 0)
        {
            float sign = (turn == 1) ? -1f : 1f;
            transform.Rotate(0f, sign * turnSpeed * Time.fixedDeltaTime, 0f);
        }

        // Interact
        if (actions.DiscreteActions[2] == 1) TryInteract();

        if (printEverySteps > 0 && stepCount % printEverySteps == 0)
            Debug.Log($"[STEP {stepCount}] pos={transform.position} vel={rb.linearVelocity} R={GetCumulativeReward():0.###}");
    }

    void FixedUpdate()
    {
        // Smooth planar motion; keep gravity’s Y
        Vector3 v = rb.linearVelocity;
        Vector3 cur = new Vector3(v.x, 0f, v.z);
        Vector3 nxt = Vector3.MoveTowards(cur, desiredXZ, accel * Time.fixedDeltaTime);
        if (nxt.magnitude > maxSpeed) nxt = nxt.normalized * maxSpeed;
        rb.linearVelocity = new Vector3(nxt.x, v.y, nxt.z);

        // --- Animation control ---
        if (animator != null)
        {
            bool walking = nxt.magnitude > 0.05f;
            animator.SetBool(walkParam, walking);
        }

        StickToGroundStep();
        ClampInsideBounds();
    }

    void ClampInsideBounds()
    {
        if (!gardenBounds) return;

        Bounds b = gardenBounds.bounds;
        Vector3 p = transform.position;
        if (!b.Contains(p))
        {
            Vector3 clamped = new Vector3(
                Mathf.Clamp(p.x, b.min.x, b.max.x),
                p.y,
                Mathf.Clamp(p.z, b.min.z, b.max.z)
            );
            transform.position = clamped;

            // Gentle bounce to stop skating along the edge
            Vector3 v = rb.linearVelocity;
            rb.linearVelocity = new Vector3(-v.x * 0.25f, v.y, -v.z * 0.25f);

            if (printEverySteps > 0 && stepCount % printEverySteps == 0)
                Debug.Log($"[STEP {stepCount}] clamped inside bounds size={b.size}");
        }
    }

    void TryInteract()
    {
        int count = Physics.OverlapSphereNonAlloc(transform.position, 2.0f, hits, interactableMask);
        Interactable best = null; float bestScore = float.PositiveInfinity;

        for (int i = 0; i < count; i++)
            if (hits[i] && hits[i].TryGetComponent(out Interactable it))
            {
                Vector3 dir = it.transform.position - transform.position;
                float angle = Vector3.Angle(transform.forward, dir);
                float score = dir.magnitude + 0.02f * angle; // prefer close & forward
                if (score < bestScore) { bestScore = score; best = it; }
            }

        if (best != null)
        {
            bool ok = best.Interact(this); // Interactable should AddReward(+…)
            if (ok && !seenThisEpisode.Contains(best.GetInstanceID()))
            {
                seenThisEpisode.Add(best.GetInstanceID());
                AddReward(+0.01f); // tiny novelty bonus
            }
            if (printEverySteps > 0 && stepCount % printEverySteps == 0)
                Debug.Log($"[STEP {stepCount}] interact {(ok ? "OK" : "FAIL")} with '{best.name}', R={GetCumulativeReward():0.###}");
        }
    }

    // ----------------- Collision Logging -----------------
    void OnCollisionEnter(Collision c)
    {
        if (!logCollisions) return;
        var cp = c.GetContact(0);
        Debug.Log($"[COLL ENTER s{stepCount}] with '{c.collider.name}' at {cp.point} n={cp.normal}");
    }
    void OnCollisionStay(Collision c)
    {
        if (!logCollisions) return;
        Debug.Log($"[COLL STAY  s{stepCount}] with '{c.collider.name}'");
    }
    void OnCollisionExit(Collision c)
    {
        if (!logCollisions) return;
        Debug.Log($"[COLL EXIT  s{stepCount}] with '{c.collider.name}'");
    }
    void OnTriggerEnter(Collider other)
    {
        if (!logCollisions) return;
        Debug.Log($"[TRIG ENTER s{stepCount}] -> '{other.name}'");
    }
    void OnTriggerExit(Collider other)
    {
        if (!logCollisions) return;
        Debug.Log($"[TRIG EXIT  s{stepCount}] -> '{other.name}'");
    }

    // ----------------- Heuristic (optional manual test) -----------------
    public override void Heuristic(in ActionBuffers a)
    {
        var da = a.DiscreteActions;
        bool w = Input.GetKey(KeyCode.W), s = Input.GetKey(KeyCode.S),
             aL = Input.GetKey(KeyCode.A), dR = Input.GetKey(KeyCode.D);

        da[0] = 0;
        if (w && !s) da[0] = 1;
        else if (s && !w) da[0] = 2;
        else if (aL && !dR) da[0] = 3;
        else if (dR && !aL) da[0] = 4;

        da[1] = Input.GetKey(KeyCode.Q) ? 1 : (Input.GetKey(KeyCode.E) ? 2 : 0);
        da[2] = Input.GetKey(KeyCode.Space) ? 1 : 0;
    }

    // Public setters (interactables may call these)
    public void SetCarryingApple(bool v) => carryingApple = v;
    public void SetSeated(bool v) => isSeated = v;
    public void SetPlayingWater(bool v) => isPlayingWater = v;
}
