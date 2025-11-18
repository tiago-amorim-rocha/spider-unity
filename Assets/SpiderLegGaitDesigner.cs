using UnityEngine;

/// <summary>
/// Simple editor-time leg pose previewer for a 3-segment spider leg.
/// Given a foot target, solves a preferred hip / knee / ankle pose in 2D (XY plane).
/// </summary>
public class SpiderLegGaitDesigner : MonoBehaviour
{
    [Header("Leg bones (3 segments + foot)")]
    public Transform body;     // for body-space reference (COM-ish)
    public Transform hip;      // segment A root
    public Transform knee;     // segment B root
    public Transform ankle;    // segment C root
    public Transform foot;     // current foot (end effector)

    [Header("Segment lengths (world units)")]
    public float length1 = 1.3f;  // hip -> knee
    public float length2 = 1.0f;  // knee -> ankle
    public float length3 = 0.7f;  // ankle -> foot

    [Header("Preferred curvature (deg)")]
    [Tooltip("Amount of knee flex relative to hip when crouched and extended.")]
    public float kneeFlexCrouch   = 110f;  // big bend
    public float kneeFlexExtend   = 70f;   // smaller bend

    [Tooltip("Amount of ankle flex relative to knee when crouched and extended.")]
    public float ankleFlexCrouch  = 40f;
    public float ankleFlexExtend  = 20f;

    [Header("Hip rotation offsets (deg)")]
    [Tooltip("Extra rotation added to the hip->foot direction when crouched.")]
    public float hipOffsetCrouch = 0f;

    [Tooltip("Extra rotation added to the hip->foot direction when extended.")]
    public float hipOffsetExtend = 0f;

    [Header("Gait preview controls")]
    [Tooltip("Optional offset from actual foot to a draggable target.")]
    public Transform footTarget;  // if null, use foot itself as target

    [Tooltip("Draw and solve in edit mode (Scene view handles).")]
    public bool previewInEditMode = true;

    [Tooltip("Mirror sign for this leg: +1 or -1 to get 'backwards' bend on left/right.")]
    public float bendSign = -1f;  // e.g. left leg -1, right leg +1

    [Header("Debug")]
    public bool drawDebug = true;

    [Header("Extension distance mapping")]
    [Tooltip("Fraction of total leg length where s=0 (crouched pose).")]
    [Range(0f, 1f)]
    public float crouchDistanceFraction = 0.4f;

    [Tooltip("Fraction of total leg length where s=1 (extended pose).")]
    [Range(0f, 1f)]
    public float extendDistanceFraction = 0.9f;

    [Header("SpiderController integration (optional)")]
    [Tooltip("If assigned, leg data will be pulled from this SpiderController.")]
    public SpiderController spiderController;

    [Tooltip("Index into SpiderController.legs array for this designer.")]
    public int spiderLegIndex = 0;

    [Tooltip("When syncing from SpiderController, also derive segment lengths automatically.")]
    public bool syncLengthsFromController = true;

    /// <summary>
    /// Try to populate body / hip / knee / ankle / foot and segment lengths
    /// from an attached SpiderController and the selected leg index.
    /// </summary>
    public void SyncFromSpiderController()
    {
        SpiderController controller = spiderController != null
            ? spiderController
            : GetComponent<SpiderController>();

        if (controller == null || controller.legs == null || controller.legs.Length == 0)
            return;

        spiderController = controller;

        int legCount = controller.legs.Length;
        int index = Mathf.Clamp(spiderLegIndex, 0, legCount - 1);
        spiderLegIndex = index;

        var leg = controller.legs[index];

        // Basic transform wiring.
        if (controller.body != null)
            body = controller.body.transform;

        hip   = leg.segment1;
        knee  = leg.segment2;
        ankle = leg.segment3;
        foot  = leg.foot;

        // Derive segment lengths so the preview roughly matches existing geometry.
        if (syncLengthsFromController)
        {
            if (hip != null && knee != null)
                length1 = Vector2.Distance(hip.position, knee.position);

            if (knee != null && ankle != null)
                length2 = Vector2.Distance(knee.position, ankle.position);

            if (ankle != null && foot != null)
                length3 = Vector2.Distance(ankle.position, foot.position);

            // Fallback to SpiderController's parametric lengths if distances are unusable.
            bool invalid = length1 <= 0f || length2 <= 0f || length3 <= 0f;
            if (invalid)
            {
                float baseLength = controller.baseSegmentLength > 0f ? controller.baseSegmentLength : 1f;
                float ratio = controller.segmentLengthRatio > 0f ? controller.segmentLengthRatio : 1.3f;

                float L1 = baseLength;
                float L2 = L1 / ratio;
                float L3 = L2 / ratio;

                if (length1 <= 0f) length1 = L1;
                if (length2 <= 0f) length2 = L2;
                if (length3 <= 0f) length3 = L3;
            }
        }

        // Default bend sign: assume leg 0 = left (-1), leg 1 = right (+1).
        if (controller.legs.Length >= 2)
        {
            bendSign = index == 0 ? -1f : 1f;
        }
    }

    private void Reset()
    {
        // When first added, try to auto-wire from a SpiderController on the same GameObject.
        SyncFromSpiderController();
    }

    /// <summary>
    /// Computes and applies a preferred hip / knee / ankle pose so the foot
    /// roughly reaches the current target position.
    /// </summary>
    public void UpdatePoseFromFootTarget()
    {
        // Ensure required transforms are present.
        if (hip == null || knee == null || ankle == null || foot == null)
        {
            return;
        }

        // 1) Pick target: use footTarget if assigned, otherwise use foot.
        Transform targetT = footTarget != null ? footTarget : foot;
        if (targetT == null)
        {
            return;
        }

        // 2) Compute vector hip -> target in world space (projected onto XY plane).
        Vector3 hipPos = hip.position;
        Vector3 targetPos = targetT.position;
        Vector2 r = new Vector2(targetPos.x - hipPos.x, targetPos.y - hipPos.y);
        float d = r.magnitude;
        if (d < 1e-4f)
        {
            return;
        }

        // Direction from hip to target in world space (degrees, XY plane).
        float dirDeg = Mathf.Atan2(r.y, r.x) * Mathf.Rad2Deg;

        // 3) Compute normalized extension factor s in [0,1] based on distance.
        float total = length1 + length2 + length3;
        if (total <= 1e-4f)
        {
            return;
        }

        // Approximate crouched vs extended span. These are tunable so you can
        // decide at which hip-foot distance you want the "crouch" vs "extend"
        // poses to occur.
        float fMin = Mathf.Clamp01(crouchDistanceFraction);
        float fMax = Mathf.Clamp01(extendDistanceFraction);
        if (fMax <= fMin + 0.001f)
            fMax = Mathf.Min(1f, fMin + 0.001f);

        float dMin = fMin * total;  // crouched-ish
        float dMax = fMax * total;  // extended-ish
        float s = Mathf.InverseLerp(dMin, dMax, d);
        s = Mathf.Clamp01(s);

        // 4) Compute preferred absolute flex angles for knee and ankle.
        // These are interpreted as real bend angles (0 = straight, 90 = right angle, etc.),
        // not just weights. We do not force the foot to hit the target exactly; instead,
        // the target distance controls extension factor s, which picks a canonical pose.
        float kneeFlex = Mathf.Max(0f, Mathf.Lerp(kneeFlexCrouch, kneeFlexExtend, s));
        float ankleFlex = Mathf.Max(0f, Mathf.Lerp(ankleFlexCrouch, ankleFlexExtend, s));
        float hipOffset = Mathf.Lerp(hipOffsetCrouch, hipOffsetExtend, s);

        float bend = bendSign >= 0f ? 1f : -1f;

        float kneeDeltaDeg = bend * kneeFlex;
        float ankleDeltaDeg = bend * ankleFlex;

        // World-space joint angles (2D: rotations around Z axis).
        float hipWorldDeg   = dirDeg + hipOffset;
        float kneeWorldDeg  = hipWorldDeg + kneeDeltaDeg;
        float ankleWorldDeg = kneeWorldDeg + ankleDeltaDeg;

        // 7) Compute joint positions along the solved chain and apply both
        //    positions and rotations to the free segments. The hip segment
        //    root stays anchored at its current position.

        // Hip root is fixed at current transform position.
        Vector2 hipRoot = new Vector2(hipPos.x, hipPos.y);

        float hipRadWorld   = hipWorldDeg * Mathf.Deg2Rad;
        float kneeRadWorld  = kneeWorldDeg * Mathf.Deg2Rad;
        float ankleRadWorld = ankleWorldDeg * Mathf.Deg2Rad;

        Vector2 dir1 = new Vector2(Mathf.Cos(hipRadWorld), Mathf.Sin(hipRadWorld));
        Vector2 kneePos = hipRoot + dir1 * length1;

        Vector2 dir2 = new Vector2(Mathf.Cos(kneeRadWorld), Mathf.Sin(kneeRadWorld));
        Vector2 anklePos = kneePos + dir2 * length2;

        Vector2 dir3 = new Vector2(Mathf.Cos(ankleRadWorld), Mathf.Sin(ankleRadWorld));
        Vector2 footPos = anklePos + dir3 * length3;

        // Apply world-space transforms.
        hip.rotation = Quaternion.Euler(0f, 0f, hipWorldDeg);

        knee.position  = new Vector3(kneePos.x,  kneePos.y,  knee.position.z);
        knee.rotation  = Quaternion.Euler(0f, 0f, kneeWorldDeg);

        ankle.position = new Vector3(anklePos.x, anklePos.y, ankle.position.z);
        ankle.rotation = Quaternion.Euler(0f, 0f, ankleWorldDeg);

        if (foot != null)
        {
            foot.position = new Vector3(footPos.x, footPos.y, foot.position.z);
        }
    }
}
