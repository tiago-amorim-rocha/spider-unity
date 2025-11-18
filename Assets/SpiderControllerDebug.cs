using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

/// <summary>
/// Handles the scene/GUI debug overlays for <see cref="SpiderController"/>.
/// Keeps SpiderController free of editor/logging concerns.
/// </summary>
public class SpiderControllerDebug : MonoBehaviour
{
    [Header("Debug output")]
    [Tooltip("Toggle gizmos, HUD, and periodic logging for the spider controller.")]
    public bool drawDebug = true;

    [Tooltip("Seconds between limit-force log lines while debugging.")]
    public float logInterval = 0.5f;

    [SerializeField]
    [Tooltip("Controller to visualize (defaults to the component on the same GameObject).")]
    private SpiderController controller;

    private float _logTimer;
    private JointLimitSnapshot _hipLeft;
    private JointLimitSnapshot _hipRight;
    private JointLimitSnapshot _kneeLeft;
    private JointLimitSnapshot _kneeRight;
    private JointLimitSnapshot _ankleLeft;
    private JointLimitSnapshot _ankleRight;

    private struct JointLimitSnapshot
    {
        public float Angle;
        public float Min;
        public float Max;
        public float Torque;
    }

    private void Reset()
    {
        controller = GetComponent<SpiderController>();
    }

    private void Awake()
    {
        if (controller == null)
            controller = GetComponent<SpiderController>();

        controller?.RegisterDebugger(this);
    }

    internal void UpdateJointDebug(
        bool isLeft,
        float hipAngle, float hipMin, float hipMax, float hipTorque,
        float kneeAngle, float kneeMin, float kneeMax, float kneeTorque,
        float ankleAngle, float ankleMin, float ankleMax, float ankleTorque)
    {
        var hip = new JointLimitSnapshot
        {
            Angle = hipAngle,
            Min = hipMin,
            Max = hipMax,
            Torque = hipTorque
        };

        var knee = new JointLimitSnapshot
        {
            Angle = kneeAngle,
            Min = kneeMin,
            Max = kneeMax,
            Torque = kneeTorque
        };

        var ankle = new JointLimitSnapshot
        {
            Angle = ankleAngle,
            Min = ankleMin,
            Max = ankleMax,
            Torque = ankleTorque
        };

        if (isLeft)
        {
            _hipLeft = hip;
            _kneeLeft = knee;
            _ankleLeft = ankle;
        }
        else
        {
            _hipRight = hip;
            _kneeRight = knee;
            _ankleRight = ankle;
        }
    }

    private void FixedUpdate()
    {
        if (!drawDebug || controller == null)
            return;

        float interval = Mathf.Max(0.01f, logInterval);
        _logTimer += Time.fixedDeltaTime;
        if (_logTimer >= interval)
        {
            _logTimer = 0f;
            //LogJointLimits();
        }
    }

    private void LogJointLimits()
    {
        Debug.Log(
            $"[HipLimits]   L angle={_hipLeft.Angle:F1} limits=[{_hipLeft.Min:F1},{_hipLeft.Max:F1}] tau={_hipLeft.Torque:F2} | " +
            $"R angle={_hipRight.Angle:F1} limits=[{_hipRight.Min:F1},{_hipRight.Max:F1}] tau={_hipRight.Torque:F2}\n" +
            $"[KneeLimits]  L angle={_kneeLeft.Angle:F1} limits=[{_kneeLeft.Min:F1},{_kneeLeft.Max:F1}] tau={_kneeLeft.Torque:F2} | " +
            $"R angle={_kneeRight.Angle:F1} limits=[{_kneeRight.Min:F1},{_kneeRight.Max:F1}] tau={_kneeRight.Torque:F2}\n" +
            $"[AnkleLimits] L angle={_ankleLeft.Angle:F1} limits=[{_ankleLeft.Min:F1},{_ankleLeft.Max:F1}] tau={_ankleLeft.Torque:F2} | " +
            $"R angle={_ankleRight.Angle:F1} limits=[{_ankleRight.Min:F1},{_ankleRight.Max:F1}] tau={_ankleRight.Torque:F2}");
    }

    private void OnGUI()
    {
        if (!drawDebug || controller == null || controller.body == null)
            return;

        GUI.Label(new Rect(10, 10, 350, 20),
            $"body pos: {controller.body.position}  vel: {controller.body.linearVelocity}");
        GUI.Label(new Rect(10, 30, 350, 40),
            $"Up/Down = vertical accel (gain {controller.verticalAccelGain:F1})\nLeft/Right = horizontal accel (gain {controller.horizontalAccelGain:F1})");
    }

    private void OnDrawGizmos()
    {
        if (!drawDebug)
            return;

        if (controller == null)
            controller = GetComponent<SpiderController>();

        if (controller == null || controller.body == null)
            return;

        if (!controller.TryGetControllerLegs(out var leftLeg, out var rightLeg))
            return;

        Vector2 leftHipPos = leftLeg.Hip.worldCenterOfMass;
        Vector2 rightHipPos = rightLeg.Hip.worldCenterOfMass;
        Vector2 leftKneePos = leftLeg.Knee.worldCenterOfMass;
        Vector2 rightKneePos = rightLeg.Knee.worldCenterOfMass;
        Vector2 leftAnklePos = leftLeg.Ankle.worldCenterOfMass;
        Vector2 rightAnklePos = rightLeg.Ankle.worldCenterOfMass;

        DrawHipAngles(leftHipPos, leftLeg.Hip.rotation, true);
        DrawHipAngles(rightHipPos, rightLeg.Hip.rotation, false);

        DrawKneeAngles(leftHipPos, leftKneePos, leftLeg.Knee.rotation, true);
        DrawKneeAngles(rightHipPos, rightKneePos, rightLeg.Knee.rotation, false);

        DrawAnkleAngles(leftKneePos, leftAnklePos, leftLeg.Ankle.rotation, true);
        DrawAnkleAngles(rightKneePos, rightAnklePos, rightLeg.Ankle.rotation, false);
    }

    private static Vector3 DirFromDegrees(float angleDeg, float length)
    {
        float rad = angleDeg * Mathf.Deg2Rad;
        return new Vector3(Mathf.Cos(rad), Mathf.Sin(rad), 0f) * length;
    }

    private static void DrawThickLine(Vector3 from, Vector3 to, Color color, float thickness = 3f)
    {
#if UNITY_EDITOR
        Color prevColor = Handles.color;
        var prevZTest = Handles.zTest;
        Handles.color = color;
        Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
        Handles.DrawAAPolyLine(thickness, new Vector3[] { from, to });
        Handles.zTest = prevZTest;
        Handles.color = prevColor;
#else
        Gizmos.color = color;
        Gizmos.DrawLine(from, to);
#endif
    }

    private void DrawHipAngles(Vector2 hipPos, float hipDeg, bool isLeft)
    {
        Vector2 bodyPos2 = controller.body.worldCenterOfMass;
        Vector2 radial = hipPos - bodyPos2;
        if (radial.sqrMagnitude <= SpiderController.LeverEpsilon)
            return;

        float parentDeg = Mathf.Atan2(radial.y, radial.x) * Mathf.Rad2Deg;
        controller.GetHipLimitRange(isLeft, out float freeMin, out float freeMax);
        float minWorld = parentDeg + freeMin;
        float maxWorld = parentDeg + freeMax;

        const float limitLength = 0.6f;
        const float actualLength = 0.7f;
        Vector3 origin = new Vector3(hipPos.x, hipPos.y, 0f);

        Color lowerColor = Color.red;
        Color upperColor = new Color(1f, 0.5f, 0f);
        Color actualColor = new Color(0.35f, 0f, 0.6f);

        DrawThickLine(origin, origin + DirFromDegrees(minWorld, limitLength), lowerColor, 5f);
        DrawThickLine(origin, origin + DirFromDegrees(maxWorld, limitLength), upperColor, 5f);
        DrawThickLine(origin, origin + DirFromDegrees(hipDeg, actualLength), actualColor, 6f);
    }

    private void DrawKneeAngles(Vector2 hipPos, Vector2 kneePos, float kneeDeg, bool isLeft)
    {
        Vector2 parentDir = kneePos - hipPos;
        if (parentDir.sqrMagnitude <= SpiderController.LeverEpsilon)
            return;

        float parentDeg = Mathf.Atan2(parentDir.y, parentDir.x) * Mathf.Rad2Deg;
        controller.GetKneeLimitRange(isLeft, out float freeMin, out float freeMax);
        float minWorld = parentDeg + freeMin;
        float maxWorld = parentDeg + freeMax;

        const float limitLength = 0.5f;
        const float actualLength = 0.6f;
        Vector3 origin = new Vector3(kneePos.x, kneePos.y, 0f);

        Color lowerColor = Color.red;
        Color upperColor = new Color(1f, 0.5f, 0f);
        Color actualColor = new Color(0.35f, 0f, 0.6f);

        DrawThickLine(origin, origin + DirFromDegrees(minWorld, limitLength), lowerColor, 4f);
        DrawThickLine(origin, origin + DirFromDegrees(maxWorld, limitLength), upperColor, 4f);
        DrawThickLine(origin, origin + DirFromDegrees(kneeDeg, actualLength), actualColor, 5f);
    }

    private void DrawAnkleAngles(Vector2 kneePos, Vector2 anklePos, float ankleDeg, bool isLeft)
    {
        Vector2 parentDir = anklePos - kneePos;
        if (parentDir.sqrMagnitude <= SpiderController.LeverEpsilon)
            return;

        float parentDeg = Mathf.Atan2(parentDir.y, parentDir.x) * Mathf.Rad2Deg;
        controller.GetAnkleLimitRange(isLeft, out float freeMin, out float freeMax);
        float minWorld = parentDeg + freeMin;
        float maxWorld = parentDeg + freeMax;

        const float limitLength = 0.4f;
        const float actualLength = 0.5f;
        Vector3 origin = new Vector3(anklePos.x, anklePos.y, 0f);

        Color lowerColor = Color.red;
        Color upperColor = new Color(1f, 0.5f, 0f);
        Color actualColor = new Color(0.35f, 0f, 0.6f);

        DrawThickLine(origin, origin + DirFromDegrees(minWorld, limitLength), lowerColor, 3f);
        DrawThickLine(origin, origin + DirFromDegrees(maxWorld, limitLength), upperColor, 3f);
        DrawThickLine(origin, origin + DirFromDegrees(ankleDeg, actualLength), actualColor, 4f);
    }
}
