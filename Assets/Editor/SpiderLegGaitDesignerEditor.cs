using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(SpiderLegGaitDesigner))]
public class SpiderLegGaitDesignerEditor : Editor
{
    public override void OnInspectorGUI()
    {
        // Draw default inspector fields first.
        DrawDefaultInspector();

        var designer = (SpiderLegGaitDesigner)target;
        if (designer == null)
        {
            return;
        }

        GUILayout.Space(4f);

        // Manual pose update button.
        if (GUILayout.Button("Update Pose From Foot Target"))
        {
            // Record transforms for Undo before modifying rotations.
            if (designer.hip != null)
            {
                Undo.RecordObject(designer.hip, "Update Spider Leg Pose");
            }
            if (designer.knee != null)
            {
                Undo.RecordObject(designer.knee, "Update Spider Leg Pose");
            }
            if (designer.ankle != null)
            {
                Undo.RecordObject(designer.ankle, "Update Spider Leg Pose");
            }

            designer.UpdatePoseFromFootTarget();

            // Mark objects dirty so changes persist in edit mode.
            EditorUtility.SetDirty(designer);
            if (designer.hip != null)   EditorUtility.SetDirty(designer.hip);
            if (designer.knee != null)  EditorUtility.SetDirty(designer.knee);
            if (designer.ankle != null) EditorUtility.SetDirty(designer.ankle);
        }

        // Optional: pull wiring from SpiderController on the same GameObject.
        if (GUILayout.Button("Sync From SpiderController"))
        {
            Undo.RecordObject(designer, "Sync Spider Leg From Controller");
            designer.SyncFromSpiderController();
            EditorUtility.SetDirty(designer);
        }

    }

    private void OnSceneGUI()
    {
        var designer = (SpiderLegGaitDesigner)target;
        if (designer == null)
        {
            return;
        }

        if (!designer.previewInEditMode)
        {
            return;
        }

        // Choose the transform to manipulate: custom foot target or the foot itself.
        Transform t = designer.footTarget != null ? designer.footTarget : designer.foot;
        if (t == null)
        {
            return;
        }

        // Draw a position handle for the foot target in world space.
        EditorGUI.BeginChangeCheck();
        Vector3 newPos = Handles.PositionHandle(t.position, Quaternion.identity);
        if (EditorGUI.EndChangeCheck())
        {
            Undo.RecordObject(t, "Move foot target");
            t.position = newPos;

            // Record leg segments for Undo before updating their transforms.
            if (designer.hip != null)
                Undo.RecordObject(designer.hip, "Update Spider Leg Pose");
            if (designer.knee != null)
                Undo.RecordObject(designer.knee, "Update Spider Leg Pose");
            if (designer.ankle != null)
                Undo.RecordObject(designer.ankle, "Update Spider Leg Pose");
            if (designer.foot != null)
                Undo.RecordObject(designer.foot, "Update Spider Leg Pose");

            designer.UpdatePoseFromFootTarget();

            // Ensure scene refreshes and changes are saved.
            EditorUtility.SetDirty(designer);
            EditorUtility.SetDirty(t);
            if (designer.hip != null)   EditorUtility.SetDirty(designer.hip);
            if (designer.knee != null)  EditorUtility.SetDirty(designer.knee);
            if (designer.ankle != null) EditorUtility.SetDirty(designer.ankle);
            if (designer.foot != null)  EditorUtility.SetDirty(designer.foot);
        }

        // Optional debug drawing for leg segments and target direction.
        if (designer.drawDebug)
        {
            Handles.color = Color.cyan;

            if (designer.hip != null && designer.knee != null)
            {
                Handles.DrawLine(designer.hip.position, designer.knee.position);
            }

            if (designer.knee != null && designer.ankle != null)
            {
                Handles.DrawLine(designer.knee.position, designer.ankle.position);
            }

            if (designer.ankle != null && designer.foot != null)
            {
                Handles.DrawLine(designer.ankle.position, designer.foot.position);
            }

            // Draw line from hip to current target to visualize reach.
            if (designer.hip != null && t != null)
            {
                Handles.color = Color.yellow;
                Handles.DrawLine(designer.hip.position, t.position);
            }
        }
    }
}
