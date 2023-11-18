#if UNITY_EDITOR
using UnityEditor;
#endif
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class IK_Controller : MonoBehaviour
{
    public InputDevice _rightController;
    public GameObject RobotBase;
    private bool isIndexFingerPressed;
    private GameObject[] jointList = new GameObject[6];
    private UR3_Solver Robot = new UR3_Solver();
    public int ChainLength = 5;
    public Transform Target;


    // Start is called before the first frame update
    void Awake()
    {
        InitializeJoints();
        if(!_rightController.isValid)
        InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, ref _rightController);
    }

    void InitializeJoints()
    {
        var RobotChildren = RobotBase.GetComponentsInChildren<Transform>();
        for (int i = 0; i < RobotChildren.Length; i++)
        {
            jointList[i] = RobotChildren[i].gameObject;
        }
    }

    private void InitializeInputDevice(InputDeviceCharacteristics inputCharacteristics, ref InputDevice inputDevice)
    {
        List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(inputCharacteristics, devices);

        //Our hands might not be active and so they will not be generated from the search.
        //We check if any devices are found here to avoid errors.
        if (devices.Count > 0)
        {
            inputDevice = devices[0];
        }
    }

    void LateUpdate()
    {
        if (_rightController.TryGetFeatureValue(CommonUsages.triggerButton, out isIndexFingerPressed) && isIndexFingerPressed)
        {
            ResolveIK();
        }
    }

    private void ResolveIK()
    {
        if (Target == null)
            return;

        var targetPosition = GetPositionRootSpace(Target);
        var targetRotation = GetRotationRootSpace(Target);
        Vector3 euler = targetRotation.eulerAngles;

        var solution_array = Robot.Solve(targetPosition.x, targetPosition.y, targetPosition.z, euler.x, euler.y, euler.z);

        for (int j = 0; j < 6; j++)
        {
            Vector3 currentRotation = jointList[j].transform.localEulerAngles;
            currentRotation.z = Robot.solutionArray[j];
            jointList[j].transform.localEulerAngles = currentRotation;
        }
    }

    private Vector3 GetPositionRootSpace(Transform current)
    {
        if (RobotBase == null)
            return current.position;
        else
            return Quaternion.Inverse(RobotBase.transform.rotation) * (current.position - RobotBase.transform.position);
    }

    private void SetPositionRootSpace(Transform current, Vector3 position)
    {
        if (RobotBase == null)
            current.position = position;
        else
            current.position = RobotBase.transform.rotation * position + RobotBase.transform.position;
    }

    private Quaternion GetRotationRootSpace(Transform current)
    {
        //inverse(after) * before => rot: before -> after
        if (RobotBase == null)
            return current.rotation;
        else
            return Quaternion.Inverse(current.rotation) * RobotBase.transform.rotation;
    }

    private void SetRotationRootSpace(Transform current, Quaternion rotation)
    {
        if (RobotBase == null)
            current.rotation = rotation;
        else
            current.rotation = RobotBase.transform.rotation * rotation;
    }

    void OnDrawGizmos()
    {
#if UNITY_EDITOR
        var current = this.transform;
        for (int i = 0; i < ChainLength && current != null && current.parent != null; i++)
        {
            var scale = Vector3.Distance(current.position, current.parent.position) * 0.1f;
            Handles.matrix = Matrix4x4.TRS(current.position, Quaternion.FromToRotation(Vector3.up, current.parent.position - current.position), new Vector3(scale, Vector3.Distance(current.parent.position, current.position), scale));
            Handles.color = Color.green;
            Handles.DrawWireCube(Vector3.up * 0.5f, Vector3.one);
            current = current.parent;
        }
#endif
    }

}