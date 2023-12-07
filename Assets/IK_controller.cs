#if UNITY_EDITOR
using UnityEditor;
#endif
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using TMPro;
using System.IO;
using System.Linq;
using System;

public class IK_Controller : MonoBehaviour
{
    public InputDevice _rightController;
    public GameObject RobotBase;
    private GameObject[] jointList = new GameObject[6];
    private UR3_Solver Robot = new();
    private IK_toolkit iK_Toolkit = new();
    private APIManager aPIManager = new();
    public int ChainLength = 5;
    public Transform Target;
    public TMP_Text text;
    public float repeatRate = 4f;
    private string filePath;
    private GameObject sphere;
    public bool waspressed = false;
    public Vector3 origPosition;
    public Vector3 currOrig = new Vector3(0.1362f, 0.3557f, 0.0738f);
    // Start is called before the first frame update
    void Start()
    {
        // filePath = Path.Combine(Application.persistentDataPath, "log.txt");
        Logger.ClearLogFile();
        // Logger.WriteToLogFile(Application.persistentDataPath);
        // var pathRead = Logger.ReadFromLogFile();
        // text.text += pathRead;
        InitializeJoints();
        InitializeCube();
        if(!_rightController.isValid)
        {
            InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, ref _rightController);
        }
        // var targetPosition = Target.position;
        // var targetRotation = Target.rotation;
        // Vector3 euler = targetRotation.eulerAngles;
        // text.text += "\n\ncoordinates: " + targetPosition.x + " " + targetPosition.y + " " + targetPosition.z + "\nangles: " + euler.x + " " + euler.y + " " + euler.z + "\n\n";
        // var targetPositionb = GetPositionRootSpace(RobotBase.transform);
        // var targetRotationb = GetRotationRootSpace(RobotBase.transform);
        // Vector3 eulerb = targetRotationb.eulerAngles;
        // text.text += "\n\nrobot coordinates: " + targetPositionb.x + " " + targetPositionb.y + " " + targetPositionb.z + "\nangles: " + eulerb.x + " " + eulerb.y + " " + eulerb.z + "\n\n";
        InvokeRepeating("UpdatePosition", 0.5f, repeatRate);
    }

    void InitializeJoints()
    {
        var RobotChildren = RobotBase.GetComponentsInChildren<Transform>();
        for (int i = 1; i < RobotChildren.Length; i++)
        {   
            jointList[i-1] = RobotChildren[i].gameObject;
        }
    }

    void InitializeCube()
    {
        sphere = GameObject.Find("SphereX");
    }

    private void InitializeInputDevice(InputDeviceCharacteristics inputCharacteristics, ref InputDevice inputDevice)
    {
        List<InputDevice> devices = new();
        //Call InputDevices to see if it can find any devices with the characteristics we're looking for
        InputDevices.GetDevicesWithCharacteristics(inputCharacteristics, devices);

        //Our hands might not be active and so they will not be generated from the search.
        //We check if any devices are found here to avoid errors.
        if (devices.Count > 0)
        {
            inputDevice = devices[0];
        }
    }

    void UpdatePosition()
    {
        bool isIndexFingerPressed;
        if (_rightController.TryGetFeatureValue(CommonUsages.triggerButton, out isIndexFingerPressed) && isIndexFingerPressed)
        {
            if (waspressed == false)
            {
                waspressed = true;
                origPosition = Target.position;
            }
            ResolveIK();

        }
        else
        {
            if (waspressed == true)
            {
                currOrig = currOrig + Target.position - origPosition;
            }
            waspressed = false;
        }
    }

    private void ResolveIK()
    {
        // if (Target == null)
        //     return;

        // var targetPosition = GetPositionRootSpace(Target);
        // var targetPosition = sphere.transform.position;
        var targetPosition = currOrig + Target.position - origPosition;
        var targetRotation = Target.rotation;
        Vector3 euler = targetRotation.eulerAngles;
        // Logger.WriteToLogFile(targetPosition.x + " " + targetPosition.y + " " + targetPosition.z);
        // Logger.WriteToLogFile(0.36f + " " + 0.5f + " " + 0.2f);
        // var content = Logger.ReadFromLogFile();
        // text.text = content;

        Robot.Solve(targetPosition.z, -targetPosition.y, -targetPosition.x, euler.x * Mathf.Deg2Rad, euler.y * Mathf.Deg2Rad, euler.z * Mathf.Deg2Rad, text);

        // var angles = targetPosition.x + " " + targetPosition.y + " " + targetPosition.z;
        // var coordinates = "0.36 0.5 0.2";
        // text.text += "\nhere " + coordinates;
        
        
        // Robot.Solve(targetPosition.x, targetPosition.y, targetPosition.z, 0f, 0f, 0f, text);
        // iK_Toolkit.IK_Calculation(Target);
        // Robot.Solve(1.358f, 0.058f, -0.015f, 0f, 0f, 0f, text);
        // text.text += "\n\ncoordinates: " + targetPosition.x + " " + targetPosition.y + " " + targetPosition.z + "\nangles: " + euler.x + " " + euler.y + " " + euler.z;

        // text.text += "\nangles: ";
        string coords = targetPosition.x + " " + targetPosition.y + " " + targetPosition.z;
        for (int i = 0; i < 6; i++)
        {
            if (float.IsNaN(Robot.solutionArray[i]))
            {
                if ((i == 1) || (i == 3))
                {
                    Robot.solutionArray[i] = -(float)Math.PI/2;
                }
                else
                {
                    Robot.solutionArray[i] = 0;
                }
            }
        }

        string angles = string.Join(" ", Robot.solutionArray);
        // string angles = string.Join(" ", iK_Toolkit.solutionArray.Select(v => double.IsNaN(v) ? 0 : v));
        aPIManager.SendControllerCoordinates(angles + ' ' + coords, text);

        for (int j = 0; j < 6; j++)
        {
            text.text += Robot.solutionArray[j] * Mathf.Rad2Deg + " ";
            // Vector3 currentRotation = jointList[j].transform.localEulerAngles;

            // if (j == 0 || j == 4)
            // {
            //     currentRotation.y = Robot.solutionArray[j];  // Rotate around the y-axis
            //     // currentRotation.y = iK_Toolkit.solutionArray[j]; // Rotate around the y-axis
            // }
            // else
            // {
            //     currentRotation.z = Robot.solutionArray[j];  // Rotate around the z-axis for other joints
            //     // currentRotation.z = iK_Toolkit.solutionArray[j]; // Rotate around the y-axis
            // }

            // jointList[j].transform.localEulerAngles = currentRotation;
        }

    }

    private Vector3 GetPositionRootSpace(Transform current)
    {
        if (Target == null)
            return current.position;
        else
            return Quaternion.Inverse(Target.transform.rotation) * (current.position - Target.transform.position);
    }

    private Quaternion GetRotationRootSpace(Transform current)
    {
        //inverse(after) * before => rot: before -> after
        if (RobotBase == null)
            return current.rotation;
        else
            return Quaternion.Inverse(current.rotation) * RobotBase.transform.rotation;
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