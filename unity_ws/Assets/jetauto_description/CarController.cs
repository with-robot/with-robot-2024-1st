using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float[] inputs = new float[7];
    private float verticalInput;

    [SerializeField] private float motorForce = 10f;
    [SerializeField] private float maxAngle = 20f;

    [SerializeField] private Transform viewFL;
    [SerializeField] private Transform viewFR;
    [SerializeField] private Transform viewBL;
    [SerializeField] private Transform viewBR;
    [SerializeField] private Transform arm_link1;
    [SerializeField] private Transform arm_link2;
    [SerializeField] private Transform arm_link3;
    [SerializeField] private Transform arm_link4;
    [SerializeField] private Transform l_in_link;
    [SerializeField] private Transform r_in_link;

    [SerializeField] private WheelCollider colliderFL;
    [SerializeField] private WheelCollider colliderFR;
    [SerializeField] private WheelCollider colliderBL;
    [SerializeField] private WheelCollider colliderBR;

    private void FixedUpdate()
    {
        GetInput();
        HandleMotor();
    }

    private void GetInput()
    {
        inputs[0] = Input.GetAxis(HORIZONTAL);
        inputs[1] = Input.GetAxis(VERTICAL);
        inputs[2] = Input.GetKey(KeyCode.R) ? 1 : (Input.GetKey(KeyCode.F) ? -1 : 0);
        inputs[3] = Input.GetKey(KeyCode.T) ? 1 : (Input.GetKey(KeyCode.G) ? -1 : 0);
        inputs[4] = Input.GetKey(KeyCode.Y) ? 1 : (Input.GetKey(KeyCode.H) ? -1 : 0);
        inputs[5] = Input.GetKey(KeyCode.U) ? 1 : (Input.GetKey(KeyCode.J) ? -1 : 0);
        inputs[6] = Input.GetKey(KeyCode.I) ? 1 : (Input.GetKey(KeyCode.K) ? -1 : 0);
    }

    private bool ValidaeAction(float angle, float dir, float min, float max)
    {
        if (angle > 180)
        {
            angle -= 360;
        }
        else if (angle < -180)
        {
            angle += 360;
        }
        if (0 < dir && (angle < max))
        {
            return true;
        }
        else if (dir < 0 && min < angle)
        {
            return true;
        }
        return false;
    }

    private void HandleMotor()
    {
        float currentTurnAngle = maxAngle * inputs[0];
        float currentAccelForce = motorForce * inputs[1];

        colliderFL.motorTorque = currentAccelForce;
        colliderFR.motorTorque = currentAccelForce;

        colliderFL.steerAngle = currentTurnAngle;
        colliderFR.steerAngle = currentTurnAngle;

        UpdateWheel(colliderFL, viewFL);
        UpdateWheel(colliderFR, viewFR);
        UpdateWheel(colliderBL, viewBL);
        UpdateWheel(colliderBR, viewBR);

        //
        if (ValidaeAction(arm_link1.localEulerAngles.y - 90, inputs[2], -90, 90))
        {
            arm_link1.Rotate(new Vector3(0, inputs[2], 0));
        }
        if (ValidaeAction(arm_link2.localEulerAngles.z, inputs[3], -90, 90))
        {
            arm_link2.Rotate(new Vector3(0, 0, inputs[3]));
        }
        if (ValidaeAction(arm_link3.localEulerAngles.z, inputs[4], -90, 90))
        {
            arm_link3.Rotate(new Vector3(0, 0, inputs[4]));
        }
        if (ValidaeAction(arm_link4.localEulerAngles.z, inputs[5], -90, 90))
        {
            arm_link4.Rotate(new Vector3(0, 0, inputs[5]));
        }
        if (ValidaeAction(l_in_link.localEulerAngles.x, inputs[6], 0, 90))
        {
            l_in_link.Rotate(new Vector3(inputs[6], 0, 0));
        }
        if (ValidaeAction(-r_in_link.localEulerAngles.x, inputs[6], 0, 90))
        {
            r_in_link.Rotate(new Vector3(-inputs[6], 0, 0));
        }
    }

    private void UpdateWheel(WheelCollider col, Transform trans)
    {
        Vector3 position;
        Quaternion rotation;
        col.GetWorldPose(out position, out rotation);

        trans.position = position;
        trans.rotation = rotation * Quaternion.Euler(0f, 0f, 90f);
    }

}
