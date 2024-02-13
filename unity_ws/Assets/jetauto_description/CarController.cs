using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";

    private float horizontalInput;
    private float verticalInput;

    [SerializeField] private float motorForce = 10f;
    [SerializeField] private float maxAngle = 20f;

    [SerializeField] private Transform viewFL;
    [SerializeField] private Transform viewFR;
    [SerializeField] private Transform viewBL;
    [SerializeField] private Transform viewBR;

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
        horizontalInput = Input.GetAxis(HORIZONTAL);
        verticalInput = Input.GetAxis(VERTICAL);
    }

    private void HandleMotor()
    {
        float currentTurnAngle = maxAngle * horizontalInput;
        float currentAccelForce = motorForce * verticalInput;

        colliderFL.motorTorque = currentAccelForce;
        colliderFR.motorTorque = currentAccelForce;

        colliderFL.steerAngle = currentTurnAngle;
        colliderFR.steerAngle = currentTurnAngle;

        UpdateWheel(colliderFL, viewFL);
        UpdateWheel(colliderFR, viewFR);
        UpdateWheel(colliderBL, viewBL);
        UpdateWheel(colliderBR, viewBR);
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
