using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using System;
using System.Collections;

public class CarController : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";


    private float[] inputs = new float[7];
    private float verticalInput;

    [SerializeField] private bool unityControl = true;

    [SerializeField] private float motorForce = 2f;
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
    [SerializeField] private Transform l_out_link;
    [SerializeField] private Transform l_link;


    [SerializeField] private Transform r_in_link;

    [SerializeField] private Transform r_out_link;
    [SerializeField] private Transform r_link;

    [SerializeField] private WheelCollider colliderFL;
    [SerializeField] private WheelCollider colliderFR;
    [SerializeField] private WheelCollider colliderBL;
    [SerializeField] private WheelCollider colliderBR;

    [SerializeField] private string jetauto_car_topic = "jetauto_car/cmd_vel";
    [SerializeField] private string jetauto_arm_topic = "jetauto_arm/cmd_vel";
    ROSConnection ros;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(jetauto_car_topic, CarCallBack);
        ros.Subscribe<JointStateMsg>(jetauto_arm_topic, ArmCallBack);
    }

    private void FixedUpdate()
    {
        if (unityControl)
        {
            GetInput();
        }
        HandleMotor();
    }

    public void RemoteControl(TwistMsg msg)
    {
        if (unityControl)
        {
            return;
        }
        inputs[0] = Mathf.Min(Mathf.Max((float)-msg.angular.z, -1.0f), 1.0f);
        inputs[1] = Mathf.Min(Mathf.Max((float)msg.linear.x, -1.0f), 1.0f);

        Debug.Log("angular: " + inputs[0] + ", torque:" + inputs[1]);

        float latentTime = (inputs[0] != 0) ? 1.286f : 2.0f;
        StartCoroutine(DelaySecond(latentTime));

        // StartCoroutine(WaitForSteeringAngle(45.0f));
    }

    private IEnumerator WaitForSteeringAngle(float targetAngle)
    {
        HandleMotor();
        float yAngle = Mathf.Abs(transform.rotation.eulerAngles.y);

        while (!isNear(yAngle, 90f) && !isNear(yAngle, 270f))
        {
            // Debug.Log("Steering angle is close to " + transform.position);
            Debug.Log("Steering angle is close to " + transform.rotation.eulerAngles);

            yield return null; // 다음 프레임으로 넘어감
        }

        // // 현재 오브젝트의 로테이션 값 가져오기
        // Vector3 currentRotation = transform.rotation.eulerAngles;

        // // Y축 회전 값을 90으로 설정
        // float newRotationY = 90f;

        // // 새로운 로테이션 벡터 생성
        // Vector3 newRotation = new Vector3(currentRotation.x, newRotationY, currentRotation.z);

        // // 오브젝트의 로테이션 값 변경
        // transform.rotation = Quaternion.Euler(newRotation);

        Debug.Log("Steering angle is close to 90 degrees!");
        // 필요한 작업 수행
        // inputs[0] = 1f;
        inputs[1] = 0.1f;
    }
    private bool isNear(float cord, float target, float thresold = 3.5f)
    {
        return Mathf.Abs(cord - target) < thresold;
    }
    private IEnumerator DelaySecond(float delaySeconds = 1.0f)
    {
        HandleMotor();
        yield return new WaitForSeconds(delaySeconds);
        // 현재 오브젝트의 로테이션 값 가져오기
        // Vector3 currentRotation = transform.rotation.eulerAngles;

        // Y축 회전 값을 90으로 설정
        // float newRotationY = (currentRotation.y < 0.0f && currentRotation.z > 0) ? 90f : -90f;

        // // 새로운 로테이션 벡터 생성
        // Vector3 newRotation = new Vector3(currentRotation.x, newRotationY, currentRotation.z);

        // // 오브젝트의 로테이션 값 변경
        // transform.rotation = Quaternion.Euler(newRotation);
        inputs[0] = 0.0f;
        inputs[1] = 0.0f;
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

    private bool ValidateAction(float angle, float dir, float min, float max)
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
        if (ValidateAction(arm_link1.localEulerAngles.y - 90, inputs[2], -170, 170))
        {
            arm_link1.Rotate(new Vector3(0, inputs[2], 0));
        }
        if (ValidateAction(arm_link2.localEulerAngles.z, inputs[3], -90, 90))
        {
            arm_link2.Rotate(new Vector3(0, 0, inputs[3]));
        }
        if (ValidateAction(arm_link3.localEulerAngles.z, inputs[4], -90, 90))
        {
            arm_link3.Rotate(new Vector3(0, 0, inputs[4]));
        }
        if (ValidateAction(arm_link4.localEulerAngles.z, inputs[5], -90, 90))
        {
            arm_link4.Rotate(new Vector3(0, 0, inputs[5]));
        }
        if (ValidateAction(l_in_link.localEulerAngles.x, inputs[6], 0, 90))
        {
            l_in_link.Rotate(new Vector3(inputs[6], 0, 0));
            l_link.Rotate(new Vector3(inputs[6], 0, 0));
            l_out_link.Rotate(new Vector3(-inputs[6], 0, 0));

        }

        if (ValidateAction(-r_in_link.localEulerAngles.x, inputs[6], 0, 90))
        {
            r_in_link.Rotate(new Vector3(-inputs[6], 0, 0));
            r_link.Rotate(new Vector3(-inputs[6], 0, 0));
            r_out_link.Rotate(new Vector3(inputs[6], 0, 0));
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

    private void CarCallBack(TwistMsg msg)
    {
        if (unityControl)
        {
            return;
        }
        inputs[0] = Mathf.Min(Mathf.Max((float)-msg.angular.z, -1.8f), 1.8f);
        inputs[1] = Mathf.Min(Mathf.Max((float)msg.linear.x, -3.0f), 3.0f);

        Debug.Log("angular.z: " + inputs[0] + ", torque.x:" + inputs[1]);

    }

    private void ArmCallBack(JointStateMsg msg)
    {
        if (unityControl)
        {
            return;
        }
        inputs[2] = Mathf.Min(Mathf.Max((float)-msg.velocity[0], -1.0f), 1.0f);
        inputs[3] = Mathf.Min(Mathf.Max((float)-msg.velocity[1], -1.0f), 1.0f);
        inputs[4] = Mathf.Min(Mathf.Max((float)-msg.velocity[2], -1.0f), 1.0f);
        inputs[5] = Mathf.Min(Mathf.Max((float)-msg.velocity[3], -1.0f), 1.0f);
        inputs[6] = Mathf.Min(Mathf.Max((float)msg.velocity[4], -1.0f), 1.0f);
    }

}
