using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Security.Cryptography.X509Certificates;
using Unity.MLAgents.Integrations.Match3;

public class BallControl : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";


    private float[] inputs = new float[7];
    private float verticalInput;

    [SerializeField] private bool unityControl = true;

    [SerializeField] private float motorForce = 1f;
    [SerializeField] private float maxAngle = 1f;

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
    CoordinateManager coordinateManager;
    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(jetauto_car_topic, CarCallBack);
        coordinateManager = new CoordinateManager(transform);
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

    class CoordinateManager
    {
        private float x;
        private float y;
        private bool isXDirection;
        private float Direction;
        public (float a, float b) GetCoordinates() => (x, y);
        Transform trans;
        float r_angle;
        private bool isRotationComplate;

        public CoordinateManager(Transform a)
        {
            trans = a;
            x = 0; y = 0;
            isXDirection = true;
            Direction = 1;
            r_angle = 0f;
            isRotationComplate = false;

        }
        public void Move(float force, float rotate)
        {
            UpdateDirection(rotate);


            if (isXDirection)
            {
                x = force * Direction;
                y = 0f;
            }
            else
            {
                x = 0f;
                y = force * Direction;
            }
        }

        private void UpdateDirection(float rotation)
        {
            if (rotation == 0)
            {
                isRotationComplate = false;
                return;
            }
            if (isRotationComplate) { return; }

            float angle = -rotation * 10f;
            if (isXDirection)
            {
                Direction = (rotation > 0) ? 1f : -1f;
            }
            else
            {
                Direction = (rotation > 0) ? 1f : -1f;

            }
            // trans.rotation.eulerAngles.z = Direction;
            // trans.rotation *= Quaternion.Euler(0f, angle, 0f);
            trans.Rotate(0f, Direction * 90f, 0f, Space.Self);
            Debug.Log("rotation: " + trans.rotation);
            RotateRobot(angle);
        }
        private void RotateRobot(float angle)
        {
            // r_angle += angle;
            // if (Mathf.Abs(r_angle) >= 90f)
            // {
            //     r_angle = 0f;
            //     isRotationComplate = true;
            // }

            isRotationComplate = true;
            isXDirection = !isXDirection;

            // ToggleDirection();
        }
        public void ToggleDirection()
        {
            if (isRotationComplate)
            {
                isXDirection = !isXDirection;
            }
        }
    }
    private void HandleMotor()
    {
        float currentTurnAngle = maxAngle * inputs[0];
        float currentAccelForce = motorForce * inputs[1];

        coordinateManager.Move(currentAccelForce, currentTurnAngle);

        (float x, float y) = coordinateManager.GetCoordinates();
        // if (currentTurnAngle != 0)
        // {
        Debug.Log("mag= " + inputs[1] + ",ang=" + inputs[0]);
        // Debug.Log("x= " + x + ",y=" + y);

        // }

        // transform.Translate(new Vector3(y, 0, x) * Time.deltaTime);
        // transform.eulerAngles += new Vector3(0, 0, currentTurnAngle);
        transform.position += new Vector3(y, 0, x) * Time.deltaTime;

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
        inputs[0] = Mathf.Min(Mathf.Max((float)-msg.angular.z, -1.0f), 1.0f);
        inputs[1] = Mathf.Min(Mathf.Max((float)msg.linear.x, -3.0f), 3.0f);

        // Debug.Log("input from robot: angular.z: " + inputs[0] + ", linear.x: " + inputs[1]);
        // Debug.Log("input from robot: angular.z: " + -msg.angular.z + ", linear.x: " + msg.linear.x);

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
