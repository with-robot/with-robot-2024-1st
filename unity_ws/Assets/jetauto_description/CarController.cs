using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;

public class CarController : MonoBehaviour
{

    private const string HORIZONTAL = "Horizontal";
    private const string VERTICAL = "Vertical";


    private float[] inputs = new float[7];
    private float verticalInput;

    [SerializeField] private bool unityControl = true;

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
        if (ValidaeAction(arm_link1.localEulerAngles.y - 90, inputs[2], -170, 170))
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
            l_link.Rotate(new Vector3(inputs[6], 0, 0));
            l_out_link.Rotate(new Vector3(-inputs[6], 0, 0));

        }

        if (ValidaeAction(-r_in_link.localEulerAngles.x, inputs[6], 0, 90))
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
        inputs[0] = Mathf.Min(Mathf.Max((float)-msg.angular.z, -1.0f), 1.0f);
        inputs[1] = Mathf.Min(Mathf.Max((float)msg.linear.x, -1.0f), 1.0f);
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
