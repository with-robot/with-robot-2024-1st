using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;

public class JointStatesPublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "joint_states";
    [SerializeField] private float publishFrequency = 0.1f;

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

    ROSConnection ros;
    private float timeElapsed = 0.0f;

    private JointStateMsg joint_states;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(topicName);

        joint_states = new JointStateMsg();
        joint_states.header.frame_id = "joint_states";
        joint_states.name = new string[] {
                "wheel_left_front_joint",
                "wheel_right_front_joint",
                "wheel_left_back_joint",
                "wheel_right_back_joint",
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "r_joint"
        };
        joint_states.position = new double[] {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        };
    }

    private void FixedUpdate()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            timeElapsed = 0;
            // now timestamp
            var now = Clock.Now;
            var stamp = new TimeMsg
            {
                sec = (int)now,
                nanosec = (uint)((now - Math.Floor(now)) * Clock.k_NanoSecondsInSeconds)
            };
            // joint states
            joint_states.position[0] = viewFL.eulerAngles.x * Mathf.Deg2Rad;
            joint_states.position[1] = viewFR.eulerAngles.x * Mathf.Deg2Rad;
            joint_states.position[2] = viewBL.eulerAngles.x * Mathf.Deg2Rad;
            joint_states.position[3] = viewBR.eulerAngles.x * Mathf.Deg2Rad;
            joint_states.position[4] = (arm_link1.localEulerAngles.y - 90) * Mathf.Deg2Rad;
            joint_states.position[5] = arm_link2.localEulerAngles.z * Mathf.Deg2Rad;
            joint_states.position[6] = arm_link3.localEulerAngles.z * Mathf.Deg2Rad;
            joint_states.position[7] = arm_link4.localEulerAngles.z * Mathf.Deg2Rad;
            joint_states.position[8] = l_in_link.localEulerAngles.x * Mathf.Deg2Rad;
            // publish ros topic
            joint_states.header.stamp = stamp;
            ros.Publish(topicName, joint_states);
        }
    }
}
