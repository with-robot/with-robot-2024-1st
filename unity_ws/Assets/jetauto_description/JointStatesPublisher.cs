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
        };
        joint_states.position = new double[] { 0.0, 0.0, 0.0, 0.0 };
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
            // publish ros topic
            joint_states.header.stamp = stamp;
            ros.Publish(topicName, joint_states);
        }
    }
}
