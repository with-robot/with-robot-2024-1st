using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;

public class OdomPublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "unity_tf";
    [SerializeField] private float publishFrequency = 0.1f;
    [SerializeField] private Transform jetauto;

    ROSConnection ros;
    private float timeElapsed = 0.0f;

    private TwistStampedMsg tf;

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistStampedMsg>(topicName);

        tf = new TwistStampedMsg();
        tf.header.frame_id = "base_footprint";
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
            tf.header.stamp = stamp;
            tf.twist.linear.x = jetauto.position.z;
            tf.twist.linear.y = -jetauto.position.x;
            tf.twist.linear.z = jetauto.position.y;
            tf.twist.angular.x = jetauto.rotation.eulerAngles.z * Mathf.Deg2Rad;
            tf.twist.angular.y = jetauto.rotation.eulerAngles.x * Mathf.Deg2Rad;
            tf.twist.angular.z = -jetauto.rotation.eulerAngles.y * Mathf.Deg2Rad;
            ros.Publish(topicName, tf);
        }
    }
}
