using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using RosMessageTypes.Sensor;
using Unity.MLAgents.Sensors;

public class LidarPublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "jetauto/scan";
    [SerializeField] private float publishFrequency = 0.1f;
    [SerializeField] private GameObject lidar_link_sensor;
    private RayPerceptionSensorComponent3D raySensor;

    ROSConnection ros;
    private float timeElapsed = 0.0f;

    private LaserScanMsg scan;

    // Start is called before the first frame update
    void Start()
    {
        raySensor = lidar_link_sensor.GetComponent<RayPerceptionSensorComponent3D>();

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);

        scan = new LaserScanMsg();
        scan.header.frame_id = "base_footprint";
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
            // Ray Sensor
            RayPerceptionOutput.RayOutput[] rayOutputs = RayPerceptionSensor.Perceive(raySensor.GetRayPerceptionInput()).RayOutputs;
            if (rayOutputs == null)
            {
                return;
            }
            int length = rayOutputs.Length;
            int center = length / 2;
            // Scan Message
            scan.header.stamp = stamp;
            scan.angle_min = -1.57f;
            scan.angle_max = 1.57f;
            scan.angle_increment = 3.14f / length;
            scan.time_increment = (1 / 10) / length;
            scan.range_min = 0.0f;
            scan.range_max = 100.0f;
            scan.ranges = new float[length];
            scan.intensities = new float[length];

            for (int i = 0; i < length; ++i)
            {
                var rayDirection = rayOutputs[i].EndPositionWorld - rayOutputs[i].StartPositionWorld;
                float range = rayOutputs[i].HitFraction * rayDirection.magnitude;
                float intensity = 1.0f;

                int index = center;
                if (i == 0)
                {
                    index = center;
                }
                else if (i % 2 == 1)
                {
                    index = center - 1 - i / 2;
                }
                else
                {
                    index = center + i / 2;
                }
                scan.ranges[index] = range;
                scan.intensities[index] = intensity;
            }
            ros.Publish(topicName, scan);
        }
    }
}
