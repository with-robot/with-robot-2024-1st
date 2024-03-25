using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using UnityEngine;
using System.Threading;
using UnityEditor.ProjectWindowCallback;

public class MoveTest : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject robot;
    CarController controller;

    TwistMsg msg;
    void Start()
    {
        msg = new TwistMsg();
        controller = robot.GetComponent<CarController>();
    }

    // Update is called once per frame
    void Update()
    {
        // 지정된 각도가 될때까지 진행
        msg.angular.z = -3.14 / 4;
        msg.linear.x = 1.5f;
        Move();

        // controller.RemoteControl(msg);

    }

    // 시작위치와 목적위치를 받아 이동을 한다.
    private void Move()
    {
        // 직진2단계

        // 우회전
        // msg.linear.x = 0f;
        // 좌회전

        controller.RemoteControl(msg);

        // Thread.Sleep(1000);

        // msg.angular.z = 0f;
        // msg.linear.x = 1.0f;
        // controller.RemoteControl(msg);

        // Thread.Sleep(1000);

        // msg.angular.z = -3.14 / 4f;
        // msg.linear.x = 2.0f;
    }

}
