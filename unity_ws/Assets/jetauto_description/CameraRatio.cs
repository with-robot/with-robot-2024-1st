using UnityEngine;

[RequireComponent(typeof(Camera))]
public class CameraRatio : MonoBehaviour
{
    public float verticalFOV = 63f;
    public float horizontalFOV = 94.9f;

    void Start()
    {
        AdjustFOV();
    }

    void AdjustFOV()
    {
        Camera cam = GetComponent<Camera>();

        // 수직 FOV 설정
        cam.fieldOfView = verticalFOV;

        // 수평 FOV를 바탕으로 종횡비 계산
        float horizontalFOVRadians = horizontalFOV * Mathf.Deg2Rad;
        float verticalFOVRadians = verticalFOV * Mathf.Deg2Rad;
        float aspect = Mathf.Tan(horizontalFOVRadians / 2) / Mathf.Tan(verticalFOVRadians / 2);

        // 카메라의 종횡비 설정
        cam.aspect = aspect;
    }
}