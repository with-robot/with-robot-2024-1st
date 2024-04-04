using UnityEngine;

public class CameraParameters : MonoBehaviour
{
    void Start()
    {
        Camera cam = GetComponent<Camera>();
        float verticalFOV = cam.fieldOfView;
        float horizontalFOV = 2 * Mathf.Atan(Mathf.Tan(verticalFOV * Mathf.Deg2Rad / 2) * cam.aspect) * Mathf.Rad2Deg;
        
        float imageWidth = Screen.width;
        float imageHeight = Screen.height;
        
        // 초점 거리 계산
        float focalLength = imageWidth / (2 * Mathf.Tan(horizontalFOV * Mathf.Deg2Rad / 2));
        
        // 주점 좌표 계산
        float cx = imageWidth / 2;
        float cy = imageHeight / 2;
        
        Debug.Log($"Focal Length: {focalLength}, {verticalFOV}, {horizontalFOV}");
        Debug.Log($"Principal Point: ({imageWidth} {imageHeight} {cx}, {cy})");
    }
}