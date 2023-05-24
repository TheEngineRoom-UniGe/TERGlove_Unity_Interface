using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float dragSpeed = 4;
    public float wheelSpeed = 10;
    public GameObject childCamera;

    void Update()
    {
        //if (Input.GetMouseButtonDown(0))
        //{
        //    return;
        //}

        if (Input.GetMouseButton(0))
        {
            transform.Rotate(new Vector3(-Input.GetAxis("Mouse Y") * dragSpeed, Input.GetAxis("Mouse X") * dragSpeed, 0), Space.Self);
        }

        //Vector3 pos = Camera.main.ScreenToViewportPoint(Input.mousePosition - dragOrigin);
        //Vector3 move = new Vector3(pos.x * dragSpeed, 0, pos.y * dragSpeed);


        Vector3 normalizedDirection = (this.gameObject.transform.position - this.childCamera.transform.position).normalized;
        float wheelAmount = Input.GetAxis("Mouse ScrollWheel");
        //Debug.Log(wheelAmount);
        this.childCamera.transform.position += normalizedDirection * wheelAmount * wheelSpeed * Time.deltaTime;
    }



}