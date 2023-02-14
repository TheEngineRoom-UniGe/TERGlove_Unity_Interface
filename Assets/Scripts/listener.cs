using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BaxterUnity;

public class listener : MonoBehaviour
{
    public GameObject cube;
    private bool _firstFlag = true;
    private Quaternion _firstQuat;
    // private Vector3 _gravity;
    // private Vector3 speed = new Vector3(0,0,0);

    void Start()
    {
        string topic_name = "/Pose";
        if (this.cube.tag == "child") topic_name="/Child";
        if (this.cube.tag == "parent") topic_name="/MC3";
        if (this.cube.name == "child_test" || this.cube.name == "child_test_global") topic_name="/Diff";
        Debug.Log(this.cube.name);
        Debug.Log(topic_name);

        ROSConnection.GetOrCreateInstance().Subscribe<Imu9Msg>(topic_name, PrintMsg);
        // Debug.Log("imuMessage");
    }

    void PrintMsg(Imu9Msg imuMessage)
    {   
        if (_firstFlag){
            _firstQuat = new Quaternion((float)imuMessage.pose.orientation.y,
                                        -(float)imuMessage.pose.orientation.z,
                                        -(float)imuMessage.pose.orientation.x,
                                        (float)imuMessage.pose.orientation.w);

            // _gravity = 2*new Vector3(-(float)imuMessage.linear_acceleration.y,
            //                            (float)imuMessage.linear_acceleration.z,
            //                            (float)imuMessage.linear_acceleration.x);

            _firstFlag = false;
        }

        
        Quaternion newQuat = new Quaternion((float)imuMessage.pose.orientation.y,
                                            -(float)imuMessage.pose.orientation.z,
                                            -(float)imuMessage.pose.orientation.x,
                                            (float)imuMessage.pose.orientation.w);

        // Vector3 newAcceleration = 2* new Vector3(-(float)imuMessage.linear_acceleration.y,
        //                                (float)imuMessage.linear_acceleration.z,
        //                                (float)imuMessage.linear_acceleration.x);

        // newQuat = _firstQuat * Quaternion.Inverse(newQuat);


        // if(this.cube.name == "child_test") this.cube.transform.localRotation = newQuat;
        // else this.cube.transform.rotation = newQuat;

        if (imuMessage.header.frame_id == this.cube.name) this.cube.transform.rotation = newQuat;
        if (this.cube.tag == "parent") this.cube.transform.rotation = newQuat;
        // Vector3 rotatedGravity = newQuat * _gravity ;
        // Debug.Log(newAcceleration);
        // newAcceleration -= rotatedGravity;
        // newAcceleration /=1000;

        // Debug.Log(rotatedGravity);
        // Debug.Log(newAcceleration);
        // this.cube.transform.position +=  0.5f * newAcceleration *0.3f*0.3f;
        // cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
    }
}
