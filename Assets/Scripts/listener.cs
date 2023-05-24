using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using Newtonsoft.Json;
using System.Threading;
using System;

[System.Serializable]
public struct Orientation_Data
{
    public int sensor_id;
    public float[] orientation;
}


public class Listener : MonoBehaviour
{
    public GameObject cube;
    public int port_number = 2400;
    public GameObject[] joint_list;

    private bool _firstFlag = true;
    private Quaternion _firstQuat;
    private static UdpClient udpClient = null;
    private IPEndPoint endPoint;
    private Thread connectThread;
    private bool isRunning = false;
    private Orientation_Data _orientationData;
    private static readonly object dataMutex = new object();
    private bool _newData = false;

    void Start()
    {
        Connect(IPAddress.Any, () =>
        {
            Debug.Log("Any code written here will execute after the connection has been established");
            ReceiveData();
        });

    }


    public void Connect(System.Net.IPAddress ip, System.Action callback)
    {
        this.connectThread = new Thread(() => Connect_Thread(ip, callback));
        this.connectThread.Start();
    }


    private void Connect_Thread(System.Net.IPAddress ip, System.Action callback)
    {
        this.isRunning = true;
        if (Listener.udpClient == null)
        {
            Listener.udpClient = new UdpClient(this.port_number);
        }

        this.endPoint = new IPEndPoint(ip, 0);

        Debug.Log("Opening a UDP socket on port: " + this.port_number);
        callback?.Invoke();
    }


    void ReceiveData()
    {
        Debug.Log("ReceiveData: " + this.isRunning);
        while (this.isRunning)
        {
            if (Listener.udpClient != null)
            {
                if (Listener.udpClient.Available > 0)
                {
                    byte[] packetData = Listener.udpClient.Receive(ref this.endPoint);
                    Orientation_Data od = this.DeserializeMsg(packetData);
                    lock (Listener.dataMutex)
                    {
                        this._orientationData = od;
                        this._newData = true;
                    }
                    
                    //this.ApplyRotation(od.orientation, od.sensor_id);
                }
            }
        }
        Listener.udpClient.Close();
    }


    void Update()
    {
        Orientation_Data od_temp;//= new Orientation_Data();
        od_temp.orientation = new float[] { 0, 0, 0, 0 };
        od_temp.sensor_id = -1;

        lock (Listener.dataMutex)
        {
            //Debug.Log(this._orientationData);
            try { 
                if (this._newData) {
                    od_temp.orientation = this._orientationData.orientation;
                    od_temp.sensor_id = this._orientationData.sensor_id;
                }

            }
            catch (NullReferenceException ex)
            {
                Debug.Log(ex.Message);
            }
            finally
            {
                this._newData = false;
            }
        }
        if (od_temp.sensor_id != -1)
        {

            this.ApplyRotation(od_temp.orientation, od_temp.sensor_id);
        }
    }


    Orientation_Data DeserializeMsg(byte[] data)
    {

        //Debug.Log(data);
        byte[] binaryJsonData = data;
        string jsonText = System.Text.Encoding.UTF8.GetString(data);
        //Debug.Log(jsonText);
        Orientation_Data imuData = JsonConvert.DeserializeObject<Orientation_Data>(jsonText);
        //Orientation_Data imuData = JsonUtility.FromJson<Orientation_Data>(jsonText);
        //Debug.Log("Received IMU Data - Sensor ID: " + imuData.sensor_id);
        //Debug.Log("Received IMU Data - Orientation - x: " +
        //    imuData.orientation[0] + ", y: " +
        //    imuData.orientation[1] + ", z: " +
        //    imuData.orientation[2] + ", w: " +
        //    imuData.orientation[3]);

        return imuData;
    }   

    void ApplyRotation(float[] orientation, int idx)
    {   
        if (_firstFlag){
            _firstQuat = new Quaternion(orientation[1],
                                        -orientation[2],
                                        -orientation[0],
                                        orientation[3]);

            _firstFlag = false;
        }

        
        Quaternion newQuat = new Quaternion(orientation[1],
                                            -orientation[2],
                                            -orientation[0],
                                            orientation[3]);


        //if (imuMessage.header.frame_id == this.cube.name) this.cube.transform.rotation = newQuat;
        //if (this.cube.tag == "parent") this.cube.transform.rotation = newQuat;
        //Debug.Log(idx);
        this.joint_list[idx].transform.rotation = newQuat;
    }

    private void OnDestroy()
    {
        // Join the receive thread to wait for its completion
        Debug.Log("Closing thread");
        this.isRunning = false;
    }
}
