using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
    
public struct UDP_packet
{
    public float[] quaternion;
    public short[] accelerometer;
    public short[] gyroscope;
    public short[] magnetometer;
    public char id;
}

public class TERGlove_driver : MonoBehaviour
{
    private UdpClient udpClient;
    private IPEndPoint endPoint;
    private VariableManager vManager;

    // Start is called before the first frame update
    void Start()
    {
        vManager = GetComponent<VariableManager>();
        udpClient = new UdpClient(vManager.port_number);
        endPoint = new IPEndPoint(IPAddress.Any, 0);
        udpClient.
        Debug.Log(udpClient.);
    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log(vManager.port_number);
        if (udpClient.Available > 0)
        {
            Debug.Log("hello");
            UDP_packet new_msg = ReadNewMsg(udpClient, endPoint);
            // Process received data here
        }
    }

    void OnDestroy()
    {
        udpClient.Close();
    }

    UDP_packet ReadNewMsg(UdpClient UdpC, IPEndPoint endP) 
    {
        UDP_packet packet = new UDP_packet();

        byte[] data = UdpC.Receive(ref endP);
        if (data.Length != 35)
        {
            throw new System.Exception("Packet size is different from the expected one!");
        }

        packet.quaternion = new float[4];
        System.Buffer.BlockCopy(data, 0, packet.quaternion, 0, sizeof(float) * 4);

        for (int i = 0; i < 3; i++)
        {
            short[] t_buff = new short[3];
            System.Buffer.BlockCopy(data, sizeof(float) * 4 + sizeof(short) * 3 * i, t_buff, 0, sizeof(short) * 3);

            switch (i)
            {
                case 0:
                    // Accelerometer Data
                    packet.accelerometer = t_buff;
                    break;
                case 1:
                    // Gyroscope Data
                    packet.gyroscope = t_buff;
                    break;
                case 2:
                    // Magnetometer Data
                    packet.magnetometer = t_buff;
                    break;
            }
        }

        packet.id = System.BitConverter.ToChar(data, sizeof(float) * 4 + sizeof(short) * 9);
        Debug.Log(packet);
        return packet;
    }
}
