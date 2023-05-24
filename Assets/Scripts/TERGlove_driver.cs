using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Text;
using Debug = UnityEngine.Debug;
using UnityEngine.TextCore.Text;

public struct TER_packet
{
    public float[] quaternion;
    public short[] accelerometer;
    public short[] gyroscope;
    public short[] magnetometer;
    public short id;
}

public class TERGlove_driver : MonoBehaviour
{
    private UdpClient udpClient;
    private IPEndPoint endPoint;
    private VariableManager variableManager;
    
    public bool print_msgs = false;
    //public delegate void OnMsgArrival(TER_packet pck);
    //public static event OnMsgArrival onMsgArrival;

    // Start is called before the first frame update
    void Start()
    {
        this.variableManager = this.gameObject.GetComponent<VariableManager>();
        this.udpClient = new UdpClient(variableManager.port_number);
        this.endPoint = new IPEndPoint(IPAddress.Any, 0);

        Debug.Log("Opening a UDP socket on port: " + this.variableManager.port_number);
    }

    void Update()
    {
        if (udpClient.Available > 0)
        {
            byte[] packetData = udpClient.Receive(ref this.endPoint);
            this.DeserializeMsg(packetData);
            // Process the received packet here...
        }
    }


    TER_packet DeserializeMsg(byte[] data)
    {
        TER_packet packet = new TER_packet();
        if (data.Length != 35)
        {
            throw new System.Exception("Packet size is different from the expected one!");
        }

        // Quaternion Data
        packet.quaternion = new float[4];
        System.Buffer.BlockCopy(data, 0, packet.quaternion, 0, sizeof(float) * 4);

        // Iterate over the three sensors
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
        packet.id = (short)data[34];

        if (print_msgs)
        {
            this.PrintTerPacket(packet);    
        }
        return packet;
    }

    void PrintTerPacket(TER_packet pack)
    {
        string packet_string = "ID: " + pack.id + "\n" +
        "Quaternion: " + string.Join(", ", pack.quaternion) + "\n" +
        "Accelerometer: " + string.Join(", ", pack.accelerometer) + "\n" +
        "Gyroscope: " + string.Join(", ", pack.gyroscope) + "\n" +
        "Magnetometer: " + string.Join(", ", pack.magnetometer);

        Debug.Log(packet_string);
    }
}
