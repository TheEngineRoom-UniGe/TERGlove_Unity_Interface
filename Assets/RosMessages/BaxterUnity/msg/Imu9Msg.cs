//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.BaxterUnity
{
    [Serializable]
    public class Imu9Msg : Message
    {
        public const string k_RosMessageName = "baxter_unity/Imu9";
        public override string RosMessageName => k_RosMessageName;

        //  This is a message to hold data from an IMU (Inertial Measurement Unit)
        // 
        //  Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
        // 
        //  If the covariance of the measurement is known, it should be filled in (if all you know is the 
        //  variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
        //  A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
        //  data a covariance will have to be assumed or gotten from some other source
        // 
        //  If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
        //  estimate), please set element 0 of the associated covariance matrix to -1
        //  If you are interpreting this message, please check for a value of -1 in the first element of each 
        //  covariance matrix, and disregard the associated estimate.
        public HeaderMsg header;
        public Geometry.PoseMsg pose;
        // geometry_msgs/Quaternion orientation
        public double[] orientation_covariance;
        //  Row major about x, y, z axes
        public Geometry.Vector3Msg angular_velocity;
        public double[] angular_velocity_covariance;
        //  Row major about x, y, z axes
        public Geometry.Vector3Msg linear_acceleration;
        public double[] linear_acceleration_covariance;
        //  Row major x, y z 
        public Geometry.Vector3Msg magnetometer;
        public double[] magnetormeter_covariance;
        //  Row major x, y z 

        public Imu9Msg()
        {
            this.header = new HeaderMsg();
            this.pose = new Geometry.PoseMsg();
            this.orientation_covariance = new double[9];
            this.angular_velocity = new Geometry.Vector3Msg();
            this.angular_velocity_covariance = new double[9];
            this.linear_acceleration = new Geometry.Vector3Msg();
            this.linear_acceleration_covariance = new double[9];
            this.magnetometer = new Geometry.Vector3Msg();
            this.magnetormeter_covariance = new double[9];
        }

        public Imu9Msg(HeaderMsg header, Geometry.PoseMsg pose, double[] orientation_covariance, Geometry.Vector3Msg angular_velocity, double[] angular_velocity_covariance, Geometry.Vector3Msg linear_acceleration, double[] linear_acceleration_covariance, Geometry.Vector3Msg magnetometer, double[] magnetormeter_covariance)
        {
            this.header = header;
            this.pose = pose;
            this.orientation_covariance = orientation_covariance;
            this.angular_velocity = angular_velocity;
            this.angular_velocity_covariance = angular_velocity_covariance;
            this.linear_acceleration = linear_acceleration;
            this.linear_acceleration_covariance = linear_acceleration_covariance;
            this.magnetometer = magnetometer;
            this.magnetormeter_covariance = magnetormeter_covariance;
        }

        public static Imu9Msg Deserialize(MessageDeserializer deserializer) => new Imu9Msg(deserializer);

        private Imu9Msg(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.pose = Geometry.PoseMsg.Deserialize(deserializer);
            deserializer.Read(out this.orientation_covariance, sizeof(double), 9);
            this.angular_velocity = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out this.angular_velocity_covariance, sizeof(double), 9);
            this.linear_acceleration = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out this.linear_acceleration_covariance, sizeof(double), 9);
            this.magnetometer = Geometry.Vector3Msg.Deserialize(deserializer);
            deserializer.Read(out this.magnetormeter_covariance, sizeof(double), 9);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.pose);
            serializer.Write(this.orientation_covariance);
            serializer.Write(this.angular_velocity);
            serializer.Write(this.angular_velocity_covariance);
            serializer.Write(this.linear_acceleration);
            serializer.Write(this.linear_acceleration_covariance);
            serializer.Write(this.magnetometer);
            serializer.Write(this.magnetormeter_covariance);
        }

        public override string ToString()
        {
            return "Imu9Msg: " +
            "\nheader: " + header.ToString() +
            "\npose: " + pose.ToString() +
            "\norientation_covariance: " + System.String.Join(", ", orientation_covariance.ToList()) +
            "\nangular_velocity: " + angular_velocity.ToString() +
            "\nangular_velocity_covariance: " + System.String.Join(", ", angular_velocity_covariance.ToList()) +
            "\nlinear_acceleration: " + linear_acceleration.ToString() +
            "\nlinear_acceleration_covariance: " + System.String.Join(", ", linear_acceleration_covariance.ToList()) +
            "\nmagnetometer: " + magnetometer.ToString() +
            "\nmagnetormeter_covariance: " + System.String.Join(", ", magnetormeter_covariance.ToList());
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
