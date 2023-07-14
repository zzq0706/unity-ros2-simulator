using System;
using System.Linq;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

public class Ultrasonic : MonoBehaviour
{
    public string topic;
    public double PublishPeriodSeconds = 0.1;
    public float RangeMetersMin = 0;
    public float RangeMetersMax = 50;
    public float HorizontalScanAngleStartDegrees = -70;
    public float HorizontalScanAngleEndDegrees = 70;
    public float VerticalScanAngleStartDegrees = -35;
    public float VerticalScanAngleEndDegrees = 35;
    public int NumMeasurementsPerScan = 10;
    public float TimeBetweenMeasurementsSeconds = 0.01f;
    public string LayerMaskName = "TurtleBot3Manual";
    public string FrameId = "ultrasonic";

    // Data for the PointCloud2 message
    private List<byte[]> pointData = new List<byte[]>();

    // ros
    ROSConnection m_Ros;
    
    public struct PointXYZ
	{
    public float x;
    public float y;
    public float z;
	}


    protected virtual void Start()
    {
        // Initialize ROS
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<PointCloud2Msg>(topic);

    }

    public void Update()
    {

        // Perform the scan
        for (int i = 0; i < NumMeasurementsPerScan; i++)
        {
            for (int j = 0; j < NumMeasurementsPerScan; j++)
            {
                var horizontalAngle = Mathf.Lerp(HorizontalScanAngleStartDegrees, HorizontalScanAngleEndDegrees, i / (float)NumMeasurementsPerScan) * Mathf.Deg2Rad;
                var verticalAngle = Mathf.Lerp(VerticalScanAngleStartDegrees, VerticalScanAngleEndDegrees, j / (float)NumMeasurementsPerScan) * Mathf.Deg2Rad;

                var directionVector = new Vector3(
                    Mathf.Cos(verticalAngle) * Mathf.Sin(horizontalAngle),
                    Mathf.Sin(verticalAngle),
                    Mathf.Cos(verticalAngle) * Mathf.Cos(horizontalAngle)
                );

                var measurementStart = RangeMetersMin * directionVector + transform.position;
                var measurementRay = new Ray(measurementStart, directionVector);
                LayerMask mask = LayerMask.GetMask(LayerMaskName);
                var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax, ~mask);

                if (foundValidMeasurement)
                {
                    var point = new PointXYZ { x = hit.point.z - transform.position.z, y = -hit.point.x + transform.position.x, z = hit.point.y - transform.position.y };

                    // Convert the point to a byte array and add it to the pointData list
                    var pointBytes = new byte[12];
                    Buffer.BlockCopy(BitConverter.GetBytes(point.x), 0, pointBytes, 0, 4);
                    Buffer.BlockCopy(BitConverter.GetBytes(point.y), 0, pointBytes, 4, 4);
                    Buffer.BlockCopy(BitConverter.GetBytes(point.z), 0, pointBytes, 8, 4);
                    pointData.Add(pointBytes);
                }
            }
        }

        // Create the PointCloud2 message
        var fields = new PointFieldMsg[]
        {
            new PointFieldMsg { name = "x", offset = 0, datatype = PointFieldMsg.FLOAT32, count = 1 },
            new PointFieldMsg { name = "y", offset = 4, datatype = PointFieldMsg.FLOAT32, count = 1 },
            new PointFieldMsg { name = "z", offset = 8, datatype = PointFieldMsg.FLOAT32, count = 1 }
        };
        
        var timestamp = new TimeStamp(Clock.time);

        var header = new HeaderMsg
        {
            frame_id = FrameId,
            stamp = new TimeMsg
            {
                sec = timestamp.Seconds,
                nanosec = timestamp.NanoSeconds,
            }
        };

        var msg = new PointCloud2Msg
        {
            header = header,
            height = 1,  // Set to 1 for an unordered point cloud
            width = (uint)pointData.Count,
            fields = fields,
            is_bigendian = false,
            point_step = 12,  // Size of a point in bytes
            row_step = (uint)(pointData.Count * 12),  // Size of a row in bytes
            data = pointData.SelectMany(point => point).ToArray(),  // Convert the list of byte arrays into a single byte array
            is_dense = true
        };

        m_Ros.Publish(topic, msg);

        // Clear the pointData list for the next scan
        pointData.Clear();

    }


}

