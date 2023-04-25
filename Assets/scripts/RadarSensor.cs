using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
// using Simulator.Bridge;
using Simulator.Bridge.Data;
// using Simulator.Utilities;
using UnityEngine;
using UnityEngine.AI;
// using Simulator.Sensors.UI;
using Unity.Robotics.ROSTCPConnector;
using DetectedRadarObjectMsg = RosMessageTypes.Dumper.DetectedRadarObjectMsg;
using DetectedRadarObjectArrayMsg = RosMessageTypes.Dumper.DetectedRadarObjectArrayMsg;
using RosMessageTypes.Geometry;



namespace Simulator.Sensors
{
    public class RadarSensor : MonoBehaviour
    {
        [Range(1.0f, 100f)]
        public float Frequency = 10f; //13.4f
        public LayerMask RadarBlockers;

        private List<RadarMesh> radars = new List<RadarMesh>();


        private uint seqId = 0;
        private float nextPublish;
        
        private Dictionary<Collider, DetectedRadarObject> Detected = new Dictionary<Collider, DetectedRadarObject>();
        
        // ros thing
        private ROSConnection _ros;
    	private DetectedRadarObjectArrayMsg _message;
    	[SerializeField] private string _topicName = "radar_scan";
    	[SerializeField] private string _frameId   = "Radars";
    	
    	private float _timeStamp   = 0f;

        
        struct Box
        {
            public Vector3 Size;
            public Color Color;
        }

        private void Awake()
        {
            radars.AddRange(GetComponentsInChildren<RadarMesh>());
            foreach (var radar in radars)
            {
                radar.Init();
            }
        }

        private void Start()
        {
        	// init ros thing
            this._ros = ROSConnection.instance;
        	this._ros.RegisterPublisher<DetectedRadarObjectArrayMsg>(this._topicName);

       
        	
            foreach (var radar in radars)
            {
                radar.SetCallbacks(WhileInRange, OnExitRange);
            }
            nextPublish = Time.time + 1.0f / Frequency;
        }

        private void Update()
        {

            if (Time.time < nextPublish)
            {
                return;
            }

            nextPublish = Time.time + 1.0f / Frequency;
            
			// publish ros data
			if (Detected.Count == 0)
			{
				Debug.Log("nothing detected");
			}
			Debug.Log("velocity of the test object is: " + Detected.First().Value.RelativeVelocity);
			this._message = ROS2ConvertFrom(Detected.Values.ToArray());
			this._ros.Send(this._topicName, this._message);
			
			// update time
			this._timeStamp = Time.time;
        }

        void WhileInRange(Collider other, RadarMesh radar)
        {

            // if (other.isTrigger)
                // return;

            if (!other.enabled)
                return;
            
            if (CheckBlocked(other))
            {
                if (Detected.ContainsKey(other))
                    Detected.Remove(other);
                return;
            }

            if (Detected.ContainsKey(other)) // update existing data
            {
                Detected[other].SensorPosition = transform.position;
                Detected[other].SensorAim = transform.forward;
                Detected[other].SensorRight = transform.right;
                Detected[other].SensorVelocity = GetSensorVelocity();
                Detected[other].SensorAngle = GetSensorAngle(other);
                Detected[other].Position = other.bounds.center;
                Detected[other].Velocity = GetObjectVelocity(other);
                Detected[other].RelativePosition = other.bounds.center - transform.position;
                Detected[other].RelativeVelocity = GetSensorVelocity() - GetObjectVelocity(other);
                Detected[other].ColliderSize = other.bounds.size;
                Detected[other].State = GetAgentState(other);
                Detected[other].NewDetection = false;
            }
            else
            {
                Box box = GetVisualizationBox(other);
                if (box.Size != Vector3.zero) // Empty box returned if tag is not right
                {
                    // Visualized.Add(other, box);
                    Detected.Add(other, new DetectedRadarObject()
                    {
                        Id = other.gameObject.GetInstanceID(),
                        SensorPosition = transform.position,
                        SensorAim = transform.forward,
                        SensorRight = transform.right,
                        SensorVelocity = GetSensorVelocity(),
                        SensorAngle = GetSensorAngle(other),
                        Position = other.bounds.center,
                        Velocity = GetObjectVelocity(other),
                        RelativePosition = other.bounds.center - transform.position,
                        RelativeVelocity = GetSensorVelocity() - GetObjectVelocity(other),
                        ColliderSize = other.bounds.size,
                        State = GetAgentState(other),
                        NewDetection = true,
                });
                }
            }
        }

        void OnExitRange(Collider other, RadarMesh radar)
        {
            OnExitRange(other);
        }

        void OnExitRange(Collider other)
        {
            if (Detected.ContainsKey(other))
                Detected.Remove(other);
        }

        private bool CheckBlocked(Collider col)
        {
            bool isBlocked = false;
            var orig = transform.position;
            var end = col.bounds.center;
            var dir = end - orig;
            var dist = (end - orig).magnitude;
            if (Physics.Raycast(orig, dir, out RaycastHit hit, dist, RadarBlockers)) // ignore if blocked
            {
                if (hit.collider != col)
                    isBlocked = true;
            }
            return isBlocked;
        }

        private Box GetVisualizationBox(Collider other)
        {
            var bbox = new Box();
            Vector3 size = Vector3.zero;

            if (other.gameObject.layer == LayerMask.NameToLayer("Obstacle"))
                bbox.Color = Color.green;
            else
            {
                return bbox;
            }

            if (other is BoxCollider)
            {
                var box = other as BoxCollider;
                bbox.Size = box.size;
                size.x = box.size.z;
                size.y = box.size.x;
                size.z = box.size.y;
            }
            else if (other is CapsuleCollider)
            {
                var capsule = other as CapsuleCollider;
                bbox.Size = new Vector3(capsule.radius * 2, capsule.height, capsule.radius * 2);
                size.x = capsule.radius * 2;
                size.y = capsule.radius * 2;
                size.z = capsule.height;
            }

            return bbox;
        }

        private Vector3 GetSensorVelocity()
        {
            return gameObject.GetComponentInParent<Rigidbody>() == null ? Vector3.zero : gameObject.GetComponentInParent<Rigidbody>().velocity;
        }

        private double GetSensorAngle(Collider col)
        {
            // angle is orientation of the obstacle in degrees as seen by radar, counterclockwise is positive
            double angle = -Vector3.SignedAngle(transform.forward, col.transform.forward, transform.up);
            if (angle > 90)
                angle -= 180;
            else if (angle < -90)
                angle += 180;
            return angle;
        }

        private Vector3 GetObjectVelocity(Collider col)
        {
            Vector3 velocity = Vector3.zero;
            var ped = col.GetComponent<NavMeshAgent>();
            if (ped != null)
                velocity = ped.desiredVelocity;
            var rb = col.attachedRigidbody;
            if (rb != null)
                velocity = rb.velocity;
            return velocity;
        }

        private int GetAgentState(Collider col)
        {
            int state = 1;
            return state;
        }
        
        public void OnVisualize()
        {

            foreach (var radar in radars)
            {
            	Graphics.DrawMesh(radar.GetComponent<MeshFilter>().sharedMesh, transform.localToWorldMatrix, radar.RadarMeshRenderer.sharedMaterial, LayerMask.NameToLayer("Sensor"));
            }
        }
        
        // ros msgs converter
        public Vector3Msg ConvertToRosVector3(UnityEngine.Vector3 v)
        {
            return new Vector3Msg() { x = v.z, y = -v.x, z = v.y };
        }

        public PointMsg ConvertToRosPoint(UnityEngine.Vector3 v)
        {
            return new PointMsg() { x = v.z, y = -v.x, z = v.y };
        }
        
        public DetectedRadarObjectArrayMsg ROS2ConvertFrom(DetectedRadarObject[] data)
        {
            var r = new DetectedRadarObjectArrayMsg();
  			r.header.frame_id = this._frameId;
# if ROS2
            int sec = (int)Math.Truncate(this._timeStamp);
# else
            // uint sec = (uint)Math.Truncate(this._timeStamp);
# endif
            uint nanosec = (uint)( (this._timeStamp - sec)*1e+9 );
            r.header.stamp.sec = sec;
            r.header.stamp.nanosec = nanosec;

			List<DetectedRadarObjectMsg> detectedObjects = new List<DetectedRadarObjectMsg>();
            foreach (var obj in data)
            {
                detectedObjects.Add(new DetectedRadarObjectMsg()
                {
                    sensor_aim = ConvertToRosVector3(obj.SensorAim),
                    sensor_right = ConvertToRosVector3(obj.SensorRight),
                    sensor_position = ConvertToRosPoint(obj.SensorPosition),
                    sensor_velocity = ConvertToRosVector3(obj.SensorVelocity),
                    sensor_angle = obj.SensorAngle,
                    object_position = ConvertToRosPoint(obj.Position),
                    object_velocity = ConvertToRosVector3(obj.Velocity),
                    object_relative_position = ConvertToRosPoint(obj.RelativePosition),
                    object_relative_velocity = ConvertToRosVector3(obj.RelativeVelocity),
                    object_collider_size = ConvertToRosVector3(obj.ColliderSize),
                    object_state = (byte)obj.State,
                    new_detection = obj.NewDetection,
                });
            }
            r.objects = detectedObjects.ToArray();

            return r;
        }

    }
}



