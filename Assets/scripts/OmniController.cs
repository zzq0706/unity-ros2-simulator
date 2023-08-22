using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;
using System;

namespace RosSharp.Control
{
    // public enum ControlMode { Keyboard, ROS};

    public class OmniController : MonoBehaviour
    {
        public GameObject wheel1;
        public GameObject wheel2;
        public GameObject wheel3;
        public GameObject wheel4;
        // public ControlMode mode = ControlMode.ROS;

        private ArticulationBody wA1;
        private ArticulationBody wA2;
        private ArticulationBody wA3;
        private ArticulationBody wA4;

        public float maxLinearSpeed = 2; //  m/s
        public float maxRotationalSpeed = 1;//
        public float wheelRadius = 0.14f; //meters
        public float R = 0.14f;
        public float trackWidth = 0.72f; // meters Distance between tyres
        public float trackLength = 1.1f;
        public float forceLimit = 10;
        public float damping = 10;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;
        

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinearX = 0f;
        private float rosLinearY = 0f;
        private float rosAngular = 0f;
        
        // init of transformation matrix
        double[,] T_wh_B = new double[4, 3];

        void Start()
        {
            wA1 = wheel1.GetComponent<ArticulationBody>();
            wA2 = wheel2.GetComponent<ArticulationBody>();
            wA3 = wheel3.GetComponent<ArticulationBody>();
            wA4 = wheel4.GetComponent<ArticulationBody>();
            SetParameters(wA1);
            SetParameters(wA2);
            SetParameters(wA3);
            SetParameters(wA4);
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
            
		    // postions of wheels
		    double[,] p = new double[,]
			{
				{ trackLength / 2 , -trackLength / 2, trackLength / 2, -trackLength / 2 },
				{ trackWidth / 2, trackWidth/ 2, -trackWidth / 2, -trackWidth / 2}
			};
			
			// transformation matrix
			double[] g = new double[] { 0, 0, 180, 180 };
			
			for (int i = 0; i < g.Length; i++)
			{
				g[i] *= Math.PI / 180;
			}
			
			double[] a = new double[] { 45, -45, -45, 45 };
			
			for (int i = 0; i < a.Length; i++)
			{
				a[i] *= Math.PI / 180;
			}
		
            
		    for (int i = 0; i < 4; i++)
			{
				double cotA = 1 / Math.Tan(a[i]);
				double cosG = Math.Cos(g[i]);
				double sinG = Math.Sin(g[i]);
				double p2i = p[1, i];
				double p1i = p[0, i];

				T_wh_B[i, 0] = 1 / R * (1 * cosG + cotA * sinG);
				T_wh_B[i, 1] = 1 / R * (1 * sinG - cotA * cosG);
				T_wh_B[i, 2] = 1 / R * (-p2i * cosG + p1i * sinG - p2i * sinG * cotA - p1i * cosG * cotA);
			}

        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinearX = (float)cmdVel.linear.x;
            rosLinearY = (float)cmdVel.linear.y;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
        	ROSUpdate();
        	
            // if (mode == ControlMode.Keyboard)
            // {
                // KeyBoardUpdate();
            // }
            // else if (mode == ControlMode.ROS)
            // {
                // ROSUpdate();
            // }     
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }


        private void ROSUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinearX = 0f;
                rosLinearY = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinearX, rosLinearY, -rosAngular);
        }

        private void RobotInput(float speedX, float speedY, float rotSpeed) // m/s and rad/s
        {
            if (speedX > maxLinearSpeed)
            {
                speedX = maxLinearSpeed;
            }
            if (speedY > maxLinearSpeed)
            {
                speedY = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            
			double[] q_dot = new double[] {speedX, speedY, -rotSpeed};
			double[] omega_wh = new double[4];

			for (int i = 0; i < 4; i++)
			{
				omega_wh[i] = T_wh_B[i, 0] * q_dot[0] + T_wh_B[i, 1] * q_dot[1] + T_wh_B[i, 2] * q_dot[2];
			}
			
			float wheel1Rotation = (float)omega_wh[0] * Mathf.Rad2Deg;
			float wheel2Rotation = (float)omega_wh[1] * Mathf.Rad2Deg;
			float wheel3Rotation = (float)omega_wh[2] * Mathf.Rad2Deg;
			float wheel4Rotation = (float)omega_wh[3] * Mathf.Rad2Deg;
			
			Debug.Log("v1: " + wheel1Rotation);
			Debug.Log("v2: " + wheel2Rotation);
			Debug.Log("v3: " + wheel3Rotation);
			Debug.Log("v4: " + wheel4Rotation);
			
            // float wheel1Rotation = (speed / wheelRadius);
            // float wheel2Rotation = wheel1Rotation;
            // float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            // if (rotSpeed != 0)
            // {
                // wheel1Rotation = (wheel1Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                // wheel2Rotation = (wheel2Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            // }
            // else
            // {
                // wheel1Rotation *= Mathf.Rad2Deg;
                // wheel2Rotation *= Mathf.Rad2Deg;
            // }
            SetSpeed(wA1, wheel1Rotation);
            SetSpeed(wA2, wheel2Rotation);
            SetSpeed(wA3, -wheel3Rotation);
            SetSpeed(wA4, -wheel4Rotation);
        }
    }
}
