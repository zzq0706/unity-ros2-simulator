using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

namespace Test.Sensor
{
	[RequireComponent(typeof(Camera))]
	public class RGBCamera : MonoBehaviour {
		RenderTexture cameraImage;
		public RenderTexture outputImage;
		Texture2D myTexture2D;
		public Camera camera;
		
		public int width = 320;
		public int height = 240;
		public float fieldOfView = 90.0f;
		public float farClipPlane = 65.535f;
		public float targetFrameRate = 10.0f;
		float time_last_image_sent = 0.0f;
		protected bool send_image = false;
		
		protected RenderTextureReadWrite render_texture_read_write = RenderTextureReadWrite.Default;
		
		// ros
		private float _timeStamp   = 0f;
		private ROSConnection _ros;
		private ImageMsg _message;  
		[SerializeField] private string _topicName = "rgb_image";
		[SerializeField] private string _frameId   = "rgb_camera";

		void Initialize() {
		    cameraImage = new RenderTexture(width, height, 24, RenderTextureFormat.DefaultHDR, render_texture_read_write);
		    outputImage = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, render_texture_read_write);

			if(camera == null) {
				camera = GetComponent<Camera>();
			}
			camera.farClipPlane = farClipPlane;
			camera.depthTextureMode = DepthTextureMode.Depth;
			camera.targetTexture = cameraImage;

			myTexture2D = new Texture2D (cameraImage.width, cameraImage.height, TextureFormat.RGB24, false);
			
			
			// setup ROS
			this._ros = ROSConnection.instance;
			this._ros.RegisterPublisher<ImageMsg>(this._topicName);

			// setup ROS Message
			this._message = new ImageMsg();
			// this._message.header.frame_id = this._frameId;
			this._message.header.frame_id = "rgb_camera_trans";
			this._message.width = (uint)this.width;
			this._message.height = (uint)this.height;
			this._message.encoding = "rgb8";
			this._message.step = (uint)this.width * 3;
		}
		
	    private byte[] FlipImage(byte[] img) 
		{
		  int stride = this.width * 3;
		  byte[] flippedImage = new byte[this.height * stride];
		  
		  for(int row = 0; row < this.height ; row++) 
		  {
			Buffer.BlockCopy(img, (this.height - row - 1) * stride, flippedImage, row * stride, stride);
		  }

			return flippedImage;
		}

		void OnPreRender() {
			camera.farClipPlane = farClipPlane;
			camera.fieldOfView = fieldOfView;
		}	

		void OnRenderImage(RenderTexture src, RenderTexture dest) {
			Graphics.Blit(src, outputImage);
			if (send_image) {
				SendImage(outputImage);
				send_image = false;
			}
		}

		protected void SendImage(RenderTexture tex) {
			RenderTexture.active = tex;		                                      
			myTexture2D.ReadPixels (new Rect (0, 0, tex.width, tex.height), 0, 0, false);
			byte[] imageBytes = myTexture2D.GetRawTextureData ();
			
			byte[] imageFlip = FlipImage(imageBytes);
			
			// Update ROS Message
# if ROS2
		  	int sec = (int)Math.Truncate(this._timeStamp);
# else
		    // uint sec = (uint)Math.Truncate(this._timeStamp);
# endif
		  	uint nanosec = (uint)( (this._timeStamp - sec)*1e+9 );
		  	this._message.header.stamp.sec = sec;
		  	this._message.header.stamp.nanosec = nanosec;
		  	
		  	this._message.data = imageFlip;
		     
		  	this._ros.Send(this._topicName, this._message);
		  	this._timeStamp = Time.time;

		}

		void Awake () {
			Initialize();
		}

		void Update () {
			float time_since_last_image_sent = Time.time - time_last_image_sent;
			if (time_since_last_image_sent > (1/targetFrameRate - Time.fixedDeltaTime * 0.5f)) {
				send_image = true;
				time_last_image_sent = Time.time;
			}
		}	
	}
}
