using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

namespace Test.Sensor
{
  [RequireComponent(typeof(Camera))]
  public class DepthCamera : MonoBehaviour
  {
    [SerializeField] private int _width  = 320;
    [SerializeField] private int _height = 240;
    [SerializeField] private float _scanRate = 10f;
    public float fieldOfView = 90.0f;
    public float farClipPlane = 65.535f;
    protected bool send_image = false;
    public int pixels;
    
    const int type = 3;	
	public Shader shader;
	
	private Material _material;
	
	
	private Material material {
	get {
		if (_material == null) {
			_material = new Material(shader);
			_material.hideFlags = HideFlags.HideAndDontSave;
		}
		return _material;
		}
	}
	
	private void OnDisable() {
	if (_material != null)
		DestroyImmediate(_material);
	}
    
    public uint width  { get => (uint)this._width; }
    public uint height { get => (uint)this._height; }
    public float scanRate { get => this._scanRate; }
    
    private Camera _camera;
    private Texture2D _texture;
    // private Rect _rect;
    RenderTexture cameraImage;
    
    // [HideInInspector] public byte[] data;
    
    // params for ros
    private float _timeElapsed = 0f;
    private float _timeStamp   = 0f;
    private ROSConnection _ros;
    private ImageMsg _message;  
    [SerializeField] private string _topicName = "depth_image";
    [SerializeField] private string _frameId   = "depth_camera";

    public void Init()
    {
      pixels = this._width * this._height;
      
      cameraImage = new RenderTexture(this._width, this._height, 24);
      this._camera  = GetComponent<Camera>();
      
      this._camera.farClipPlane = farClipPlane;
      this._camera.fieldOfView = fieldOfView;
      this._camera.depthTextureMode = DepthTextureMode.Depth;
      
      this._texture = new Texture2D(this._width, this._height, TextureFormat.RGB24, false);
      // this._rect = new Rect(0, 0, this._width, this._height);
      // this._texture.Apply();
      this._camera.targetTexture = cameraImage;

      // Camera.onPostRender += UpdateImage;
      
      // setup ROS
      this._ros = ROSConnection.instance;
      this._ros.RegisterPublisher<ImageMsg>(this._topicName);

      // setup ROS Message
      this._message = new ImageMsg();
      // this._message.header.frame_id = this._frameId;
      this._message.header.frame_id = "depth_camera_trans";
      this._message.width = (uint)this._width;
      this._message.height = (uint)this._height;
      this._message.encoding = "16UC1";
      this._message.step = (uint)this._width * 2;
      // this._message.format = "png";
    }
  
    void Awake() 
    {
      Init();
    }
    
    void OnRenderImage(RenderTexture src, RenderTexture dest) 
    {
      Graphics.Blit(src, dest, material);
      if (send_image) 
      {
          SendImage();
          send_image = false;
      }
    }
    
    void Update () 
    {
      this._timeElapsed += Time.deltaTime;
      if (this._timeElapsed > (1f/this._scanRate)) 
      {
		    send_image = true;
		    // Update time
		    this._timeElapsed = 0;
	    }
    }
    
    private byte[] FlipImage(byte[] img) 
    {
      int stride = this._width * 3;
      byte[] flippedImage = new byte[this._height * stride];
      
      for(int row = 0; row < this._height ; row++) 
      {
        Buffer.BlockCopy(img, (this._height - row - 1) * stride, flippedImage, row * stride, stride);
      }

        return flippedImage;
    }

     
    protected void SendImage() 
    {
      RenderTexture.active = cameraImage;		                                      
      this._texture.ReadPixels(new Rect (0, 0, cameraImage.width, cameraImage.height), 0, 0, false);
      byte[] image = this._texture.GetRawTextureData();
      byte[] imageBytes = FlipImage(image);
      
      // Update ROS Message
# if ROS2
      int sec = (int)Math.Truncate(this._timeStamp);
# else
         // uint sec = (uint)Math.Truncate(this._timeStamp);
# endif
      uint nanosec = (uint)( (this._timeStamp - sec)*1e+9 );
      this._message.header.stamp.sec = sec;
      this._message.header.stamp.nanosec = nanosec;
         
      byte[] data = new byte[pixels * 2];
         
      float max_range_millimeters = farClipPlane * 1000;
         
      for (int i = 0; i < pixels; i++)
      {
        byte r = imageBytes[3 * i];
        byte g = imageBytes[3 * i + 1];
        byte b = imageBytes[3 * i + 2];
        float depth = 1.0f * (float)r / 255.0f + (float)g / 255.0f * 1.0f / 255.0f + (float)b / 255.0f * 1.0f / 65025.0f;
        
        if (depth > 0.99f)
        {
          depth = 0.0f;
        }
           
        ushort int_millimeters = (ushort)(depth * max_range_millimeters);
           
           
        data[2 * i] = (byte)((int_millimeters << 8) >> 8);
        data[2 * i + 1] = (byte)(int_millimeters >> 8);
      }
      
      this._message.data = data;
         
      this._ros.Send(this._topicName, this._message);
      this._timeStamp = Time.time;
    }
  
  }
}
