using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

using Unity.Robotics.ROSTCPConnector;
using Float64 = RosMessageTypes.Std.Float64Msg;

namespace Ocs.Vehicle.Controller
{
    public class DumperController : MonoBehaviour
    {
        [SerializeField] private Crawler _vehicle;
        private Ocs.Input.VehicleInput _input;

        [Header("- Topic Name -")]
        [SerializeField] private string leftCrawler_topic = "dumper/leftCrawler";
        [SerializeField] private string rightCrawler_topic = "dumper/rightCrawler";
        private float leftCrawler_input, rightCrawler_input;

#if UNITY_EDITOR
        [Header("- Debug -")]
        [SerializeField] private bool _debug_forceManualMode = false;
#endif

        private void Awake()
        {
            this._input = new Ocs.Input.VehicleInput();
        }

        private void OnEnable() => this._input.Enable();
        private void OnDestroy() => this._input.Dispose();

        private void OnDisable()
        {
            this._input.Disable();
        }

        private void Start()
        {
            // Callback
            this._input.Crawler.LeftReverse.started += context =>{
                if(_vehicle.ownership){
                    this._vehicle.LeftReverse = !this._vehicle.LeftReverse;
                }
            };
            this._input.Crawler.RightReverse.started += context =>{
                if(_vehicle.ownership){
                    this._vehicle.RightReverse = !this._vehicle.RightReverse;
                }
            };

            //ros
            ROSConnection.GetOrCreateInstance().Subscribe<Float64>(this.leftCrawler_topic, leftCrawler_callback);
            ROSConnection.GetOrCreateInstance().Subscribe<Float64>(this.rightCrawler_topic, rightCrawler_callback);

        }

        void Update()
        {
#if UNITY_EDITOR
            if (_debug_forceManualMode)
            {
                this._vehicle.LeftCrawlerInput = this._input.Crawler.LeftForward.ReadValue<float>();
                this._vehicle.RightCrawlerInput = this._input.Crawler.RightForward.ReadValue<float>();
                return;
            }
#endif
            if (this._vehicle.automation){ //ros
                //left crawler
                    if(leftCrawler_input >= 0){
                        this._vehicle.LeftReverse = true;
                        this._vehicle.LeftCrawlerInput = leftCrawler_input;
                    }else if(leftCrawler_input < 0){
                        this._vehicle.LeftReverse = false;
                        this._vehicle.LeftCrawlerInput = System.Math.Abs(leftCrawler_input);
                    }
                    //right crawler
                    if(rightCrawler_input >= 0){
                        this._vehicle.RightReverse = true;
                        this._vehicle.RightCrawlerInput = rightCrawler_input;
                    }else if(rightCrawler_input < 0){
                        this._vehicle.RightReverse = false;
                        this._vehicle.RightCrawlerInput = System.Math.Abs(rightCrawler_input);
                    }

            }else{ //controller
                if(this._vehicle.ownership){
                    this._vehicle.LeftCrawlerInput = this._input.Crawler.LeftForward.ReadValue<float>();
                    this._vehicle.RightCrawlerInput = this._input.Crawler.RightForward.ReadValue<float>();
                }
            }        
        }

        void leftCrawler_callback(Float64 message)
        {
            leftCrawler_input = (float)message.data;
        }

        void rightCrawler_callback(Float64 message)
        {
            rightCrawler_input = (float)message.data;
        }
    }
}
