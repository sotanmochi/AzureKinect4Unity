// Copyright (c) Soichiro Sugimoto.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;

namespace AzureKinect4Unity
{
    public class AzureKinectManager : MonoBehaviour
    {
        public ImageFormat ColorImageFormat = ImageFormat.ColorBGRA32;
        public ColorCameraMode ColorCameraMode = ColorCameraMode._1920_x_1080_30fps;
        public DepthCameraMode DepthCameraMode = DepthCameraMode._512_x_512_30fps;

        public List<AzureKinectSensor> SensorList => _AzureKinectSensorList;
        private List<AzureKinectSensor> _AzureKinectSensorList = new List<AzureKinectSensor>();

        public List<string> DeviceSerialNumList => _DeviceSerialNumList;
        private List<string> _DeviceSerialNumList = new List<string>();

        private List<CancellationTokenSource> _CancellationTokenSourceList = new List<CancellationTokenSource>();

        public bool Initialized => _Initialized;
        private bool _Initialized;

        private int _MainThreadID;

        void Awake()
        {
            Initialize();
        }

        void OnDestroy()
        {
            foreach (var tokenSource in _CancellationTokenSourceList)
            {
                tokenSource.Cancel();
            }

            foreach (var kinectSensor in _AzureKinectSensorList)
            {
                kinectSensor.CloseSensor();
            }
        }

        void OnApplicationQuit()
        {
            OnDestroy();
        }

        public void Initialize()
        {
            if (!_Initialized)
            {
                // _MainThreadID = Thread.CurrentThread.ManagedThreadId;

                int deviceCount = AzureKinectSensor.GetDeviceCount();
                for (int i = 0; i < deviceCount; i++)
                {
                    var kinectSensor = new AzureKinectSensor(ColorImageFormat, ColorCameraMode, DepthCameraMode);
                    if (kinectSensor.OpenSensor(i))
                    {
                        _AzureKinectSensorList.Add(kinectSensor);
                        _DeviceSerialNumList.Add(kinectSensor.Device.SerialNum);
                        _CancellationTokenSourceList.Add(new CancellationTokenSource());

                        kinectSensor.CloseSensor();
                    }
                }

                _Initialized = true;
            }
        }

        public AzureKinectSensor OpenDevice(int deviceIndex)
        {
            var kinectSensor = _AzureKinectSensorList[deviceIndex];

            kinectSensor.OpenSensor(deviceIndex);
            RunAnotherThread(_CancellationTokenSourceList[deviceIndex].Token, kinectSensor);

            return kinectSensor;
        }

        public void CloseDevice(int deviceIndex)
        {
            _CancellationTokenSourceList[deviceIndex].Cancel();
            _AzureKinectSensorList[deviceIndex].CloseSensor();
        }

        private void RunAnotherThread(CancellationToken cancellationToken, AzureKinectSensor kinectSensor)
        {
            Task.Run(() =>
            {
                // Multithread
                // Debug.Log("********************");
                // Debug.Log(" MainThreadID: " + _MainThreadID);
                // Debug.Log(" AnotherThreadID: " + Thread.CurrentThread.ManagedThreadId);
                // Debug.Log(" KinectSerialNum: " + kinectSensor.Device.SerialNum);
                // Debug.Log("********************");

                while(true)
                {
                    cancellationToken.ThrowIfCancellationRequested();
                    kinectSensor.ProcessCameraFrame();
                }
            });
        }
    }
}
