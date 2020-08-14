using System.Collections.Generic;
using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;

namespace AzureKinect4Unity
{
    [RequireComponent(typeof(PointCloudMesh))]
    public class AzureKinectOrientationEstimation : MonoBehaviour
    {
        [SerializeField] AzureKinectManager _AzureKinectManager;
        [SerializeField] int _DeviceNumber = 0;

        AzureKinectSensor _KinectSensor;
        ExponentialSmoothingLowPassFilter _LowPassFilter;
        float [] _Accel = new float[3];
        float [] _AccelOut = new float[3];

        PointCloudMesh _PointCloudMesh;
        Texture2D _ColorImageTexture;

        void Start()
        {
            List<AzureKinectSensor> kinectSensors = _AzureKinectManager.SensorList;
            if (_DeviceNumber < kinectSensors.Count)
            {
                _KinectSensor = _AzureKinectManager.SensorList[_DeviceNumber];
                if (_KinectSensor != null)
                {
                    Debug.Log("ColorResolution: " + _KinectSensor.ColorImageWidth + "x" + _KinectSensor.ColorImageHeight);
                    Debug.Log("DepthResolution: " + _KinectSensor.DepthImageWidth + "x" + _KinectSensor.DepthImageHeight);

                    _LowPassFilter = new ExponentialSmoothingLowPassFilter((uint)_Accel.Length, 0.05f);

                    _ColorImageTexture = new Texture2D(_KinectSensor.ColorImageWidth, _KinectSensor.ColorImageHeight, TextureFormat.BGRA32, false);

                    _PointCloudMesh = GetComponent<PointCloudMesh>();
                    _PointCloudMesh.GenerateMesh(_KinectSensor.ColorImageWidth, _KinectSensor.ColorImageHeight);
                }
            }
        }

        void Update()
        {
            _KinectSensor = _AzureKinectManager.SensorList[_DeviceNumber];
            if (_KinectSensor != null)
            {
                if (_KinectSensor.RawColorImage != null)
                {
                    _ColorImageTexture.LoadRawTextureData(_KinectSensor.RawColorImage);
                    _ColorImageTexture.Apply();
                }

                if (_KinectSensor.PointCloud != null)
                {
                    Short3[] pointCloud = _KinectSensor.PointCloud;
                    
                    Vector3[] vertices = new Vector3[pointCloud.Length];
                    for (int i = 0; i < vertices.Length; i++)
                    {
                        vertices[i] = new Vector3(pointCloud[i].X * 0.001f, pointCloud[i].Y * -0.001f, pointCloud[i].Z * 0.001f);
                    }

                    _PointCloudMesh.UpdateVertices(vertices);
                    _PointCloudMesh.UpdateColorTexture(_KinectSensor.RawColorImage);
                }

                System.Numerics.Vector3 accel = _KinectSensor.ImuSample.AccelerometerSample;
                _Accel[0] = accel.X;
                _Accel[1] = accel.Y;
                _Accel[2] = accel.Z;
                _LowPassFilter.Apply(_Accel, ref _AccelOut);
                accel.X = _AccelOut[0];
                accel.Y = _AccelOut[1];
                accel.Z = _AccelOut[2];

                UnityEngine.Vector3 kinectOrientation = OrientationEstimator.EstimateFromAccelerometerForUnity(accel);

                this.transform.eulerAngles = kinectOrientation;
            }
        }
    }
}
