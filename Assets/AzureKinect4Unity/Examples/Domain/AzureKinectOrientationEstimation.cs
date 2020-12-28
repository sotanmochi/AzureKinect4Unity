using UnityEngine;
using Microsoft.Azure.Kinect.Sensor;

namespace AzureKinect4Unity
{
    [RequireComponent(typeof(PointCloudRenderer))]
    public class AzureKinectOrientationEstimation : MonoBehaviour
    {
        [SerializeField] bool LowPassFilter = true;

        [SerializeField] uint _Order = 3;
        [SerializeField] float _CutoffFrequency = 1; // [Hz]
        [SerializeField] float _SamplingFrequency = 60; // [Hz]

        AzureKinectSensor _KinectSensor;
        ILowPassFilter _LowPassFilter;
        float [] _Accel = new float[3];
        float [] _AccelOut = new float[3];

        PointCloudRenderer _PointCloudRenderer;
        byte[] _DepthRawData;

        public bool Initialized => _Initialized;
        private bool _Initialized;

        public void Initialize(AzureKinectSensor kinectSensor)
        {
            if (!_Initialized)
            {
                _KinectSensor = kinectSensor;
                if (_KinectSensor != null)
                {
                    Debug.Log("ColorResolution: " + _KinectSensor.ColorImageWidth + "x" + _KinectSensor.ColorImageHeight);
                    Debug.Log("DepthResolution: " + _KinectSensor.DepthImageWidth + "x" + _KinectSensor.DepthImageHeight);

                    // _LowPassFilter = new ExponentialSmoothingLowPassFilter((uint)_Accel.Length, 0.05f);
                    // _LowPassFilter = new DoubleExponentialSmoothingLowPassFilter((uint)_Accel.Length, 0.3f, 0.3f);
                    _LowPassFilter = new ButterworthFilter(_Order, _SamplingFrequency, _CutoffFrequency, (uint)_Accel.Length);

                    _DepthRawData = new byte[_KinectSensor.DepthImageWidth * _KinectSensor.DepthImageHeight * sizeof(ushort)];
                    _PointCloudRenderer = GetComponent<PointCloudRenderer>();

                    CameraCalibration deviceDepthCameraCalibration = _KinectSensor.DeviceCalibration.DepthCameraCalibration;
                    CameraCalibration deviceColorCameraCalibration = _KinectSensor.DeviceCalibration.ColorCameraCalibration;

                    K4A.Calibration calibration = new K4A.Calibration();
                    calibration.DepthCameraCalibration = CreateCalibrationCamera(deviceDepthCameraCalibration, _KinectSensor.DepthImageWidth, _KinectSensor.DepthImageHeight);
                    calibration.ColorCameraCalibration = CreateCalibrationCamera(deviceColorCameraCalibration, _KinectSensor.ColorImageWidth, _KinectSensor.ColorImageHeight);

                    _PointCloudRenderer.GenerateMesh(calibration, K4A.CalibrationType.Depth);

                    _Initialized = true;
                }
            }
        }

        private K4A.CalibrationCamera CreateCalibrationCamera(CameraCalibration cameraCalibration, int width, int height)
        {
            K4A.CalibrationCamera calibrationCamera = new K4A.CalibrationCamera();

            float[] intrinsicsParameters = cameraCalibration.Intrinsics.Parameters;
            Extrinsics extrinsics = cameraCalibration.Extrinsics;

            calibrationCamera.resolutionWidth = width;
            calibrationCamera.resolutionHeight = height;
            calibrationCamera.metricRadius = cameraCalibration.MetricRadius;

            calibrationCamera.intrinsics.cx = intrinsicsParameters[0];
            calibrationCamera.intrinsics.cy = intrinsicsParameters[1];
            calibrationCamera.intrinsics.fx = intrinsicsParameters[2];
            calibrationCamera.intrinsics.fy = intrinsicsParameters[3];
            calibrationCamera.intrinsics.k1 = intrinsicsParameters[4];
            calibrationCamera.intrinsics.k2 = intrinsicsParameters[5];
            calibrationCamera.intrinsics.k3 = intrinsicsParameters[6];
            calibrationCamera.intrinsics.k4 = intrinsicsParameters[7];
            calibrationCamera.intrinsics.k5 = intrinsicsParameters[8];
            calibrationCamera.intrinsics.k6 = intrinsicsParameters[9];
            calibrationCamera.intrinsics.codx = intrinsicsParameters[10];
            calibrationCamera.intrinsics.cody = intrinsicsParameters[11];
            calibrationCamera.intrinsics.p2 = intrinsicsParameters[12]; // p2: tangential distortion coefficient y
            calibrationCamera.intrinsics.p1 = intrinsicsParameters[13]; // p1: tangential distortion coefficient x
            calibrationCamera.intrinsics.metricRadius = intrinsicsParameters[14];

            calibrationCamera.extrinsics.rotation[0][0] = extrinsics.Rotation[0];
            calibrationCamera.extrinsics.rotation[0][1] = extrinsics.Rotation[1];
            calibrationCamera.extrinsics.rotation[0][2] = extrinsics.Rotation[2];
            calibrationCamera.extrinsics.rotation[1][0] = extrinsics.Rotation[3];
            calibrationCamera.extrinsics.rotation[1][1] = extrinsics.Rotation[4];
            calibrationCamera.extrinsics.rotation[1][2] = extrinsics.Rotation[5];
            calibrationCamera.extrinsics.rotation[2][0] = extrinsics.Rotation[6];
            calibrationCamera.extrinsics.rotation[2][1] = extrinsics.Rotation[7];
            calibrationCamera.extrinsics.rotation[2][2] = extrinsics.Rotation[8];
            calibrationCamera.extrinsics.translation[0] = extrinsics.Translation[0];
            calibrationCamera.extrinsics.translation[1] = extrinsics.Translation[1];
            calibrationCamera.extrinsics.translation[2] = extrinsics.Translation[2];

            return calibrationCamera;
        }

        void Update()
        {
            if (_KinectSensor != null)
            {
                if (_KinectSensor.RawDepthImage != null)
                {
                    short[] depthImage = _KinectSensor.RawDepthImage;
                    System.Buffer.BlockCopy(depthImage, 0, _DepthRawData, 0, _DepthRawData.Length);

                    // _PointCloudRenderer.UpdateColorTexture(_KinectSensor.TransformedColorImage);
                    _PointCloudRenderer.UpdateDepthTexture(_DepthRawData);
                }

                System.Numerics.Vector3 accel = _KinectSensor.ImuSample.AccelerometerSample;
                if (LowPassFilter)
                {
                    _Accel[0] = accel.X;
                    _Accel[1] = accel.Y;
                    _Accel[2] = accel.Z;
                    _LowPassFilter.Apply(_Accel, ref _AccelOut);
                    accel.X = _AccelOut[0];
                    accel.Y = _AccelOut[1];
                    accel.Z = _AccelOut[2];
                }

                UnityEngine.Vector3 kinectOrientation = OrientationEstimator.EstimateFromAccelerometerForUnity(accel);

                this.transform.eulerAngles = kinectOrientation;
            }
        }
    }
}
