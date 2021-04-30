using UnityEngine;
using AzureKinectToolkit.FloorDetection;

namespace AzureKinect4Unity.Examples
{
    public class AzureKinectFloorDetector : MonoBehaviour
    {
        [SerializeField] GameObject _KinectObject;

        [SerializeField] bool _LowPassFilterEnabled = true;
        [SerializeField] uint _Order = 3;
        [SerializeField] float _CutoffFrequency = 1; // [Hz]
        [SerializeField] float _SamplingFrequency = 60; // [Hz]

        AzureKinectSensor _KinectSensor;
        FloorDetector _FloorDetector;

        ILowPassFilter _LowPassFilter;
        float [] _KinectPos = new float[3];
        float [] _KinectPosOut = new float[3];

        public bool Initialized => _Initialized;
        private bool _Initialized;

        void Awake()
        {
            _FloorDetector = new FloorDetector();
        }

        void OnDestroy()
        {
            _FloorDetector.Dispose();
        }

        public void Initialize(AzureKinectSensor kinectSensor)
        {
            if (!_Initialized)
            {
                _KinectSensor = kinectSensor;
                if (_KinectSensor != null)
                {
                    _LowPassFilter = new ButterworthFilter(_Order, _SamplingFrequency, _CutoffFrequency, (uint)_KinectPos.Length);
                    _Initialized = true;
                }
            }
        }

        void Update()
        {
            if (_KinectSensor != null)
            {
                if (_KinectSensor.PointCloud != null && _KinectSensor.ImuSample != null)
                {
                    int downsampleStep = 2;
                    int minimumFloorPointCount = 1024 / (downsampleStep * downsampleStep);
                    if (_FloorDetector.TryDetectFloorPlane(out AzureKinectToolkit.Plane? plane, _KinectSensor.PointCloud, 
                                                            _KinectSensor.DepthImageWidth, _KinectSensor.DepthImageHeight, downsampleStep, 
                                                            _KinectSensor.ImuSample, _KinectSensor.DeviceCalibration, minimumFloorPointCount))
                    {
                        Vector3 planeOrigin = new Vector3(plane.Value.Origin.X, plane.Value.Origin.Y, plane.Value.Origin.Z);
                        Vector3 planeNormal = Vector3.Normalize(new Vector3(plane.Value.Normal.X, plane.Value.Normal.Y, plane.Value.Normal.Z));

                        // Vector3 forwardDirection = planeOrigin - Vector3.Dot(planeOrigin, planeNormal) * planeNormal;
                        // float forwardDistance = forwardDirection.magnitude;

                        float height = Vector3.Dot(planeOrigin, planeNormal);

                        // Vector3 kinectPos = new Vector3(0, -height, -forwardDistance);
                        Vector3 kinectPos = new Vector3(0, -height, 0);
                        if (_LowPassFilterEnabled)
                        {
                            _KinectPos[0] = kinectPos.x;
                            _KinectPos[1] = kinectPos.y;
                            _KinectPos[2] = kinectPos.z;
                            _LowPassFilter.Apply(_KinectPos, ref _KinectPosOut);
                            kinectPos.x = _KinectPosOut[0];
                            kinectPos.y = _KinectPosOut[1];
                            kinectPos.z = _KinectPosOut[2];
                        }

                        _KinectObject.transform.position = kinectPos;
                    }
                }
            }
        }
    }
}