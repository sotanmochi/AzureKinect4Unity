using UnityEngine;

namespace AzureKinect4Unity.Examples
{
    public class OrientationEstimationPresenter : MonoBehaviour
    {
        [SerializeField] DeviceManagerView _View;
        [SerializeField] AzureKinectManager _DeviceManager;
        [SerializeField] AzureKinectOrientationEstimation _OrientationEstimator;

        void Awake()
        {
            _View.OnClickedOpenDevice.AddListener(() => 
            {
                int deviceIndex = _View.CurrentDeviceIndex;
                var kinectSensor = _DeviceManager.OpenDevice(deviceIndex);
                _OrientationEstimator.Initialize(kinectSensor);
            });
        }
    }
}