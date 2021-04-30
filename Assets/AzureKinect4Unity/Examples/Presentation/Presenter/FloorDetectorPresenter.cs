using UnityEngine;

namespace AzureKinect4Unity.Examples
{
    public class FloorDetectorPresenter : MonoBehaviour
    {
        [SerializeField] DeviceManagerView _View;
        [SerializeField] AzureKinectManager _DeviceManager;
        [SerializeField] AzureKinectFloorDetector _FloorDetector;

        void Awake()
        {
            _View.OnClickedOpenDevice.AddListener(() => 
            {
                int deviceIndex = _View.CurrentDeviceIndex;
                var kinectSensor = _DeviceManager.OpenDevice(deviceIndex);
                _FloorDetector.Initialize(kinectSensor);
            });
        }
    }
}