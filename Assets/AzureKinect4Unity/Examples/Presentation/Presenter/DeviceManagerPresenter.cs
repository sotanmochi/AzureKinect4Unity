using UnityEngine;

namespace AzureKinect4Unity.Examples
{
    public class DeviceManagerPresenter : MonoBehaviour
    {
        [SerializeField] DeviceManagerView _View;
        [SerializeField] AzureKinectManager _DeviceManager;

        void Awake()
        {
            if(!_DeviceManager.Initialized)
            {
                _DeviceManager.Initialize();
            }

            _View.UpdateDeviceNameList(_DeviceManager.DeviceSerialNumList);

            _View.OnClickedOpenDevice.AddListener(() => 
            {
                int deviceIndex = _View.CurrentDeviceIndex;
                _DeviceManager.OpenDevice(deviceIndex);
            });
        }
    }
}