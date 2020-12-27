using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

namespace AzureKinect4Unity.Examples
{
    public class DeviceManagerView : MonoBehaviour
    {
        [SerializeField] Dropdown _Dropdown_Device;
        [SerializeField] Button _Button_OpenDevice;

        public UnityEvent OnClickedOpenDevice;

        public int CurrentDeviceIndex => _CurrentDeviceIndex;
        private int _CurrentDeviceIndex = 0;

        public void UpdateDeviceNameList(IReadOnlyList<string> deviceNameList)
        {
            _Dropdown_Device.options.Clear();

            foreach (string deviceName in deviceNameList)
            {
                _Dropdown_Device.options.Add(new Dropdown.OptionData(deviceName));
            }
        }

        void Awake()
        {
            _Dropdown_Device.onValueChanged.AddListener(index =>
            {
                _CurrentDeviceIndex = index;
            });

            _Button_OpenDevice.onClick.AddListener(() =>
            {
                OnClickedOpenDevice?.Invoke();
            });
        }

        void OnDestroy()
        {
            _Dropdown_Device.onValueChanged.RemoveAllListeners();
            _Button_OpenDevice.onClick.RemoveAllListeners();
        }
    }
}
