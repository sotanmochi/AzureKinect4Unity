// Copyright (c) 2020 Soichiro Sugimoto.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;
using AzureKinectToolkit;

namespace AzureKinect4Unity
{
    public static class OrientationEstimator
    {
        public static UnityEngine.Vector3 EstimateForAzureKinect(System.Numerics.Vector3 k4aImuAccSample)
        {
            UnityEngine.Vector3 estimatedAngles = UnityEngine.Vector3.zero;

            System.Numerics.Vector3 orientation = SensorOrientationEstimator.EstimateFromAccelerometerForAzureKinect(k4aImuAccSample);
            estimatedAngles.z = orientation.X; // Roll
            estimatedAngles.x = orientation.Y; // Pitch
            estimatedAngles.y = orientation.Z; // Yaw

            return estimatedAngles;
        }
    }
}
