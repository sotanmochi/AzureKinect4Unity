// Copyright (c) 2020 Soichiro Sugimoto.
// Licensed under the MIT License. See LICENSE in the project root for license information.
//
// 3D coordinate systems for Gyroscope and accelerometer
// https://docs.microsoft.com/ja-jp/azure/Kinect-dk/coordinate-systems
//

using System;

namespace AzureKinect4Unity
{
    public static class OrientationEstimator
    {
        public static UnityEngine.Vector3 EstimateFromAccelerometerForUnity(System.Numerics.Vector3 accelerometer)
        {
            UnityEngine.Vector3 estimatedAngles = UnityEngine.Vector3.zero;

            System.Numerics.Vector3 orientation = EstimateFromAccelerometer(accelerometer);
            estimatedAngles.z = orientation.X + 180.0f; // Roll
            estimatedAngles.x = orientation.Y; // Pitch
            estimatedAngles.y = orientation.Z; // Yaw

            return estimatedAngles;
        }

        public static System.Numerics.Vector3 EstimateFromAccelerometer(System.Numerics.Vector3 accelerometer)
        {
            System.Numerics.Vector3 estimatedAngles = System.Numerics.Vector3.Zero;

            float ayz = (float) Math.Sqrt(accelerometer.Y * accelerometer.Y + accelerometer.Z * accelerometer.Z);

            float roll = (float) Math.Atan2(accelerometer.Y, accelerometer.Z);
            float pitch = (float) Math.Atan2(accelerometer.X, ayz);

            estimatedAngles.X = (float)(roll * 180.0 / Math.PI);
            estimatedAngles.Y = (float)(pitch * 180.0 / Math.PI);

            return estimatedAngles;
        }
    }
}
