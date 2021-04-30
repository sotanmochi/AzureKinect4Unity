// Copyright (c) 2021 Soichiro Sugimoto.
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;

namespace AzureKinectToolkit
{
    public static class SensorOrientationEstimator
    {
        /// <summary>
        ///  Calculate tilted angles of a sensor for Azure Kinect.
        /// </summary>
        /// <param name="k4aImuAccSample"></param>
        /// <returns></returns>
        public static System.Numerics.Vector3 EstimateFromAccelerometerForAzureKinect(System.Numerics.Vector3 k4aImuAccSample)
        {
            //
            // Azure Kinect coordinate systems
            // https://docs.microsoft.com/ja-jp/azure/Kinect-dk/coordinate-systems
            //
            // In the accelerometer coordinate system, 
            // the exact positive direction of axis is the opposite direction figured in the document.
            //
            // The coordinate system is oriented such that
            //   the positive X-axis points forward,
            //   the positive Y-axis points right, 
            //   the positive Z-axis points upward.
            //
            // The acceleration due to gravity, g, is in a direction toward the ground.
            //
            return EstimateFromAccelerometer(-k4aImuAccSample);
        }

        /// <summary>
        /// Calculate tilted angles of a sensor in the coordinate system with the positive direction of the Z-axis toward the ground direction.
        /// </summary>
        /// <param name="accelerometer"></param>
        /// <returns></returns>
        public static System.Numerics.Vector3 EstimateFromAccelerometer(System.Numerics.Vector3 accelerometer)
        {
            System.Numerics.Vector3 estimatedAngles = System.Numerics.Vector3.Zero;

            float ayz = (float) Math.Sqrt(accelerometer.Y * accelerometer.Y + accelerometer.Z * accelerometer.Z);

            float roll  = (float)  Math.Atan2(accelerometer.Y, accelerometer.Z);
            float pitch = (float) -Math.Atan2(accelerometer.X, ayz);

            estimatedAngles.X = (float)(roll * 180.0 / Math.PI);
            estimatedAngles.Y = (float)(pitch * 180.0 / Math.PI);

            return estimatedAngles;
        }
    }
}
