//
// Licensed under the MIT License.
//
// This code is a ported and modified version of the floor plane detector from Azure Kinect DK Code Samples.
//
// The original source code is available on GitHub.
// https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/floor_detector_sample/FloorDetector.cpp
//
// -----
//
// MIT License
//
// Copyright (c) Microsoft Corporation.
// Copyright (c) 2021 Soichiro Sugimoto
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading;
using System.Threading.Tasks;
using System.Linq;
using System.Numerics;
using Microsoft.Azure.Kinect.Sensor;

namespace AzureKinectToolkit.FloorDetection
{
    public class FloorDetector : IDisposable
    {
        private ConcurrentQueue<Plane> _resultQueue = new ConcurrentQueue<Plane>();
        private Task _task;
        private CancellationTokenSource _cts;

        public void Dispose()
        {
            _cts?.Cancel();
            _cts?.Dispose();
        }

        public bool TryDetectFloorPlane(out Plane? plane, Short3[] pointCloudBuffer, int bufferWidth, int bufferHeight, int downsampleStep, 
                                        ImuSample imuSample, Calibration sensorCalibration, int minimumFloorPointCount)
        {
            if (_task == null || _task.IsCompleted)
            {
                _cts = new CancellationTokenSource();
                _task = Task.Run(async() => 
                {
                    List<Vector3> cloudPoints = GenerateCloudPoints(pointCloudBuffer, bufferWidth, bufferHeight, downsampleStep);
                    return DetectFloorPlane(cloudPoints, imuSample, sensorCalibration, minimumFloorPointCount);
                }
                , _cts.Token)
                .ContinueWith(task =>
                {
                    if (task.Result.HasValue)
                    {
                        _resultQueue.Enqueue(task.Result.Value);
                    }
                    _cts.Dispose();
                    _cts = null;
                });
            }

            if (_resultQueue.TryDequeue(out Plane detectedPlane))
            {
                plane = detectedPlane;
                return true;
            }

            plane = null;
            return false;
        }

        public List<Vector3> GenerateCloudPoints(in Short3[] pointCloudBuffer, int width, int height, int downsampleStep)
        {
            List<Vector3> cloudPoints = new List<Vector3>(pointCloudBuffer.Length / (downsampleStep * downsampleStep));

            for (int h = 0; h < height; h += downsampleStep)
            {
                for (int w = 0; w < width; w += downsampleStep)
                {
                    int pixelIndex = h * width + w;

                    // When the point cloud is invalid, the z-depth value is 0.
                    if (pointCloudBuffer[pixelIndex].Z > 0)
                    {
                        float millimeterToMeter = 0.001f;
                        float posX = pointCloudBuffer[pixelIndex].X * millimeterToMeter;
                        float posY = pointCloudBuffer[pixelIndex].Y * millimeterToMeter;
                        float posZ = pointCloudBuffer[pixelIndex].Z * millimeterToMeter;

                        cloudPoints.Add(new Vector3(posX, posY, posZ));
                    }
                }
            }

            return cloudPoints;
        }

        public Plane? DetectFloorPlane(in List<Vector3> cloudPoints, in ImuSample imuSample, in Calibration sensorCalibration, int minimumFloorPointCount)
        {
            var gravity = EstimateGravityVectorForDepthCamera(imuSample, sensorCalibration);
            if (gravity.HasValue && cloudPoints.Count > 0)
            {
                // Up normal is opposite to gravity down vector.
                Vector3 up = Vector3.Normalize(-gravity.Value);

                // Compute elevation of cloud points (projections on the floor normal).
                Vector3 point = new Vector3(0);
                float[] offsets = new float[cloudPoints.Count];
                for (int i = 0; i < cloudPoints.Count; i++)
                {
                    point.X = cloudPoints[i].X;
                    point.Y = cloudPoints[i].Y;
                    point.Z = cloudPoints[i].Z;
                    offsets[i] = Vector3.Dot(up, point);
                }

                // There could be several horizontal planes in the scene (floor, tables, ceiling).
                // For the floor, look for lowest N points whose elevations are within a small range from each other.

                float planeDisplacementRangeInMeters = 0.050f; // 5 cm in meters.
                float planeMaxTiltInDeg = 5.0f;

                int binAggregation = 6;
                float binSize = planeDisplacementRangeInMeters / binAggregation;
                HistogramBin[] histBins = Histogram(offsets, binSize);

                // Cumulative histogram counts.
                int[] cumulativeHistCounts = new int[histBins.Length];
                cumulativeHistCounts[0] = histBins[0].Count;
                for (int i = 1; i < histBins.Length; ++i)
                {
                    cumulativeHistCounts[i] = cumulativeHistCounts[i - 1] + histBins[i].Count;
                }

                for (int i = 1; i + binAggregation < cumulativeHistCounts.Length; ++i)
                {
                    int aggBinStart = i;                 // inclusive bin
                    int aggBinEnd = i + binAggregation;  // exclusive bin
                    int inlierCount = cumulativeHistCounts[aggBinEnd - 1] - cumulativeHistCounts[aggBinStart - 1];
                    if (inlierCount > minimumFloorPointCount)
                    {
                        float offsetStart = histBins[aggBinStart].LeftEdge; // inclusive
                        float offsetEnd = histBins[aggBinEnd].LeftEdge;     // exclusive

                        // Inlier indices.
                        List<int> inlierIndices = new List<int>();
                        for (int j = 0; j < offsets.Length; ++j)
                        {
                            if (offsetStart <= offsets[j] && offsets[j] < offsetEnd)
                            {
                                inlierIndices.Add(j);
                            }
                        }

                        // Fit plane to inlier points.
                        var result = FitPlaneToInlierPoints(cloudPoints, inlierIndices);

                        if (result.HasValue)
                        {
                            var refinedPlane = result.Value;

                            // Ensure normal is upward.
                            if (Vector3.Dot(refinedPlane.Normal, up) < 0)
                            {
                                refinedPlane.Normal = refinedPlane.Normal * -1;
                            }

                            // Ensure normal is mostly vertical.
                            var floorTiltInDeg = Math.Acos(Vector3.Dot(refinedPlane.Normal, up)) * 180.0f / 3.14159265f;
                            if (floorTiltInDeg < planeMaxTiltInDeg)
                            {
                                // For reduced jitter, use gravity for floor normal.
                                refinedPlane.Normal = up;
                                return refinedPlane;
                            }
                        }

                        return null;
                    }
                }
            }

            return null;
        }

        private Vector3? EstimateGravityVectorForDepthCamera(in ImuSample imuSample, in Calibration sensorCalibration)
        {
            // An accelerometer at rest on the surface of the Earth will measure an acceleration
            // due to Earth's gravity straight **upwards** (by definition) of g ~ 9.81 m/s2.
            // https://en.wikipedia.org/wiki/Accelerometer

            Vector3 imuAcc = imuSample.AccelerometerSample; // in meters per second squared.

            // Estimate gravity when device is not moving.
            if (Math.Abs(imuAcc.Length() - 9.81f) < 0.2f)
            {
                // Extrinsic Rotation from ACCEL to DEPTH.
                CalibrationDeviceType source = CalibrationDeviceType.Accel;
                CalibrationDeviceType target = CalibrationDeviceType.Depth;

                int index = (int)source * (int)CalibrationDeviceType.Num + (int)target;
                float[] R = sensorCalibration.DeviceExtrinsics[index].Rotation;

                Vector3 Rx = new Vector3(R[0], R[1], R[2]);
                Vector3 Ry = new Vector3(R[3], R[4], R[5]);
                Vector3 Rz = new Vector3(R[6], R[7], R[8]);
                Vector3 depthAcc = new Vector3(Vector3.Dot(Rx, imuAcc), Vector3.Dot(Ry, imuAcc) , Vector3.Dot(Rz, imuAcc));

                // The acceleration due to gravity, g, is in a direction toward the ground.
                // However an accelerometer at rest in a gravity field reports upward acceleration
                // relative to the local inertial frame (the frame of a freely falling object).
                Vector3 depthGravity = depthAcc * -1;
                return depthGravity;
            }

            return null;
        }

        struct HistogramBin
        {
            public int Count { get; set; }
            public float LeftEdge { get; set; }

            public HistogramBin(int count, float leftEdge)
            {
                Count = count;
                LeftEdge = leftEdge;
            }
        }

        private HistogramBin[] Histogram(in float[] values, float binSize)
        {
            float minVal = values.Min();
            float maxVal = values.Max();

            int binCount = 1 + (int)((maxVal - minVal) / binSize);

            // Bins
            HistogramBin[] histBins = new HistogramBin[binCount];
            for (int i = 0; i < binCount; i++)
            {
                histBins[i] = new HistogramBin(0, i * binSize + minVal);
            }

            // Counts
            for (int i = 0; i < values.Length; i++)
            {
                int binIndex = (int)((values[i] - minVal) / binSize);
                histBins[binIndex].Count++;
            }

            return histBins;
        }

        private Plane? FitPlaneToInlierPoints(in List<Vector3> cloudPoints, in List<int> inlierIndices)
        {
            // https://www.ilikebigbits.com/2015_03_04_plane_from_points.html

            if (inlierIndices.Count < 3)
            {
                return null;
            }

            // Compute centroid.
            Vector3 centroid = new Vector3(0, 0, 0);
            foreach (int index in inlierIndices)
            {
                centroid = centroid + cloudPoints[index];
            }
            centroid = centroid / (float)(inlierIndices.Count);

            // Compute the zero-mean 3x3 symmetric covariance matrix relative.
            float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
            foreach (int index in inlierIndices)
            {
                Vector3 r = cloudPoints[index] - centroid;
                xx += r.X * r.X;
                xy += r.X * r.Y;
                xz += r.X * r.Z;
                yy += r.Y * r.Y;
                yz += r.Y * r.Z;
                zz += r.Z * r.Z;
            }

            float detX = yy * zz - yz * yz;
            float detY = xx * zz - xz * xz;
            float detZ = xx * yy - xy * xy;

            float detMax = Math.Max(Math.Max(detX, detY), detZ);
            if (detMax <= 0)
            {
                return null;
            }

            Vector3 normal = new Vector3(0, 0, 0);
            if (detMax == detX)
            {
                normal = new Vector3(detX, xz * yz - xy * zz, xy * yz - xz * yy);
            }
            else if (detMax == detY)
            {
                normal = new Vector3(xz * yz - xy * zz, detY, xy * xz - yz * xx);
            }
            else
            {
                normal = new Vector3(xy * yz - xz * yy, xy * xz - yz * xx, detZ);
            }

            return Plane.Create(Vector3.Normalize(normal), centroid);
        }
    }
}
