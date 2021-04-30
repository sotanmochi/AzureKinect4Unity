// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
//
// This code is a ported and modified version of the floor plane detector from Azure Kinect DK Code Samples.
//
// The original source code is available on GitHub.
// https://github.com/microsoft/Azure-Kinect-Samples/blob/master/body-tracking-samples/floor_detector_sample/SampleMathTypes.h
//

using System;
using System.Numerics;
using System.Runtime.InteropServices;
using Point = System.Numerics.Vector3;

namespace AzureKinectToolkit
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Plane
    {
        public Vector3 Normal { get; set; }
        public Point Origin { get; set; }
        public float C { get; set; }

        public Plane(Vector3 normal, Point origin, float c)
        {
            this.Normal = normal;
            this.Origin = origin;
            this.C = c;
        }

        public static Plane Create(Vector3 n, Point p)
        {
            float c = n.X * p.X + n.Y * p.Y + n.Z * p.Z;
            return new Plane(n, p, -c);
        }

        public static Plane Create(in Point p1, in Point p2, in Point p3)
        {
            Vector3 v1 = p2 - p1;
            Vector3 v2 = p2 - p3;
            Vector3 n = v1 * v2;
            return Create(n, p1);
        }

        public Vector3 ProjectVector(in Vector3 v)
        {
            return v - Normal * (Vector3.Dot(v, Normal) / Normal.LengthSquared());
        }

        public Vector3 ProjectPoint(in Point p)
        {
            return Origin + ProjectVector(p - Origin);
        }

        public float AbsDistance(in Point p)
        {
            return Math.Abs(p.X * Normal.X + p.Y * Normal.Y + p.Z * Normal.Z + C) / Normal.Length();
        }
    }
}
