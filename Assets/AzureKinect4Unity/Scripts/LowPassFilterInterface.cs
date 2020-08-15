// Copyright (c) Soichiro Sugimoto.
// Licensed under the MIT License. See LICENSE in the project root for license information.

namespace AzureKinect4Unity
{
    public interface LowPassFilter
    {
        bool Apply(in float[] input, ref float[] output);
    }
}
