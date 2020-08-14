// Copyright (c) Soichiro Sugimoto.
// Licensed under the MIT License. See LICENSE in the project root for license information.

namespace AzureKinect4Unity
{
    public class ExponentialSmoothingLowPassFilter
    {
        float[] _LastOutput;
        float _Weight;

        public ExponentialSmoothingLowPassFilter(uint vectorSize, float weight = 0.1f)
        {
            _LastOutput = new float[vectorSize];
            _Weight = weight;
        }

        public bool Apply(in float[] input, ref float[] output)
        {
            if ((input.Length != output.Length)
            ||  (input.Length != _LastOutput.Length))
            {
                return false;
            }

            for (int k = 0; k < input.Length; k++)
            {
                output[k] = _Weight * input[k] + (1.0f - _Weight) * _LastOutput[k];
                _LastOutput[k] = output[k];
            }

            return true;
        }
    }
}
