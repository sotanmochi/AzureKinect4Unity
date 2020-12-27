// Copyright (c) 2020 Soichiro Sugimoto
// Licensed under the MIT License. See LICENSE in the project root for license information.

using System;

namespace AzureKinect4Unity
{
    public class ButterworthFilter : ILowPassFilter
    {
        uint _VectorSize;

        uint N; // Order
        double [,] xk; // Input buffer
        double [,] yk; // Outpuf buffer
        double[] a; // Coefficient
        double[] b; // Coefficient

        /// <summary>
        /// Create a Butterworth filter
        /// </summary>
        /// <param name="order"></param>
        /// <param name="fs">Sampling frequency [Hz]</param>
        /// <param name="fc">Cutoff frequency [Hz]</param>
        /// <param name="vectorSize">Vector data size</param>
        public ButterworthFilter(uint order, double fs, double fc, uint vectorSize = 1)
        {
            _VectorSize = vectorSize;

            N = Math.Min(order, 4);
            N = Math.Max(order, 1);

            xk = new double[(N + 1), vectorSize];
            yk = new double[(N + 1), vectorSize];

            a = new double[(N + 1)];
            b = new double[(N + 1)];

            double fa = Math.Tan(Math.PI * fc / fs);
            double sqrt2 = Math.Sqrt(2);
    
            // UnityEngine.Debug.Log("fa: " + fa);
            // UnityEngine.Debug.Log("Pi*fc/fs: " + (Math.PI * fc / fs));

            if (N == 1)
            {
                b[0] = 1;
                b[1] = (fa - 1) / (fa + 1);

                a[0] = fa / (fa + 1);
                a[1] = fa / (fa + 1);

                // UnityEngine.Debug.Log("Sum of coeff (order:1): " + (-b[1] + a[0] + a[1]));

                // UnityEngine.Debug.Log("**************************");
                // UnityEngine.Debug.Log("b[1]: " + b[1]);
                // UnityEngine.Debug.Log("a[0]: " + a[0]);
                // UnityEngine.Debug.Log("a[1]: " + a[1]);
                // UnityEngine.Debug.Log("**************************");
            }
            else if (N == 2)
            {
                b[0] = 1;
                b[1] = (2*fa*fa - 2) / (fa*fa + sqrt2*fa + 1);
                b[2] = (fa*fa - sqrt2*fa + 1) / (fa*fa + sqrt2*fa + 1);

                a[0] = (fa*fa) / (fa*fa + sqrt2*fa + 1);
                a[1] = (2*fa*fa) / (fa*fa + sqrt2*fa + 1);
                a[2] = (fa*fa) / (fa*fa + sqrt2*fa + 1);

                // UnityEngine.Debug.Log("Sum of coeff (order:2): " + (-b[1] - b[2] + a[0] + a[1] + a[2]));

                // UnityEngine.Debug.Log("**************************");
                // UnityEngine.Debug.Log("b[1]: " + b[1]);
                // UnityEngine.Debug.Log("b[2]: " + b[2]);
                // UnityEngine.Debug.Log("a[0]: " + a[0]);
                // UnityEngine.Debug.Log("a[1]: " + a[1]);
                // UnityEngine.Debug.Log("a[2]: " + a[2]);
                // UnityEngine.Debug.Log("**************************");
            }
            else if (N == 3)
            {
                double temp = 1.0 + 2.0*fa + 2.0*fa*fa + fa*fa*fa;

                b[0] = 1;
                b[1] = (-3.0 - 2.0*fa + 2.0*fa*fa + 3.0*fa*fa*fa) / temp;
                b[2] = ( 3.0 - 2.0*fa - 2.0*fa*fa + 3.0*fa*fa*fa) / temp;
                b[3] = (-1.0 + 2.0*fa - 2.0*fa*fa + 1.0*fa*fa*fa) / temp;

                a[0] = (1.0*fa*fa*fa) / temp;
                a[1] = (3.0*fa*fa*fa) / temp;
                a[2] = (3.0*fa*fa*fa) / temp;
                a[3] = (1.0*fa*fa*fa) / temp;

                // UnityEngine.Debug.Log("Sum of coeff (order:3): " + (-b[1] - b[2] - b[3] + a[0] + a[1] + a[2] + a[3]));
            }
            else if (N == 4)
            {
                double alpha = 2.0*Math.Cos(5.0/8.0*Math.PI) + 2.0*Math.Cos(7.0/8.0*Math.PI);
                double beta = 2.0 + 2.0/sqrt2;
                double temp = fa*fa*fa*fa - alpha*fa*fa*fa + beta*fa*fa - alpha*fa + 1.0;

                b[0] = 1;
                b[1] = (4.0*fa*fa*fa*fa - 2.0*alpha*fa*fa*fa + 2.0*alpha*fa - 4.0) / temp;
                b[2] = (6.0*fa*fa*fa*fa - 2.0*beta*fa*fa + 6.0) / temp;
                b[3] = (4.0*fa*fa*fa*fa + 2.0*alpha*fa*fa*fa - 2.0*alpha*fa - 4.0) / temp;
                b[4] = (1.0*fa*fa*fa*fa + alpha*fa*fa*fa + beta*fa*fa + alpha*fa + 1.0) / temp;

                a[0] = (1.0*fa*fa*fa*fa) / temp;
                a[1] = (4.0*fa*fa*fa*fa) / temp;
                a[2] = (6.0*fa*fa*fa*fa) / temp;
                a[3] = (4.0*fa*fa*fa*fa) / temp;
                a[4] = (1.0*fa*fa*fa*fa) / temp;

                // UnityEngine.Debug.Log("Sum of coeff (order:4): " + (-b[1] - b[2] - b[3] - b[4] + a[0] + a[1] + a[2] + a[3] + a[4]));
            }
        }

        public bool Init(in float[] input)
        {
            if (input.Length != _VectorSize)
            {
                return false;
            }

            if (N == 1)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[1,m] = input[m]; // x[k - 1]
                    yk[1,m] = input[m]; // y[k - 1]
                }
            }
            else if (N == 2)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[2,m] = input[m]; // x[k - 2]
                    xk[1,m] = input[m]; // x[k - 1]
                    yk[2,m] = input[m]; // y[k - 2]
                    yk[1,m] = input[m]; // y[k - 1]
                }
            }
            else if (N == 3)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[3,m] = input[m]; // x[k - 3]
                    xk[2,m] = input[m]; // x[k - 2]
                    xk[1,m] = input[m]; // x[k - 1]
                    yk[3,m] = input[m]; // y[k - 3]
                    yk[2,m] = input[m]; // y[k - 2]
                    yk[1,m] = input[m]; // y[k - 1]
                }
            }
            else if (N == 4)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[4,m] = input[m]; // x[k - 4]
                    xk[3,m] = input[m]; // x[k - 3]
                    xk[2,m] = input[m]; // x[k - 2]
                    xk[1,m] = input[m]; // x[k - 1]
                    yk[4,m] = input[m]; // y[k - 4]
                    yk[3,m] = input[m]; // y[k - 3]
                    yk[2,m] = input[m]; // y[k - 2]
                    yk[1,m] = input[m]; // y[k - 1]
                }
            }

            return true;
        }

        public bool Apply(in float[] input, ref float[] output)
        {
            if ((input.Length != output.Length)
            ||  (input.Length != _VectorSize))
            {
                return false;
            }

            if (N == 1)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[0,m] = input[m];
                }
                for (int m = 0; m < input.Length; m++)
                {
                    // y[k] = (a0*x[k] + a1*x[k - 1]) - b1*y[k - 1]
                    yk[0,m] = (float)(a[0]*xk[0,m] + a[1]*xk[1,m] - b[1]*yk[1,m]);
                }
                for (int m = 0; m < output.Length; m++)
                {
                    output[m] = (float)yk[0,m];
                    xk[1,m] = xk[0,m]; // x[k - 1] <- x[k]
                    yk[1,m] = yk[0,m]; // y[k - 1] <- y[k]
                }
            }
            else if (N == 2)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[0,m] = input[m];
                }
                for (int m = 0; m < input.Length; m++)
                {
                    // y[k] = (a0*x[k] + a1*x[k - 1] + a2*x[k - 2]) 
                    //                 -(b1*y[k - 1] + b2*y[k - 2])
                    yk[0,m] = (float)(a[0]*xk[0,m] + a[1]*xk[1,m] + a[2]*xk[2,m]
                                                   - b[1]*yk[1,m] - b[2]*yk[2,m]);
                }
                for (int m = 0; m < output.Length; m++)
                {
                    output[m] = (float)yk[0,m];
                    xk[2,m] = xk[1,m]; // x[k - 2] <- x[k - 1]
                    xk[1,m] = xk[0,m]; // x[k - 1] <- x[k]
                    yk[2,m] = yk[1,m]; // y[k - 2] <- y[k - 1]
                    yk[1,m] = yk[0,m]; // y[k - 1] <- y[k]
                }
            }
            else if (N == 3)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[0,m] = input[m];
                }
                for (int m = 0; m < input.Length; m++)
                {
                    // y[k] = (a0*x[k] + a1*x[k - 1] + a2*x[k - 2] + a3*x[k - 3])
                    //                 -(b1*y[k - 1] + b2*y[k - 2] + b3*y[k - 3])
                    yk[0,m] = (float)(a[0]*xk[0,m] + a[1]*xk[1,m] + a[2]*xk[2,m] + a[3]*xk[3,m]
                                                   - b[1]*yk[1,m] - b[2]*yk[2,m] - b[3]*yk[3,m]);
                }
                for (int m = 0; m < output.Length; m++)
                {
                    output[m] = (float)yk[0,m];
                    xk[3,m] = xk[2,m]; // x[k - 3] <- x[k - 2]
                    xk[2,m] = xk[1,m]; // x[k - 2] <- x[k - 1]
                    xk[1,m] = xk[0,m]; // x[k - 1] <- x[k]
                    yk[3,m] = yk[2,m]; // y[k - 3] <- y[k - 2]
                    yk[2,m] = yk[1,m]; // y[k - 2] <- y[k - 1]
                    yk[1,m] = yk[0,m]; // y[k - 1] <- y[k]
                }
            }
            else if (N == 4)
            {
                for (int m = 0; m < input.Length; m++)
                {
                    xk[0,m] = input[m];
                }
                for (int m = 0; m < input.Length; m++)
                {
                    // y[k] = (a0*x[k] + a1*x[k - 1] + a2*x[k - 2] + a3*x[k - 3] + a4*x[k - 4])
                    //                 -(b1*y[k - 1] + b2*y[k - 2] + b3*y[k - 3] + b4*y[k - 4])
                    yk[0,m] = (float)(a[0]*xk[0,m] + a[1]*xk[1,m] + a[2]*xk[2,m] + a[3]*xk[3,m] + a[4]*xk[4,m]
                                                   - b[1]*yk[1,m] - b[2]*yk[2,m] - b[3]*yk[3,m] - b[4]*yk[4,m]);
                }
                for (int m = 0; m < output.Length; m++)
                {
                    output[m] = (float)yk[0,m];
                    xk[4,m] = xk[3,m]; // x[k - 4] <- x[k - 3]
                    xk[3,m] = xk[2,m]; // x[k - 3] <- x[k - 2]
                    xk[2,m] = xk[1,m]; // x[k - 2] <- x[k - 1]
                    xk[1,m] = xk[0,m]; // x[k - 1] <- x[k]
                    yk[4,m] = yk[3,m]; // y[k - 4] <- y[k - 3]
                    yk[3,m] = yk[2,m]; // y[k - 3] <- y[k - 2]
                    yk[2,m] = yk[1,m]; // y[k - 2] <- y[k - 1]
                    yk[1,m] = yk[0,m]; // y[k - 1] <- y[k]
                }
            }

            return true;
        }
    }
}
