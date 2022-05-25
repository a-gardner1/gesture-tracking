using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    /// <summary>
    /// Implements a basic, sequential neural network, i.e. a multilayer perceptron. 
    /// The ability to train is not currently provided.
    /// </summary>
    class BasicNeuralNetwork
    {
        
        public delegate Vector<double> Activation(Vector<double> input);

        public static Activation relu = (Vector<double> x) =>
        {
            x = x.Clone();
            for (int i = 0; i < x.Count; ++i)
            {
                x[i] = Math.Max(0, x[i]);
            }
            return x;
        };

        public static Activation sigmoid = (Vector<double> x) =>
        {
            x = x.Clone();
            for (int i = 0; i < x.Count; ++i)
            {
                if (x[i] >= 0)
                {
                    x[i] = 1 / (1 + Math.Exp(-x[i]));
                }
                else
                {
                    double z = Math.Exp(x[i]);
                    x[i] = z / (1 + z);
                }
            }
            return x;
        };

        public static Activation tanh = (Vector<double> x) =>
        {
            return 2 * sigmoid(x) - 1;
        };

        public static Activation softmax = (Vector<double> x) =>
        {
            x = x.Clone();
            double max = x.Maximum();
            for (int i = 0; i < x.Count; ++i)
            {
                x[i] = Math.Exp(x[i]-max);
            }
            double sum = x.Sum();
            x = x / sum;
            return x;
        };

        public static Activation log_softmax = (Vector<double> x) =>
        {
            Vector<double> tmp = x.Clone();
            double max = x.Maximum();
            for (int i = 0; i < x.Count; ++i)
            {
                tmp[i] = Math.Exp(x[i] - max);
            }
            double sum = Math.Log(tmp.Sum());
            x = x - (sum + max);
            return x;
        };

        Activation parseActivationString(string a, bool forceLog = false)
        {
            switch (a.ToLower())
            {
                case "relu":
                    return relu;
                default:
                case "sigmoid":
                    return sigmoid;
                case "tanh":
                    return tanh;
                case "softmax":
                    if (forceLog)
                    {
                        return log_softmax;
                    }
                    else
                    {
                        return softmax;
                    }
            }
        }

        public class Layer
        {
            public Layer(int inputSize, int outputSize, Activation a)
            {
                W = new DenseMatrix(outputSize, inputSize);
                b = new DenseVector(outputSize);
                activation = a;
            }

            public Layer(Matrix<double> W, Vector<double> b, Activation a)
            {
                this.W = W.Clone();
                this.b = b.Clone();
                activation = a;
            }

            private Matrix<double> W;
            private Vector<double> b;
            private Activation activation;

            public int InputDim
            {
                get
                {
                    return W.ColumnCount;
                }
            }

            public int OutputDim
            {
                get
                {
                    return W.RowCount;
                }
            }

            public Vector<double> activate(Vector<double> input)
            {
                return activation(W * input + b);
            }
        }

        List<Layer> layers = new List<Layer>();

        public BasicNeuralNetwork()
        {
        }


        public int InputDim
        {
            get { return layers[0].InputDim; }
        }

        public int OutputDim
        {
            get { return layers.Last().OutputDim; }
        }

        public void loadFromCSV(String fileName)
        {
            System.IO.StreamReader reader = new System.IO.StreamReader(fileName);
            List<string[]> lines = new List<string[]>();
            string nextLine;
            while ((nextLine = reader.ReadLine()) != null)
            {
                string[] tokens = nextLine.Split(',');
                if (tokens.Length > 0)
                {
                    lines.Add(tokens);
                }
            }
            reader.Close();
            Activation activation = null;
            Matrix<double> W = null;
            Vector<double> b = null;
            int attributeCount = 0;
            for(int i = 0; i < lines.Count; ++i)
            {
                if(attributeCount == 7)
                {
                    // a full set of attributes has been loaded. add a layer
                    addLayer(new Layer(W, b, activation));
                    attributeCount = 0;
                }
                switch (lines[i][0].ToLower())
                {
                    case "activation":
                        activation = parseActivationString(lines[i][1].ToLower(), true);
                        attributeCount |= 1;
                        break;
                    case "weights":
                        W = new DenseMatrix(Int32.Parse(lines[i][1]), Int32.Parse(lines[i][2]));
                        ++i;
                        for (int j = 0; j < W.RowCount; ++j)
                        {
                            for (int k = 0; k < W.ColumnCount; ++k)
                            {
                                W[j, k] = Double.Parse(lines[i + j][k]);
                            }
                        }
                        i += W.RowCount - 1;
                        attributeCount |= 2;
                        break;
                    case "bias":
                        b = new DenseVector(Int32.Parse(lines[i][1]));
                        ++i;
                        for (int j = 0; j < b.Count; ++j)
                        {
                            b[j] = Double.Parse(lines[i][j]);
                        }
                        attributeCount |= 4;
                        break;
                }
            }
            if (attributeCount == 7)
            {
                // a full set of attributes has been loaded. add the final layer
                addLayer(new Layer(W, b, activation));
            }
        }

        public void addLayer(Layer next)
        {
            if (layers.Count == 0 || next.InputDim == layers.Last().OutputDim)
            {
                layers.Add(next);
            }
            else
            {
                throw new ArgumentException("Expected input dimension of " + layers.Last().OutputDim + " but got " + next.InputDim + ".");
            }
        }

        public Vector<double> activate(Vector<double> input)
        {
            if (input.Count != InputDim)
            {
                throw new ArgumentException("Expected input dimension of " + InputDim + " but got " + input.Count + ".");
            }
            for (int i = 0; i < layers.Count; ++i)
            {
                input = layers[i].activate(input);
            }
            return input;
        }

        public Matrix<double> batchActivate(System.Collections.ObjectModel.ReadOnlyCollection<Vector<double>> inputs)
        {
            Matrix<double> output = new DenseMatrix(inputs.Count, OutputDim);
            for (int i = 0; i < inputs.Count; ++i)
            {
                output.SetRow(i, activate(inputs[i]));
            }
            return output;
        }
    }
}
