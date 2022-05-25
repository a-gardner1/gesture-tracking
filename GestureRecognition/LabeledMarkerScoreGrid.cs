using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using Microsoft.VisualBasic.FileIO;

namespace GestureRecognition
{
    /// <summary>
    /// Useful for assigning observed markers to parts of the hand.
    /// 
    /// </summary>
    class LabeledMarkerScoreGrid
    {
        String fileName;
        bool useNeuralNet = false;
        BasicNeuralNetwork mlp = null;
        Vector dimensions = new Vector();
        Vector LB = new Vector();
        Vector UB = new Vector();
        double delta = 1;
        Matrix<double> gridScores;
        double[][][] corners = { new double[][] { new double[] { 0, 0 }, new double[] { 0, 0 } }, 
                                   new double[][] { new double[] { 0, 0 }, new double[] { 0, 0 } } };
        Matrix<double> mostRecentCostMatrix = new DenseMatrix(1);

        /// <summary>
        /// Get the most recently computed cost matrix from a call to labelPositions(), enumeratePositions(), or
        /// orderPosition(). This value, like the functions, is not thread-safe.
        /// 
        /// This cost matrix does not include the prior.
        /// </summary>
        public Matrix<double> MostRecentCostMatrix
        {
            get { return mostRecentCostMatrix; }
        }

        public int LabelCount
        {
            get
            {
                if(useNeuralNet)
                {
                    return mlp.OutputDim-1;
                }
                else
                {
                    return gridScores.ColumnCount;
                }
            }
        }

        public enum MarkerLabel
        {
            MIDDLEFINGERJOINT, MIDDLEFINGERTIP, PINKYFINGERJOINT, PINKYFINGERTIP, POINTERFINGERJOINT,
            POINTERFINGERTIP, RINGFINGERJOINT, RINGFINGERTIP, THUMBJOINT, THUMBKNUCKLE, THUMBTIP, UNAVAILABLE
        }

        public LabeledMarkerScoreGrid(String fileName, bool useNeuralNet = false)
        {
            this.fileName = fileName;
            this.useNeuralNet = useNeuralNet;
            if (useNeuralNet)
            {
                mlp = new BasicNeuralNetwork();
            }
        }

        public void load()
        {
            if (useNeuralNet)
            {
                loadNeuralNet();
            }
            else
            {
                loadGrid();
            }
        }

        /// <summary>
        /// Loads the precomputed grid scores from the file specified on construction.
        /// </summary>
        public void loadGrid()
        {
            using (TextFieldParser parser = new TextFieldParser(fileName))
            {
                System.Console.WriteLine("Loading SVM score grid . . .");
                parser.TextFieldType = FieldType.Delimited;
                parser.SetDelimiters(",");
                int i = 0;
                while (!parser.EndOfData)
                {
                    string[] fields = parser.ReadFields();
                    switch (i)
                    {
                        case 0: // y-dimension
                            dimensions.setY(Double.Parse(fields[0]));
                            break;
                        case 1: // x-dimenion
                            dimensions.setX(Double.Parse(fields[0]));
                            break;
                        case 2: // z-dimension
                            dimensions.setZ(Double.Parse(fields[0]));
                            gridScores = new DenseMatrix((int)(dimensions[0]*dimensions[1]*dimensions[2]), fields.Length);
                            break;
                        case 3: //LB-x
                            LB.setX(Double.Parse(fields[0]));
                            break;
                        case 4: //LB-y
                            LB.setY(Double.Parse(fields[0]));
                            break;
                        case 5: //LB-z
                            LB.setZ(Double.Parse(fields[0]));
                            break;
                        case 6: //UB-x
                            UB.setX(Double.Parse(fields[0]));
                            break;
                        case 7: //UB-y
                            UB.setY(Double.Parse(fields[0]));
                            break;
                        case 8: //UB-z
                            UB.setZ(Double.Parse(fields[0]));
                            break;
                        case 9: //delta
                            delta = Double.Parse(fields[0]);
                            break;
                        default: // scores
                            for(int j = 0; j < fields.Length; ++j)
                            {
                                gridScores[i - 10, j] = Double.Parse(fields[j]);
                            }
                            break;
                    }
                    if (i > 0 && i % 40000 == 0)
                    {
                        System.Console.WriteLine(". . .");
                    }
                    ++i;
                }
                System.Console.WriteLine("Grid loaded!");
            }
        }

        public void loadNeuralNet()
        {
            mlp.loadFromCSV(fileName);
        }

        /// <summary>
        /// Assign positions labels such that the joint probability of assigned markers is maximized.
        /// One may also provide a prior log-probability (/likelihood) in order to maximize the posterior joint probability.
        /// Not thread-safe!
        /// </summary>
        /// <param name="positions"></param>
        /// <returns></returns>
        public List<int> labelPositions(ReadOnlyCollection<Vector> positions, Matrix<double> prior = null)
        {
            if (prior != null && (prior.RowCount != positions.Count || prior.ColumnCount != LabelCount))
            {
                throw new ArgumentException("Prior distribution has wrong number of components.");
            }
            Matrix<double> costMatrix;
            if (useNeuralNet)
            {
                costMatrix = -mlp.batchActivate(
                                positions.ToList().Select<Vector, Vector<double>>(
                                (x) =>
                                {
                                    return new DenseVector(new double[] { x[0], x[1], x[2] });
                                }).ToList().AsReadOnly()).SubMatrix(0, prior.RowCount, 0, LabelCount);
            }
                else{
                    costMatrix = new DenseMatrix(positions.Count, LabelCount);
                }
            if(mostRecentCostMatrix.RowCount != costMatrix.RowCount 
                || mostRecentCostMatrix.ColumnCount != costMatrix.ColumnCount)
            {
                mostRecentCostMatrix = new DenseMatrix(positions.Count, LabelCount);
            }
            for (int p = 0; p < positions.Count; ++p)
            {
                for (int s = 0; s < LabelCount; ++s)
                {
                    if (!useNeuralNet)
                    {
                        costMatrix[p, s] = -Math.Log(getGridScore(positions[p], s));
                    }
                    //costMatrix[p, s] = 1-getGridScore(positions[p], s);
                    if (Double.IsInfinity(costMatrix[p, s]))
                    {
                        costMatrix[p, s] = 1000;
                    }
                    mostRecentCostMatrix[p, s] = costMatrix[p, s];
                    if (prior != null)
                    {
                        costMatrix[p, s] += prior[p, s];
                    }
                }
            }
            return Assignment.getAssignment(costMatrix);
        }

        public List<MarkerLabel> enumeratePositions(ReadOnlyCollection<Vector> positions, Matrix<double> prior = null)
        {
            List<int> assignment = labelPositions(positions, prior);
            List<MarkerLabel> enumerations = Enumerable.Repeat<MarkerLabel>(MarkerLabel.UNAVAILABLE, assignment.Count).ToList();
            for (int i = 0; i < assignment.Count; ++i )
            {
                if (assignment[i] != -1)
                {
                    enumerations[assignment[i]] = (MarkerLabel)i;
                }
            }
            return enumerations;
        }

        public List<Vector> orderPositions(ReadOnlyCollection<Vector> positions, Matrix<double> prior = null)
        {
            List<int> assignment = labelPositions(positions, prior);
            List<Vector> ordered = Enumerable.Repeat<Vector>(new Vector(true), assignment.Count).ToList();
            for (int i = 0; i < assignment.Count; ++i)
            {
                if (assignment[i] != -1)
                {
                    ordered[i] = positions[assignment[i]];
                }
            }
            return ordered;
        }


        public double getGridScore(Vector position, int grid)
        {
            int x = -1, y = -1, z = -1;
            Vector f = new Vector();
            if(position[0] >= LB[0] && position[0] <= UB[0] &&
                position[1] >= LB[1] && position[1] <= UB[1] &&
                position[2] >= LB[2] && position[2] <= UB[2])
            {
                Vector realIndices = (position - LB) / delta;
                x = (int)Math.Floor(realIndices.getX());
                y = (int)Math.Floor(realIndices.getY());
                z = (int)Math.Floor(realIndices.getZ());
                f[0] = realIndices[0] - x;
                f[1] = realIndices[1] - y;
                f[2] = realIndices[2] - z;
            }
            for(int i = 0; i < 8; ++i)
            {
                corners[i & 1][(i & 2) >> 1][(i & 4) >> 2] = getScore(x + (i & 1), y + ((i & 2) >> 1), z + ((i & 4) >> 2), grid);
            }
            // perform trilinear interpolation
            double score = ((corners[0][0][0] * (1 - f[0]) +
                corners[1][0][0] * f[0]) * (1 - f[1])
                + (corners[0][1][0] * (1 - f[0]) +
                corners[1][1][0] * f[0]) * f[1]) * (1 - f[2])
                +
                ((corners[0][0][1] * (1 - f[0]) +
                corners[1][0][1] * f[0]) * (1 - f[1])
                + (corners[0][1][1] * (1 - f[0]) +
                corners[1][1][1] * f[0]) * f[1]) * f[2];
            return score;
        }

        private double getScore(int x , int y, int z, int grid)
        {
            if(x < dimensions.getX() && y < dimensions.getY() && z < dimensions.getZ()
                && x >=0 && y >= 0 && z >= 0)
            {
                return gridScores[linearizeIndex(x, y, z), grid];
            }
            else
            {
                return 0;
            }
        }

        private int linearizeIndex(int x, int y, int z)
        {
            return (int)(z * dimensions.getY() * dimensions.getX() + x * dimensions.getY() + y);
        }

        private Tuple<int, int, int> delinearizeIndex(int index)
        {
            int y = index % (int)dimensions.getY();
            int x = (int)((index - y) / dimensions.getY()) % (int)dimensions.getX();
            int z = (int)((index - y - x * dimensions.getY()) / (dimensions.getY() * dimensions.getX()));
            return new Tuple<int, int, int>(x, y, z);
        }

        private List<List<Tuple<double, double, double, double>>> findMajorAreas(double threshold, bool byIndex)
        {
            var result = new List<List<Tuple<double, double, double, double>>>();
            for (int j = 0; j < gridScores.ColumnCount; ++j)
            {
                result.Add(new List<Tuple<double, double, double, double>>());
                for (int i = 0; i < gridScores.RowCount; ++i)
                {
                    if (gridScores[i, j] >= threshold)
                    {
                        var indices = delinearizeIndex(i);
                        if (byIndex)
                        {
                            result[j].Add(new Tuple<double, double, double, double>(indices.Item1,
                                indices.Item2,
                                indices.Item3,
                                gridScores[i, j]));
                        }
                        else
                        {
                            Vector position = new Vector(LB.getX() + indices.Item1 * delta,
                                LB.getY() + indices.Item2 * delta,
                                LB.getZ() + indices.Item3 * delta);
                            result[j].Add(new Tuple<double, double, double, double>(position.getX(),
                                position.getY(),
                                position.getZ(),
                                gridScores[i, j]));
                        }
                    }
                }
            }
            return result;
        }
    }
}
