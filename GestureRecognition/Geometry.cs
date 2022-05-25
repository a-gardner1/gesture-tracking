using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;
using System.Collections.ObjectModel;

namespace GestureRecognition
{
    class Geometry
    {
        public static List<Vector> getCoordinateSystem(List<Vector> pattern)
        {
            return getCoordinateSystem(pattern.AsReadOnly());
        }

        /// <summary>
        /// Returns the origin and axes of a coordinate system based on the given 4 marker pattern.
        /// </summary>
        /// <param name="pattern"></param>
        /// <returns></returns>
        public static List<Vector> getCoordinateSystem(ReadOnlyCollection<Vector> pattern)
        {
            List<Vector> system = new List<Vector>();
            Vector origin = pattern[0];
            if (!origin.isInvalid())
            {
                Vector xAxis = pattern[1];
                Vector yAxis = pattern[2];
                Vector extra = pattern[3];
                Vector zAxis = new Vector(true);
                if (!xAxis.isInvalid() && !yAxis.isInvalid())
                {
                    xAxis = (xAxis - origin).normed();
                    yAxis = (yAxis - origin).normed();
                    zAxis = xAxis.cross(yAxis).normed();
                    yAxis = zAxis.cross(xAxis).normed();
                }
                else if (!yAxis.isInvalid())
                {
                    yAxis = (yAxis - origin).normed();
                    zAxis = (extra - origin).cross(yAxis).normed();
                    xAxis = yAxis.cross(zAxis).normed();
                }
                else if (!xAxis.isInvalid())
                {
                    xAxis = (xAxis - origin).normed();
                    yAxis = extra - origin;
                    zAxis = xAxis.cross(yAxis).normed();
                    yAxis = zAxis.cross(xAxis).normed();
                }
                system.Add(origin);
                system.Add(xAxis);
                system.Add(yAxis);
                system.Add(zAxis);
            }
            return system;
        }

        /// <summary>
        /// Transform a list of points into another coordinate system.
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="points"></param>
        /// <returns></returns>
        public static List<Vector> transform(Vector origin, Vector x, Vector y, Vector z, ReadOnlyCollection<Vector> points)
        {
            List<Vector> transformed = new List<Vector>();
            foreach(Vector point in points)
            {
                transformed.Add((point - origin).transform(x*2, y, z));
            }
            return transformed;
        }

        /// <summary>
        /// Transform a list of points from a local system 
        /// specified in global coordinates to a global system.
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="points"></param>
        /// <returns></returns>
        public static List<Vector> transformToGlobal(Vector localOrigin, Vector localX, Vector localY, Vector localZ, ReadOnlyCollection<Vector> points)
        {
            List<Vector> globalVars = new List<Vector>(new Vector[] {new Vector(), 
                localOrigin + new Vector(1, 0, 0), 
                localOrigin + new Vector(0, 1, 0), 
                localOrigin + new Vector(0, 0, 1)});
            globalVars = transform(localOrigin, localX, localY, localZ, globalVars.AsReadOnly());
            return transform(globalVars[0], globalVars[1], globalVars[2], globalVars[3], points);
        }

        /// <summary>
        /// Transform a list of points into another coordinate system, 
        /// removing any point that is farther than a specified radius
        /// from the new origin.
        /// </summary>
        /// <param name="origin"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="points"></param>
        /// <param name="radius"></param>
        /// <returns></returns>
        public static List<Vector> transformAndPrune(Vector origin, Vector x, Vector y, Vector z, ReadOnlyCollection<Vector> points, double innerRadius, double outerRadius)
        {
            if (innerRadius > outerRadius)
            {
                return new List<Vector>();
            }
            List<Vector> transformed = new List<Vector>();
            foreach (Vector point in points)
            {
                Vector transformation = (point - origin).transform(x, y, z);
                if (transformation.getMagnitude() <= outerRadius && transformation.getMagnitude() >= innerRadius)
                {
                    transformed.Add(transformation);
                }
            }
            return transformed;
        }

        public static List<Vector> prune(Vector origin, ReadOnlyCollection<Vector> points,
            double innerRadius, double outerRadius)
        {
            if (innerRadius > outerRadius)
            {
                return new List<Vector>();
            }
            List<Vector> pruned = new List<Vector>();
            foreach (Vector point in points)
            {
                Vector distance = (point - origin);
                if (distance.getMagnitude() <= outerRadius && distance.getMagnitude() >= innerRadius)
                {
                    pruned.Add(point);
                }
            }
            return pruned;
        }

        /// <summary>
        /// Returns the mean squared error between two collections of Vectors.
        /// Assumes Vectors in each list are already given in corresponding order.
        /// If one list is smaller than the other or contains invalid vectors, 
        /// its missing elements are treated as zero unless indicated by the optional
        /// boolean flag.
        /// </summary>
        /// <param name="list1"></param>
        /// <param name="list2"></param>
        /// <returns></returns>
        public static double getSumSquareError(ReadOnlyCollection<Vector> list1, ReadOnlyCollection<Vector> list2, bool treatAsZero = true)
        {
            double mse = 0;
            Tuple<ReadOnlyCollection<Vector>,ReadOnlyCollection<Vector>> lists = list1.Count > list2.Count ? 
                new Tuple<ReadOnlyCollection<Vector>, ReadOnlyCollection<Vector>>(list2, list1) :
                new Tuple<ReadOnlyCollection<Vector>, ReadOnlyCollection<Vector>>(list1, list2);
            for (int i = 0; i < lists.Item1.Count; ++i)
            {
                if (!list1[i].isInvalid() && !list2[i].isInvalid())
                {
                    mse += Math.Pow((list1[i] - list2[i]).getMagnitude(), 2);
                }
                else if (treatAsZero)
                {
                    if (list1[i].isInvalid() && !list2[i].isInvalid())
                    {
                        mse += Math.Pow(list2[i].getMagnitude(), 2);
                    }
                    else if (!list1[i].isInvalid() && list2[i].isInvalid())
                    {
                        mse += Math.Pow(list1[i].getMagnitude(), 2);
                    }
                }
            }
            if (treatAsZero)
            {
                for (int i = lists.Item1.Count; i < lists.Item2.Count; ++i)
                {
                    mse += Math.Pow(lists.Item2[i].getMagnitude(), 2);
                }
            }
            return mse;
        }


        public static Matrix<double> calcRotationMatrix(ReadOnlyCollection<Vector> initial, ReadOnlyCollection<Vector> final)
        {
            System.Diagnostics.Debug.Assert(initial.Count == final.Count);
            Matrix<double> Y1, Y2;
            Y1 = new DenseMatrix(3, initial.Count);
            Y2 = new DenseMatrix(3, final.Count);
            for(int i = 0; i < initial.Count; ++i)
            {
                Y1[0, i] = initial[i].getX();
                Y1[1, i] = initial[i].getY();
                Y1[2, i] = initial[i].getZ();
            }
            for (int i = 0; i < final.Count; ++i)
            {
                Y2[0, i] = final[i].getX();
                Y2[1, i] = final[i].getY();
                Y2[2, i] = final[i].getZ();
            }
            Y1 = Y1.TransposeAndMultiply(Y2);
            MathNet.Numerics.LinearAlgebra.Factorization.Svd<double> svd = Y1.Svd();
            Y2 = CreateMatrix.DenseIdentity<double>(3);
            Y2[2, 2] = (svd.U.Multiply(svd.VT)).Determinant();
            return (svd.U * Y2 * svd.VT).Transpose();
        }
        public static Matrix<double> calcRotationMatrix(Vector<double> initial, Vector<double> final)
        {
            System.Diagnostics.Debug.Assert(initial.Count == final.Count);
            Matrix<double> Y1, Y2;
            Y1 = new DenseMatrix(3, initial.Count);
            Y2 = new DenseMatrix(3, final.Count);
            for (int i = 0; i < initial.Count; ++i)
            {
                Y1[0, i] = initial[3 * i];
                Y1[1, i] = initial[3 * i + 1];
                Y1[2, i] = initial[3 * i + 2];
            }
            for (int i = 0; i < final.Count; ++i)
            {
                Y2[0, i] = final[3 * i];
                Y2[1, i] = final[3 * i + 1];
                Y2[2, i] = final[3 * i + 2];
            }
            Y1 = Y1.TransposeAndMultiply(Y2);
            MathNet.Numerics.LinearAlgebra.Factorization.Svd<double> svd = Y1.Svd();
            Y2 = CreateMatrix.DenseIdentity<double>(3);
            Y2[2, 2] = (svd.U.Multiply(svd.VT)).Determinant();
            return (svd.U * Y2 * svd.VT).Transpose();
        }

        public static Vector calcAxisAngle(ReadOnlyCollection<Vector> initial, ReadOnlyCollection<Vector> final, 
            double smallTheta = 0, double bigTheta = Math.PI)
        {
            Matrix<double> RHat = calcRotationMatrix(initial, final);
            return calcAxisAngle(RHat, smallTheta, bigTheta);
        }

        public static Vector calcAxisAngle(Vector<double> initial, Vector<double> final,
            double smallTheta = 0, double bigTheta = Math.PI)
        {
            Matrix<double> RHat = calcRotationMatrix(initial, final);
            return calcAxisAngle(RHat, smallTheta, bigTheta);
        }

        public static Vector calcAxisAngle(Matrix<double> RHat,
            double smallTheta = 0, double bigTheta = Math.PI)
        {
            smallTheta = Math.Max(0, smallTheta);
            bigTheta = Math.Min(Math.PI, bigTheta);
            double atheta = (RHat.Trace() - 1) / 2; // numerical issues may cause impossible values slightly outside [-1,1]
            double theta = Math.Acos(atheta);
            if (theta <= smallTheta || atheta >= 1)
            {
                // approximately zero rotation
                return new Vector();
            }
            else if (theta >= bigTheta || atheta <= -1)
            {
                // approximately 180 degree rotation
                // using alternative calculation than what is proposed in paper
                // not sure which is more numerically stable
                return (new Vector(Math.Sqrt((RHat[0, 0] + 1) / 2),
                    Math.Sqrt((RHat[1, 1] + 1) / 2),
                    Math.Sqrt((RHat[2, 2] + 1) / 2))) * theta;
            }
            //else if (theta >= bigTheta || theta <= smallTheta)
            //{
            //    // small angle approximation to avoid division by zero
            //    return (new Vector(RHat[3, 2] - RHat[2, 3], RHat[1, 3] - RHat[3, 1], RHat[2, 1] - RHat[1, 2])) / 2;
            //}
            else
            {
                return (new Vector(RHat[2, 1] - RHat[1, 2], RHat[0, 2] - RHat[2, 0], RHat[1, 0] - RHat[0, 1])) * (theta / (2 * Math.Sin(theta)));
            }
        }

        public static Matrix<double> getCrossProductMatrix(Vector v)
        {
            Matrix<double> V = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(3);
            V[0, 1] = -v[2];
            V[0, 2] = v[1];
            V[1, 2] = -v[0];
            V[1, 0] = -V[0, 1];
            V[2, 0] = -V[0, 2];
            V[2, 1] = -V[1, 2];
            return V;
        }

        public static Matrix<double> getOuterProductMatrix(Vector v, Vector u)
        {
            Matrix<double> O = new MathNet.Numerics.LinearAlgebra.Double.DenseMatrix(3);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    O[i, j] = v[i] * u[j];
                }
            }
            return O;
        }
    }
}
