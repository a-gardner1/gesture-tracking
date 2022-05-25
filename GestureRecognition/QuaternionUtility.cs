using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    abstract class QuaternionUtility
    {
       
        public static Vector<double> toMathNetVector(Quaternion q)
        {
            return new DenseVector(new double[] { q.Scalar, q.Vector[0], q.Vector[1], q.Vector[2] });
        }

        /// <summary>
        /// Calculate the derivative of the product p*q of two quaternions p and q with respect
        /// to the left quaternion p or right quaternion q.
        /// </summary>
        /// <param name="p"></param>
        /// <param name="isLeft"></param>
        /// <returns></returns>
        private static Matrix<double> productDeriv(Quaternion p, bool isLeft)
        {
            Matrix<double> D = new DenseMatrix(4);
            for (int i = 0; i < 4; ++i )
            {
                D[0, i] = -p[i];
                D[i, 0] = p[i];
            }
            if (isLeft)
            {
                D.SetSubMatrix(1, 1, p.Scalar * CreateMatrix.DenseIdentity<double>(3) - Geometry.getCrossProductMatrix(p.Vector));
            }
            else
            {
                D.SetSubMatrix(1, 1, p.Scalar * CreateMatrix.DenseIdentity<double>(3) + Geometry.getCrossProductMatrix(p.Vector));
            }
            return D;
        }

        public static Matrix<double> leftDeriv(Quaternion q)
        {
            return productDeriv(q, true);
        }

        public static Matrix<double> rightDeriv(Quaternion p)
        {
            return productDeriv(p, false);
        }
        
        /// <summary>
        /// Calculate the product pqp^*, where p and q are quaternions and
        /// p^* is the conjugate of p. 
        /// Useful for calculating rotations.
        /// </summary>
        /// <param name="p"></param>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Quaternion conjugateProduct(Quaternion p, Quaternion q)
        {
            //Quaternion testValue = new Quaternion(p[0] * p[0] * q[0] + q[0] * p.Vector.dot(p.Vector),
            //    p[0] * (p[0] * q.Vector + 2 * p.Vector.cross(q.Vector))
            //    + p.Vector * p.Vector.dot(q.Vector)
            //    - p.Vector.cross(q.Vector).cross(p.Vector));

            return p * q * p.conjugate();
        }

        /// <summary>
        /// Calculate the product pqp^*, where p and q are quaternions and
        /// p^* is the conjugate of p. 
        /// Useful for calculating rotations.
        /// </summary>
        /// <param name="p"></param>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Quaternion conjugateProduct(Quaternion p, Vector v)
        {
            return p * (new Quaternion(0, v)) * p.conjugate();
        }

        /// <summary>
        /// Calculate the derivative of pqp^* with respect to p, where p and q are quaternions and
        /// p^* is the conjugate of p.
        /// </summary>
        /// <param name="p"></param>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Matrix<double> conjugateProductOuterDeriv(Quaternion p, Quaternion q)
        {
            Matrix<double> D = new DenseMatrix(4);
            Quaternion cross = new Quaternion(0, p.Vector.cross(q.Vector));
            for(int i = 0; i < 4; ++i) 
            {
                D[0, i] = 2 * q.Scalar * p[i];
                D[i, 0] = 2 * p.Scalar * q[i] + 2 * cross[i];
            }
            //use outer product in-place
            D.SetSubMatrix(1, 1, Geometry.getOuterProductMatrix(p.Vector, q.Vector));
            D.SetSubMatrix(1, 1, 2 * (CreateMatrix.DenseIdentity<double>(3) * p.Vector.dot(q.Vector)
                + D.SubMatrix(1, 3, 1, 3) // outer product
                - p[0] * Geometry.getCrossProductMatrix(q.Vector)
                - D.SubMatrix(1, 3, 1, 3).Transpose())); // transpose of outer product
            return D;
        }

        /// <summary>
        /// Calculate the derivative of pqp^* with respect to q, where p and q are quaternions and
        /// p^* is the conjugate of p.
        /// </summary>
        /// <param name="p"></param>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Matrix<double> conjugateProductInnerDeriv(Quaternion p, Quaternion q)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Each rotation vector v is associated with a unit quaternion q(v).
        /// Calculates the derivative of q(v) with respect to v.
        /// </summary>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Matrix<double> quaternionizationDeriv(Vector v, double smallTheta = 0.0001)
        {
            Matrix<double> D = new DenseMatrix(4, 3);
            double magnitude = v.getMagnitude();
            if(magnitude <= smallTheta)
            {
                for(int i = 0; i < 3; ++i)
                {
                    D[0, i] = -v[i] / 4.0;
                }
                D.SetSubMatrix(1, 0, CreateMatrix.DenseIdentity<double>(3) / 2 - Geometry.getOuterProductMatrix(v, v) / 24);
            }
            else
            {
                double magD2 = magnitude /2;
                double sinTheta = Math.Sin(magD2) / magnitude;
                double cosSinTheta = (magD2 * Math.Cos(magD2) - Math.Sin(magD2)) / (magnitude * magnitude * magnitude);
                for (int i = 0; i < 3; ++i)
                {
                    D[0, i] = -sinTheta * v[i] / 2.0;
                }
                D.SetSubMatrix(1, 0, CreateMatrix.DenseIdentity<double>(3) * sinTheta + Geometry.getOuterProductMatrix(v, v) * cosSinTheta);
            }
            return D;
        }

        /// <summary>
        /// Each rotation vector v is associated with a unit quaternion q(v).
        /// Calculates the derivative of q(v) with respect to v in terms of q(v).
        /// If q(v) is not a unit quaternion, it is normalized internally prior to
        /// computation. 
        /// </summary>
        /// <param name="q"></param>
        /// <returns></returns>
        public static Matrix<double> quaternionizationDeriv(Quaternion q, double threshold = 0.0001, double smallTheta = 0.0001)
        {
            Vector v;
            // normalize q to unit norm 
            q = q / Math.Sqrt(q.norm());
            if (Math.Abs(q[0]) + threshold >= 1)
            {
                // no rotation
                v = new Vector();
            }
            else
            {
                v = new Vector(q[1], q[2], q[3]);
                v = v * (2 * Math.Acos(q[0]) / Math.Sqrt(1 - q[0] * q[0]));
            }
            return quaternionizationDeriv(v, smallTheta);
        }

    }
}
