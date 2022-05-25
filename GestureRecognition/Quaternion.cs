using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace GestureRecognition
{
    class Quaternion
    {
        double scalar;
        Vector vector;

        public Quaternion()
        {
            scalar = 0;
            vector = new Vector();
        }

        public Quaternion(Quaternion other)
        {
            scalar = other.scalar;
            vector = new Vector(other.vector);
        }

        public Quaternion(double scalar, double i, double j, double k)
        {
            this.scalar = scalar;
            vector = new Vector(i, j, k);
        }

        public Quaternion(double scalar, Vector vector)
        {
            this.scalar = scalar;
            this.vector = vector;
        }

        public Quaternion(Vector rotation, double smallTheta = 0.0001)
        {
            scalar = Math.Cos(rotation.getMagnitude() / 2);
            if (rotation.getMagnitude() <= smallTheta)
            {
                vector = rotation / 2;
            }
            else
            {
                vector = rotation * (Math.Sin(rotation.getMagnitude() / 2) / rotation.getMagnitude());
            }
        }

        public Vector toAxisAngle()
        {
            Quaternion unit = this / modulus();
            if (unit.scalar != 1)
            {
                double theta = 2 * Math.Acos(unit.scalar);
                return unit.vector * (theta / unit.vector.getMagnitude());
            }
            else
            {
                return new Vector();
            }
        }

        public double this[int index]
        {
            get
            {
                if (index == 0)
                {
                    return scalar;
                }
                else
                {
                    return vector[index - 1];
                }
            }
            set
            {
                if (index == 0)
                {
                    scalar = value;
                }
                else
                {
                    vector[index - 1] = value;
                }
            }
        }

        public double Scalar
        {
            get
            {
                return scalar;
            }
        }

        public Vector Vector
        {
            get
            {
                return vector;
            }
        }

        public static Quaternion operator +(Quaternion lhs, Quaternion rhs)
        {
            return new Quaternion(lhs.scalar + rhs.scalar, lhs.vector + rhs.vector);
        }

        public static Quaternion operator -(Quaternion lhs, Quaternion rhs)
        {
            return new Quaternion(lhs.scalar - rhs.scalar, lhs.vector - rhs.vector);
        }

        public static Quaternion operator *(double lhs, Quaternion rhs)
        {
            return new Quaternion(lhs * rhs.scalar, rhs.vector * lhs);
        }

        public static Quaternion operator *(Quaternion lhs, double rhs)
        {
            return new Quaternion(lhs.scalar * rhs, lhs.vector * rhs);
        }

        public static Quaternion operator /(Quaternion lhs, double rhs)
        {
            return new Quaternion(lhs.scalar / rhs, lhs.vector / rhs);
        }

        public static Quaternion operator *(Quaternion lhs, Quaternion rhs)
        {
            return new Quaternion(lhs[0] * rhs[0] - lhs.vector.dot(rhs.vector),
                rhs.vector * lhs[0] + lhs.vector * rhs[0] + lhs.vector.cross(rhs.vector));
        }

        /// <summary>
        /// Right divide rhs by lhs.
        /// I.e. given two quaternions p and q, solves for quaternion x where
        /// xq = p.
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static Quaternion operator /(Quaternion lhs, Quaternion rhs)
        {
            return rhs * (lhs.conjugate() / lhs.norm());
        }

        /// <summary>
        /// Left divide rhs by lhs.
        /// I.e. given two quaternions p and q, solves for quaternion x where
        /// qx = p.
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public static Quaternion operator |(Quaternion lhs, Quaternion rhs)
        {
            return (lhs.conjugate() / lhs.norm()) * rhs;
        }

        public double norm()
        {
            return scalar * scalar + vector.getMagnitude() * vector.getMagnitude();
        }

        public double modulus()
        {
            return Math.Sqrt(norm());
        }

        public Quaternion conjugate()
        {
            return new Quaternion(scalar, vector * -1);
        }

        public Quaternion inverse()
        {
            return conjugate() / norm();
        }

        public double dot(Quaternion other)
        {
            return scalar * other.scalar + vector.dot(other.vector);
        }

        /// <summary>
        /// Spherical linear interpolation (SLERP)
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="t"></param> Between 0 and 1.
        /// <returns></returns>
        public static Quaternion SLERP(Quaternion start, Quaternion end, double t)
        {
            if (t <= 0)
            {
                return new Quaternion(start);
            }
            else if (t >= 1)
            {
                return new Quaternion(end);
            }
            else
            {
                double theta = Math.Acos(start.dot(end));
                return Math.Sin((1 - t) * theta) / Math.Sin(theta) * start + Math.Sin(t * theta) / Math.Sin(theta) * end;
            }
        }
    }
}
