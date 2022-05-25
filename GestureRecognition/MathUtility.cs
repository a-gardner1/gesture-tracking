using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace GestureRecognition
{
    abstract class MathUtility
    {
        public static Vector<double> upperTriangleVectorization(Matrix<double> mat)
        {

            int numElements;
            if (mat.RowCount <= mat.ColumnCount)
            {
                numElements = mat.RowCount * mat.ColumnCount - mat.RowCount * (mat.RowCount + 1) / 2 + mat.RowCount;
            }
            else
            {
                numElements = mat.RowCount * mat.ColumnCount - mat.ColumnCount * (mat.ColumnCount + 1) / 2 + mat.ColumnCount;
            }
            Vector<double> upperTriangle = new DenseVector(numElements);
            for (int i = 0, k = 0; i < mat.ColumnCount; ++i)
            {
                for (int j = 0; j <= Math.Min(i, mat.RowCount); ++j, ++k)
                {
                    upperTriangle[k] = mat[j, i];
                }
            }
            return upperTriangle;
        }

        public static Matrix<double> toDiagonalMatrix(Vector<double> vec)
        {
            Matrix<double> result = new SparseMatrix(vec.Count);
            for(int i = 0; i < vec.Count; ++i)
            {
                result[i, i] = vec[i];
            }
            return result;
        }

        public static Matrix<double> reshape(Matrix<double> mat, int nRows, int nCols)
        {
            if (mat.ColumnCount * mat.RowCount == nRows * nCols)
            {
                double[] vec = mat.ToColumnWiseArray();
                Matrix<double> reshaped = new DenseMatrix(nRows, nCols);
                for (int i = 0, k = 0; i < nCols; ++i)
                {
                    for (int j = 0; j < nRows; ++j, ++k)
                    {
                        reshaped[j, i] = vec[k];
                    }
                }
                return reshaped;
            }
            else
            {
                throw new ArgumentException("The number of elements before and after reshaping must be the same.");
            }
        }
    }
}
