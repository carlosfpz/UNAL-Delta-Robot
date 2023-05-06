using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;

namespace DeltaRobot_WPF_NetCore.Kinematics
{
    public static class MatrixFunctions
    {
        public static double DegreesToRad(double d_Angle_Degrees)
        {
            double d_Angle_Radians = Math.PI * d_Angle_Degrees / 180.0;
            return d_Angle_Radians;
        }

        public static Matrix CalculateAutoZRotationDeg(double d_RotationDeg)
        {
            //Debug.WriteLine("CalculateAutoZRotation()");

            d_RotationDeg *= -1.0;

            // Creating a Matrix structure.
            Matrix m_RotationMatrix = new Matrix(1, 0, 0, 1, 0, 0); //Identity matrix generation (m11, m12, m21, m22, offsetX, offsetY):
            // 1 0 0
            // 0 1 0
            // 0 0 1

            // Rotate the matrix X degrees about the origin.            
            m_RotationMatrix.Rotate(d_RotationDeg);

            //Aprox result
            m_RotationMatrix.M11 = Math.Round(m_RotationMatrix.M11, 15);
            m_RotationMatrix.M12 = Math.Round(m_RotationMatrix.M12, 15);
            m_RotationMatrix.M21 = Math.Round(m_RotationMatrix.M21, 15);
            m_RotationMatrix.M22 = Math.Round(m_RotationMatrix.M22, 15);

            /*
            //Show result
            Debug.WriteLine("Auto Z Rotation:");
            Debug.WriteLine(m_RotationMatrix.M11 + "   " + m_RotationMatrix.M12 + "   0");
            Debug.WriteLine(m_RotationMatrix.M21 + "   " + m_RotationMatrix.M22 + "   0");
            Debug.WriteLine(m_RotationMatrix.OffsetX + "   " + m_RotationMatrix.OffsetY + "   1");
            */

            return m_RotationMatrix;
        }

        public static Matrix CalculateMatrixInverse(Matrix m_Matrix)
        {
            //Debug.WriteLine("CalculateMatrixInverse()");

            // Checking if myMatrix is invertible.
            if (m_Matrix.HasInverse)
            {
                m_Matrix.Invert();

                //Aprox result
                m_Matrix.M11 = Math.Round(m_Matrix.M11, 15);
                m_Matrix.M12 = Math.Round(m_Matrix.M12, 15);
                m_Matrix.M21 = Math.Round(m_Matrix.M21, 15);
                m_Matrix.M22 = Math.Round(m_Matrix.M22, 15);

                /*
                //Show result
                Debug.WriteLine("Inverted matrix:");
                Debug.WriteLine(m_Matrix.M11 + "   " + m_Matrix.M12 + "   0");
                Debug.WriteLine(m_Matrix.M21 + "   " + m_Matrix.M22 + "   0");
                Debug.WriteLine(m_Matrix.OffsetX + "   " + m_Matrix.OffsetY + "   1");
                */

                // Return the inverted matrix.
                return m_Matrix;
            }
            else
            {
                Debug.WriteLine("ERROR: Matrix is not invertible");
                throw new InvalidOperationException("The matrix is not invertible.");
            }
        }

        public static double RadToDegress(double d_Angle_Radians)
        {
            double d_Angle_Degrees = (180.0 / Math.PI) * d_Angle_Radians;
            return d_Angle_Degrees;
        }

        /*
        public static double[,] CalculateManualRotationZ(double d_Angle_Degrees)
        {
            double d_Angle_Radians = Math.PI * d_Angle_Degrees / 180.0;

            //Debug.WriteLine("Angle rad = " + d_Angle_Radians); 

            double[,] d_Matrix3x3 = new double[3, 3];   //Rows, columns
            
            double d_SinAngle = Math.Round(Math.Sin(d_Angle_Radians), 15);
            double d_CosAngle = Math.Round(Math.Cos(d_Angle_Radians), 15);

            //Matrix rotation calculation
            //Row 0 calculation
            d_Matrix3x3[0, 0] = d_CosAngle;
            d_Matrix3x3[0, 1] = -1.0 * d_SinAngle;
            d_Matrix3x3[0, 2] = 0.0;

            //Row 1 calculation
            d_Matrix3x3[1, 0] = d_SinAngle;
            d_Matrix3x3[1, 1] = d_CosAngle;
            d_Matrix3x3[1, 2] = 0.0;

            //Row 2 calculation
            d_Matrix3x3[2, 0] = 0.0;
            d_Matrix3x3[2, 1] = 0.0;
            d_Matrix3x3[2, 2] = 1.0;

            return d_Matrix3x3;
        }
        */
    }
}
