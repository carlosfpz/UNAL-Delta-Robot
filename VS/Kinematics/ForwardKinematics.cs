using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Numerics;
using System.Diagnostics;

namespace DeltaRobot_WPF_NetCore.Kinematics
{
    public static class ForwardKinematics
    {
        public static bool bl_CalculateForwardKinematics = true;
        public static double[] d_EndEffectorPosition_mm = new double[3];

        public static (bool, double[]) CalculateForwardKinematics(double[] d_Arms_Q_Positions_Deg, bool bl_ShowResults)
        {
            if (bl_ShowResults)
            {
                Debug.WriteLine("CalculateForwardKinematics()");
                Debug.WriteLine("FK: Angulos motorreductor = " + d_Arms_Q_Positions_Deg[0] + " " + d_Arms_Q_Positions_Deg[1] + " " + d_Arms_Q_Positions_Deg[2]);
            }
                          
            /*
            //Eliminar singularidad
            if (d_Arms_Q_Positions_Deg[0] == d_Arms_Q_Positions_Deg[1])
            {
                //d_Arms_Q_Positions_Deg[0] = d_Arms_Q_Positions_Deg[0] - 0.000001;
                Debug.WriteLine("ForwardKinematics: Singularity correction 1");
                d_Arms_Q_Positions_Deg[0] = d_Arms_Q_Positions_Deg[0] - 0.01;
            }

            if (d_Arms_Q_Positions_Deg[0] == d_Arms_Q_Positions_Deg[2])
            {
                //d_Arms_Q_Positions_Deg[0] = d_Arms_Q_Positions_Deg[0] - 0.000001;
                Debug.WriteLine("ForwardKinematics: Singularity correction 2");
                d_Arms_Q_Positions_Deg[0] = d_Arms_Q_Positions_Deg[0] - 0.01;
            }

            if (d_Arms_Q_Positions_Deg[1] == d_Arms_Q_Positions_Deg[2])
            {
                //d_Arms_Q_Positions_Deg[1] = d_Arms_Q_Positions_Deg[0] + 0.000001;
                Debug.WriteLine("ForwardKinematics: Singularity correction 3");
                d_Arms_Q_Positions_Deg[1] = d_Arms_Q_Positions_Deg[0] + 0.01;
            } 
            */
                             
            //double[] d_EndEffectorPosition_mm = new double[3];
            bool bl_Valid_Result_FwdKinematics = false;
            double d_TranslateX;
            double d_TranslateY;
            double d_TranslateZ;

            // ****** CADERA 1 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_rA;
            d_TranslateY = 0.0;
            d_TranslateZ = 0.0;

            Matrix4x4 m_Hip1_PosXYZ_REF0;   //Posicion XYZ de la cadera 1 visto desde el marco de referencia del robot
            m_Hip1_PosXYZ_REF0 = new Matrix4x4(1, 0, 0, (float)d_TranslateX, 
                                               0, 1, 0, (float)d_TranslateY, 
                                               0, 0, 1, (float)d_TranslateZ, 
                                               0, 0, 0, 1);
            //Debug.WriteLine("m_Hip1_PosXYZ_REF0 = " + m_Hip1_PosXYZ_REF0);
            

            // ****** RODILLA 1 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_L2 * Math.Cos(MatrixFunctions.DegreesToRad(d_Arms_Q_Positions_Deg[0]));
            d_TranslateY = 0.0;
            d_TranslateZ = -1.0 * InverseKinematics.d_RobotParameter_L2 * Math.Sin(MatrixFunctions.DegreesToRad(d_Arms_Q_Positions_Deg[0]));
            Matrix4x4 m_Knee1_PosXYZ_HIP1;  //Posicion XYZ de la rodilla 1 visto desde la cadera 1 

            m_Knee1_PosXYZ_HIP1 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                0, 1, 0, (float)d_TranslateY,
                                                0, 0, 1, (float)d_TranslateZ,
                                                0, 0, 0, 1);
            //Debug.WriteLine("m_Knee1_PosXYZ_HIP1 = " + m_Knee1_PosXYZ_HIP1);

            Matrix4x4 m_Knee1_PosXYZ_REF0; //Posicion XYZ de la rodilla 1 visto desde el marco de referencia del robot 
            m_Knee1_PosXYZ_REF0 = m_Hip1_PosXYZ_REF0 * m_Knee1_PosXYZ_HIP1;
            //Debug.WriteLine("m_Knee1_PosXYZ_REF0 = " + m_Knee1_PosXYZ_REF0);


            // ****** CADERA 2 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_rA * Math.Cos(MatrixFunctions.DegreesToRad(120.0));
            d_TranslateY = InverseKinematics.d_RobotParameter_rA * Math.Sin(MatrixFunctions.DegreesToRad(120.0));
            d_TranslateZ = 0.0;
            Matrix4x4 m_Hip2_PosXYZ_REF0;   //Posicion XYZ de la cadera 2 visto desde el marco de referencia del robot
            m_Hip2_PosXYZ_REF0 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                               0, 1, 0, (float)d_TranslateY,
                                               0, 0, 1, (float)d_TranslateZ,
                                               0, 0, 0, 1);
            //Debug.WriteLine("m_Hip2_PosXYZ_REF0 = " + m_Hip2_PosXYZ_REF0);

            // ****** RODILLA 2 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_L2 * Math.Cos(MatrixFunctions.DegreesToRad(d_Arms_Q_Positions_Deg[1]));
            d_TranslateY = 0.0;
            d_TranslateZ = -1.0 * InverseKinematics.d_RobotParameter_L2 * Math.Sin(MatrixFunctions.DegreesToRad(d_Arms_Q_Positions_Deg[1]));
            
            Matrix4x4 m_Knee2_PosXYZ_HIP2;   //Posicion XYZ de la rodilla 2 visto desde la cadera 2
            m_Knee2_PosXYZ_HIP2 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                0, 1, 0, (float)d_TranslateY,
                                                0, 0, 1, (float)d_TranslateZ,
                                                0, 0, 0, 1);
            //Debug.WriteLine("m_Knee2_PosXYZ_HIP2 = " + m_Knee2_PosXYZ_HIP2);

            //En la siguiente linea toca poner el signo contrario para hacer la matriz de rotacion en Z (Visual lo hace diferente)
            Matrix4x4 m_ZRotation120Deg = Matrix4x4.CreateRotationZ((float)MatrixFunctions.DegreesToRad(-120.0));   //Create Z 120° rotation matrix
            //Debug.WriteLine("m_ZRotation120Deg = " + m_ZRotation120Deg);
            Matrix4x4 m_Knee2_PosXYZ_REF0; //Posicion XYZ de la rodilla 2 visto desde el marco de referencia del robot 
            m_Knee2_PosXYZ_REF0 = m_Hip2_PosXYZ_REF0 * m_ZRotation120Deg * m_Knee2_PosXYZ_HIP2;
            //Debug.WriteLine("m_Knee2_PosXYZ_REF0 = " + m_Knee2_PosXYZ_REF0);


            // ****** CADERA 3 ******
            //d_TranslateX = InverseKinematics.d_RobotParameter_rA * Math.Cos(MatrixFunctions.DegreesToRad(120.0));
            //d_TranslateY = -1.0 * InverseKinematics.d_RobotParameter_rA * Math.Sin(MatrixFunctions.DegreesToRad(120.0));
            d_TranslateX = InverseKinematics.d_RobotParameter_rA * Math.Cos(MatrixFunctions.DegreesToRad(240.0));
            d_TranslateY = InverseKinematics.d_RobotParameter_rA * Math.Sin(MatrixFunctions.DegreesToRad(240.0));
            d_TranslateZ = 0.0;
            Matrix4x4 m_Hip3_PosXYZ_REF0;   //Posicion XYZ de la cadera 2 visto desde el marco de referencia del robot
            m_Hip3_PosXYZ_REF0 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                               0, 1, 0, (float)d_TranslateY,
                                               0, 0, 1, (float)d_TranslateZ,
                                               0, 0, 0, 1);
            //Debug.WriteLine("m_Hip3_PosXYZ_REF0 = " + m_Hip3_PosXYZ_REF0);

            // ****** RODILLA 3 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_L2 * Math.Cos(MatrixFunctions.DegreesToRad(d_Arms_Q_Positions_Deg[2]));
            d_TranslateY = 0.0;
            d_TranslateZ = -1.0 * InverseKinematics.d_RobotParameter_L2 * Math.Sin(MatrixFunctions.DegreesToRad(d_Arms_Q_Positions_Deg[2]));

            Matrix4x4 m_Knee3_PosXYZ_HIP3;   //Posicion XYZ de la rodilla 3 visto desde la cadera 3
            m_Knee3_PosXYZ_HIP3 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                0, 1, 0, (float)d_TranslateY,
                                                0, 0, 1, (float)d_TranslateZ,
                                                0, 0, 0, 1);
            //Debug.WriteLine("m_Knee3_PosXYZ_HIP3 = " + m_Knee3_PosXYZ_HIP3);

            //En la siguiente linea toca poner el signo contrario para hacer la matriz de rotacion en Z (Visual lo hace diferente)
            Matrix4x4 m_ZRotation240Deg = Matrix4x4.CreateRotationZ((float)MatrixFunctions.DegreesToRad(-240.0));   //Create Z 240° rotation matrix
            //Debug.WriteLine("m_ZRotation240Deg = " + m_ZRotation240Deg);
            Matrix4x4 m_Knee3_PosXYZ_REF0; //Posicion XYZ de la rodilla 3 visto desde el marco de referencia del robot 
            m_Knee3_PosXYZ_REF0 = m_Hip3_PosXYZ_REF0 * m_ZRotation240Deg * m_Knee3_PosXYZ_HIP3;
            //Debug.WriteLine("m_Knee3_PosXYZ_REF0 = " + m_Knee3_PosXYZ_REF0);


            // ****** TOBILLO 1 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_rB;
            d_TranslateY = 0.0;
            d_TranslateZ = 0.0;

            Matrix4x4 m_Ankle1_PosXYZ_ENDEFFECTOR;   //Posicion XYZ del tobillo 1 visto desde el efector final
            m_Ankle1_PosXYZ_ENDEFFECTOR = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                        0, 1, 0, (float)d_TranslateY,
                                                        0, 0, 1, (float)d_TranslateZ,
                                                        0, 0, 0, 1);
            //Debug.WriteLine("m_Ankle1_PosXYZ_ENDEFFECTOR = " + m_Ankle1_PosXYZ_ENDEFFECTOR);

            // ****** TOBILLO 2 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_rB * Math.Cos(MatrixFunctions.DegreesToRad(120.0));
            d_TranslateY = InverseKinematics.d_RobotParameter_rB * Math.Sin(MatrixFunctions.DegreesToRad(120.0));
            d_TranslateZ = 0.0;

            Matrix4x4 m_Ankle2_PosXYZ_ENDEFFECTOR;   //Posicion XYZ del tobillo 2 visto desde el efector final
            m_Ankle2_PosXYZ_ENDEFFECTOR = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                        0, 1, 0, (float)d_TranslateY,
                                                        0, 0, 1, (float)d_TranslateZ,
                                                        0, 0, 0, 1);
            //Debug.WriteLine("m_Ankle2_PosXYZ_ENDEFFECTOR = " + m_Ankle2_PosXYZ_ENDEFFECTOR);

            // ****** TOBILLO 3 ******
            d_TranslateX = InverseKinematics.d_RobotParameter_rB * Math.Cos(MatrixFunctions.DegreesToRad(240.0));
            d_TranslateY = InverseKinematics.d_RobotParameter_rB * Math.Sin(MatrixFunctions.DegreesToRad(240.0));
            d_TranslateZ = 0.0;

            Matrix4x4 m_Ankle3_PosXYZ_ENDEFFECTOR;   //Posicion XYZ del tobillo 3 visto desde el efector final
            m_Ankle3_PosXYZ_ENDEFFECTOR = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                        0, 1, 0, (float)d_TranslateY,
                                                        0, 0, 1, (float)d_TranslateZ,
                                                        0, 0, 0, 1);
            //Debug.WriteLine("m_Ankle3_PosXYZ_ENDEFFECTOR = " + m_Ankle3_PosXYZ_ENDEFFECTOR);


            // ****** POSICION CENTRAL ESFERAS VIRTUALES ******
            // Resta de matrices con funcion de Visual: Matrix4x4 m_Sphere1_PosXYZ_REF0 = Matrix4x4.Subtract(m_Knee1_PosXYZ_REF0, m_Ankle1_PosXYZ_ENDEFFECTOR);   //Centro XYZ de la esfera 1 visto desde el marco de referencia del robot
            Matrix4x4 m_Sphere1_PosXYZ_REF0 = m_Knee1_PosXYZ_REF0 - m_Ankle1_PosXYZ_ENDEFFECTOR;   //Centro XYZ de la esfera 1 visto desde el marco de referencia del robot
            Matrix4x4 m_Sphere2_PosXYZ_REF0 = m_Knee2_PosXYZ_REF0 - m_Ankle2_PosXYZ_ENDEFFECTOR;   //Centro XYZ de la esfera 2 visto desde el marco de referencia del robot
            Matrix4x4 m_Sphere3_PosXYZ_REF0 = m_Knee3_PosXYZ_REF0 - m_Ankle3_PosXYZ_ENDEFFECTOR;   //Centro XYZ de la esfera 3 visto desde el marco de referencia del robot

            //Debug.WriteLine("m_Sphere1_PosXYZ_REF0 = " + m_Sphere1_PosXYZ_REF0);
            //Debug.WriteLine("m_Sphere2_PosXYZ_REF0 = " + m_Sphere2_PosXYZ_REF0);
            //Debug.WriteLine("m_Sphere3_PosXYZ_REF0 = " + m_Sphere3_PosXYZ_REF0);
            
            // ****** CENTROS Y RADIOS DE LAS ESFERAS VIRTUALES
            
            // Centro y radio de la esfera virtual 1
            double d_x1 = (double)m_Sphere1_PosXYZ_REF0.M14; //Capturar valor fila 1, columna 4
            double d_y1 = (double)m_Sphere1_PosXYZ_REF0.M24;
            double d_z1 = (double)m_Sphere1_PosXYZ_REF0.M34;
            double d_r1 = InverseKinematics.d_RobotParameter_L1;

            // Centro y radio de la esfera virtual 2
            double d_x2 = (double)m_Sphere2_PosXYZ_REF0.M14; //Capturar valor fila 1, columna 4
            double d_y2 = (double)m_Sphere2_PosXYZ_REF0.M24;
            double d_z2 = (double)m_Sphere2_PosXYZ_REF0.M34;
            double d_r2 = InverseKinematics.d_RobotParameter_L1;

            // Centro y radio de la esfera virtual 3
            double d_x3 = (double)m_Sphere3_PosXYZ_REF0.M14; //Capturar valor fila 1, columna 4
            double d_y3 = (double)m_Sphere3_PosXYZ_REF0.M24;
            double d_z3 = (double)m_Sphere3_PosXYZ_REF0.M34;
            double d_r3 = InverseKinematics.d_RobotParameter_L1;

            // ******* CALCULOS CINEMATICA DIRECTA ******

            double d_a11 = 2.0 * (d_x3 - d_x1);
            double d_a12 = 2.0 * (d_y3 - d_y1);
            double d_a13 = 2.0 * (d_z3 - d_z1);

            if (d_a13 == 0)
            {
                Debug.WriteLine("FK: Singularidad con a13 = 0, z3 = %f, z1 = %f', z3, z1");
                bl_Valid_Result_FwdKinematics = false;
            }
                
            double d_a21 = 2 * (d_x3 - d_x2);
            double d_a22 = 2 * (d_y3 - d_y2);
            double d_a23 = 2 * (d_z3 - d_z2);

            if (d_a23 == 0)
            {
                Debug.WriteLine("FK: Singularidad con a23 = 0, z3 = %f, z2 = %f', z3, z2");
                bl_Valid_Result_FwdKinematics = false;
            }

            double d_a1 = (d_a11 / d_a13) - (d_a21 / d_a23);

            if (d_a1 == 0)
            {
                Debug.WriteLine("FK: Singularidad con a1 = 0, a13 = %f, a23 = %f', a13, a23");
                bl_Valid_Result_FwdKinematics = false;
            }

            double d_a2 = (d_a12 / d_a13) - (d_a22 / d_a23);

            double d_b1 = Math.Pow(d_r1, 2) - Math.Pow(d_r3, 2) - Math.Pow(d_x1, 2) - Math.Pow(d_y1, 2) - Math.Pow(d_z1, 2) + Math.Pow(d_x3, 2) + Math.Pow(d_y3, 2) + Math.Pow(d_z3, 2);
            double d_b2 = Math.Pow(d_r2, 2) - Math.Pow(d_r3, 2) - Math.Pow(d_x2, 2) - Math.Pow(d_y2, 2) - Math.Pow(d_z2, 2) + Math.Pow(d_x3, 2) + Math.Pow(d_y3, 2) + Math.Pow(d_z3, 2);
            
            double d_a3 = (d_b2 / d_a23) - (d_b1 / d_a13);
            double d_a4 = -d_a2 / d_a1;
            double d_a5 = -d_a3 / d_a1;
            double d_a6 = (-d_a21 * d_a4 - d_a22) / d_a23;
            double d_a7 = (d_b2 - d_a21 * d_a5) / d_a23;

            double d_a = Math.Pow(d_a4, 2) + 1 + Math.Pow(d_a6, 2);

            if (d_a == 0)
            {
                Debug.WriteLine("FK: Singularidad con a1 = 0, a4 = %f, a6 = %f", d_a4, d_a6);
                bl_Valid_Result_FwdKinematics = false;
            }
                
            double d_b = 2.0 * d_a4 * (d_a5 - d_x1) - 2.0 * d_y1 + 2.0 * d_a6 * (d_a7 - d_z1);
            double d_c = d_a5 * (d_a5 - 2.0 * d_x1) + d_a7 * (d_a7 - 2.0 * d_z1) + Math.Pow(d_x1, 2) + Math.Pow(d_y1, 2) + Math.Pow(d_z1, 2) - Math.Pow(d_r1, 2);

            double d_yPos = (-d_b + (Math.Sqrt(Math.Pow(d_b, 2) - 4.0 * d_a * d_c))) / (2.0 * d_a);
            double d_yNeg = (-d_b - (Math.Sqrt(Math.Pow(d_b, 2) - 4.0 * d_a * d_c))) / (2.0 * d_a);
            
            double d_xPos = d_a4 * d_yPos + d_a5;
            double d_xNeg = d_a4 * d_yNeg + d_a5;

            double d_zPos = d_a6 * d_yPos + d_a7;
            double d_zNeg = d_a6 * d_yNeg + d_a7;

            // Buscar respuesta valida
            if (d_zPos < 0.0 && d_zNeg < 0)
            {
                //Debug.WriteLine("FK: ADVERTENCIA: Existen 2 respuestas validas");
                d_EndEffectorPosition_mm[0] = d_xPos;
                d_EndEffectorPosition_mm[1] = d_yPos;
                d_EndEffectorPosition_mm[2] = d_zPos;
            }
            else if (d_zPos < 0)
            {
                //Debug.WriteLine("FK: Respuesta positiva es valida");
                d_EndEffectorPosition_mm[0] = d_xPos;
                d_EndEffectorPosition_mm[1] = d_yPos;
                d_EndEffectorPosition_mm[2] = d_zPos;
            }
            else if (d_zNeg < 0)
            {
                //Debug.WriteLine("FK: Respuesta negativa es valida");
                d_EndEffectorPosition_mm[0] = d_xNeg;
                d_EndEffectorPosition_mm[1] = d_yNeg;
                d_EndEffectorPosition_mm[2] = d_zNeg;
            }
            else
            {
                Debug.WriteLine("FK: Ninguna respuesta es valida (Posicion: 0, 0, Z = home)");
                
                d_EndEffectorPosition_mm[0] = 0.0000;
                d_EndEffectorPosition_mm[1] = 0.0000;
                d_EndEffectorPosition_mm[2] = InverseKinematics.d_Robot_ZPos_HomePosition;
            }

            if (bl_ShowResults)
            {
                Debug.WriteLine("FK: Respuesta encontrada -> X = " + d_EndEffectorPosition_mm[0] + "mm Y = " + d_EndEffectorPosition_mm[1] + "mm Z = " + d_EndEffectorPosition_mm[2] + "mm");
            }

            bl_Valid_Result_FwdKinematics = true;
                       
            return (bl_Valid_Result_FwdKinematics, d_EndEffectorPosition_mm);
        }

        public static bool CalculatePasiveAngles()
        {
            bool bl_Valid_Result_InvKinematics = false;

            double d_TranslateX;
            double d_TranslateY;
            double d_TranslateZ;

            // ****** TOBILLO 1 ******
            d_TranslateX = d_EndEffectorPosition_mm[0] + InverseKinematics.d_RobotParameter_rB;
            d_TranslateY = d_EndEffectorPosition_mm[1];
            d_TranslateZ = d_EndEffectorPosition_mm[2];

            Matrix4x4 m_Ankle1_PosXYZ_REF0;   //Posicion XYZ del tobillo 1 visto desde el marco de referencia del robot
            m_Ankle1_PosXYZ_REF0 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                                 0, 1, 0, (float)d_TranslateY,
                                                 0, 0, 1, (float)d_TranslateZ,
                                                 0, 0, 0, 1);
            //Debug.WriteLine("m_Ankle1_REF0 = " + m_Ankle1_REF0);

            Matrix4x4 m_Ankle1_REF1 = m_Ankle1_PosXYZ_REF0;   //Posicion XYZ del tobillo 1 visto desde el plano de la pierna 1
            //Debug.WriteLine("m_Ankle1_REF1 = " + m_Ankle1_REF1);


            // ****** TOBILLO 2 ******
            d_TranslateX = d_EndEffectorPosition_mm[0] + (InverseKinematics.d_RobotParameter_rB * Math.Cos(MatrixFunctions.DegreesToRad(120.0)));
            d_TranslateY = d_EndEffectorPosition_mm[1] + (InverseKinematics.d_RobotParameter_rB * Math.Sin(MatrixFunctions.DegreesToRad(120.0)));
            d_TranslateZ = d_EndEffectorPosition_mm[2];

            Matrix4x4 m_Ankle2_REF0;   //Posicion XYZ del tobillo 2 visto desde el marco de referencia del robot
            m_Ankle2_REF0 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                          0, 1, 0, (float)d_TranslateY,
                                          0, 0, 1, (float)d_TranslateZ,
                                          0, 0, 0, 1);
            //Debug.WriteLine("m_Ankle2_REF0 = " + m_Ankle2_REF0);

            //En la siguiente linea toca poner el signo contrario para hacer la matriz de rotacion en Z (Visual lo hace diferente)
            Matrix4x4 m_ZRotation_Minus120Deg = Matrix4x4.CreateRotationZ((float)MatrixFunctions.DegreesToRad(120.0));   //Create Z -120° rotation matrix
            //Debug.WriteLine("m_ZRotation_Minus120Deg = " + m_ZRotation_Minus120Deg);
            Matrix4x4 m_Ankle2_REF2; //Posicion XYZ del tobillo 2 visto desde el plano de la pierna 2
            m_Ankle2_REF2 = m_ZRotation_Minus120Deg * m_Ankle2_REF0;
            //Debug.WriteLine("m_Ankle2_REF2 = " + m_Ankle2_REF2);


            // ****** TOBILLO 3 ******
            d_TranslateX = d_EndEffectorPosition_mm[0] + (InverseKinematics.d_RobotParameter_rB * Math.Cos(MatrixFunctions.DegreesToRad(120.0)));
            d_TranslateY = d_EndEffectorPosition_mm[1] - (InverseKinematics.d_RobotParameter_rB * Math.Sin(MatrixFunctions.DegreesToRad(120.0)));
            d_TranslateZ = d_EndEffectorPosition_mm[2];

            Matrix4x4 m_Ankle3_REF0;   //Posicion XYZ del tobillo 3 visto desde el marco de referencia del robot
            m_Ankle3_REF0 = new Matrix4x4(1, 0, 0, (float)d_TranslateX,
                                          0, 1, 0, (float)d_TranslateY,
                                          0, 0, 1, (float)d_TranslateZ,
                                          0, 0, 0, 1);
            //Debug.WriteLine("m_Ankle3_REF0 = " + m_Ankle3_REF0);

            //En la siguiente linea toca poner el signo contrario para hacer la matriz de rotacion en Z (Visual lo hace diferente)
            Matrix4x4 m_ZRotation_Minus240Deg = Matrix4x4.CreateRotationZ((float)MatrixFunctions.DegreesToRad(240.0));   //Create Z -240° rotation matrix
            //Debug.WriteLine("m_ZRotation_Minus240Deg = " + m_ZRotation_Minus240Deg);
            Matrix4x4 m_Ankle3_REF3; //Posicion XYZ del tobillo 3 visto desde el plano de la pierna 3
            m_Ankle3_REF3 = m_ZRotation_Minus240Deg * m_Ankle3_REF0;
            //Debug.WriteLine("m_Ankle3_REF3 = " + m_Ankle3_REF3);

            //***********************************************************
            //INVERSE KINEMATICS CALCULATION
            //Debug.WriteLine("******** INVERSE KINEMATICS CALCULATION *********");

            double[] d_Theta_Rad = new double[3];
            //double[] d_Theta_Deg = new double[3];
            double[] d_Alpha_Rad = new double[3];
            //double[] d_Alpha_Deg = new double[3];
            double[] d_Gamma_Rad = new double[3];
            //double[] d_Gamma_Deg = new double[3];
            double[] d_Beta_Rad = new double[3];
            //double[] d_Beta_Deg = new double[3];
            double[] d_L = new double[3];
            double d_Num;
            double d_Den;

            for (int i = 0; i < 3; i++)
            {
                Matrix4x4 m_Ankle123_REF123;    //Matriz temporal a analizar (Contiene las posiciones del tobillo a analizar con respecto al plano de su propia pierna)
                m_Ankle123_REF123 = new Matrix4x4(1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0,
                                                  0, 0, 0, 1);

                if (i == 0)
                {
                    m_Ankle123_REF123 = m_Ankle1_REF1;
                }
                else if (i == 1)
                {
                    m_Ankle123_REF123 = m_Ankle2_REF2;
                }
                else if (i == 2)
                {
                    m_Ankle123_REF123 = m_Ankle3_REF3;
                }

                float f_Ankle123_PosX_REF123 = m_Ankle123_REF123.M14;   //Capturar la posicion X del tobillo a analizar con respecto al plano de su propia pierna
                float f_Ankle123_PosY_REF123 = m_Ankle123_REF123.M24;   //Capturar la posicion Y del tobillo a analizar con respecto al plano de su propia pierna
                float f_Ankle123_PosZ_REF123 = m_Ankle123_REF123.M34;   //Capturar la posicion Z del tobillo a analizar con respecto al plano de su propia pierna

                //Debug.WriteLine("********** Tobillo " + (i + 1) + " ************");
                //Debug.WriteLine("Posicion tobillo " + (i + 1) + " con respecto al plano de la pierna " + (i + 1) + ": X = " + f_Ankle123_PosX_REF123 + " Y = " + f_Ankle123_PosY_REF123 + " Z = " + f_Ankle123_PosZ_REF123);                

                d_Theta_Rad[i] = Math.Asin(f_Ankle123_PosY_REF123 / InverseKinematics.d_RobotParameter_L1);
                InverseKinematics.d_Theta_Deg[i] = MatrixFunctions.RadToDegress(d_Theta_Rad[i]);
                //Debug.WriteLine("Theta [rad] = " + d_Theta_Rad[i] + " Theta [deg] = " + d_Theta_Deg[i]);

                d_Num = Math.Pow(f_Ankle123_PosX_REF123, 2.0) + Math.Pow(f_Ankle123_PosY_REF123, 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0) - Math.Pow(InverseKinematics.d_RobotParameter_L1, 2.0) - Math.Pow(InverseKinematics.d_RobotParameter_L2, 2.0);
                d_Den = 2.0 * InverseKinematics.d_RobotParameter_L1 * InverseKinematics.d_RobotParameter_L2 * Math.Cos(d_Theta_Rad[i]);
                d_Alpha_Rad[i] = Math.Acos(d_Num / d_Den);
                InverseKinematics.d_Alpha_Deg[i] = MatrixFunctions.RadToDegress(d_Alpha_Rad[i]);
                //Debug.WriteLine("Alpha [rad] = " + d_Alpha_Rad[i] + " Alpha [deg] = " + d_Alpha_Deg[i]);

                if (0.0 <= f_Ankle123_PosX_REF123 && f_Ankle123_PosX_REF123 <= InverseKinematics.d_RobotParameter_rA)
                {
                    //Debug.WriteLine("Active joint " + (i + 1) + " case 1");

                    d_L[i] = Math.Sqrt(Math.Pow((InverseKinematics.d_RobotParameter_rA - f_Ankle123_PosX_REF123), 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0));
                    d_Gamma_Rad[i] = -1.0 * Math.Atan(f_Ankle123_PosZ_REF123 / (InverseKinematics.d_RobotParameter_rA - f_Ankle123_PosX_REF123));
                    d_Beta_Rad[i] = Math.Acos((Math.Pow(d_L[i], 2.0) + Math.Pow(InverseKinematics.d_RobotParameter_L2, 2.0) - Math.Pow((InverseKinematics.d_RobotParameter_L1 * Math.Cos(d_Theta_Rad[i])), 2.0)) / (2.0 * d_L[i] * InverseKinematics.d_RobotParameter_L2));
                    //d_ActiveJointsAngles_rad[i] = Math.PI - (d_Gamma_Rad[i] + d_Beta_Rad[i]);

                    bl_Valid_Result_InvKinematics = true;
                }

                else if (f_Ankle123_PosX_REF123 < 0.0)
                {
                    //Debug.WriteLine("Active joint " + (i + 1) + " case 2");

                    d_L[i] = Math.Sqrt(Math.Pow((Math.Abs(f_Ankle123_PosX_REF123) + InverseKinematics.d_RobotParameter_rA), 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0));
                    d_Gamma_Rad[i] = -1.0 * Math.Atan(f_Ankle123_PosZ_REF123 / (Math.Abs(f_Ankle123_PosX_REF123) + InverseKinematics.d_RobotParameter_rA));
                    d_Beta_Rad[i] = Math.Acos((Math.Pow(d_L[i], 2.0) + Math.Pow(InverseKinematics.d_RobotParameter_L2, 2.0) - Math.Pow((InverseKinematics.d_RobotParameter_L1 * Math.Cos(d_Theta_Rad[i])), 2.0)) / (2.0 * d_L[i] * InverseKinematics.d_RobotParameter_L2));
                    //d_ActiveJointsAngles_rad[i] = Math.PI - (d_Gamma_Rad[i] + d_Beta_Rad[i]);

                    bl_Valid_Result_InvKinematics = true;
                }

                else if (f_Ankle123_PosX_REF123 > InverseKinematics.d_RobotParameter_rA)
                {
                    //Debug.WriteLine("Active joint " + (i + 1) + " case 3");

                    d_L[i] = Math.Sqrt(Math.Pow((f_Ankle123_PosX_REF123 - InverseKinematics.d_RobotParameter_rA), 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0));
                    d_Gamma_Rad[i] = -1.0 * Math.Atan((f_Ankle123_PosX_REF123 - InverseKinematics.d_RobotParameter_rA) / f_Ankle123_PosZ_REF123);
                    d_Beta_Rad[i] = Math.Acos((Math.Pow(d_L[i], 2.0) + Math.Pow(InverseKinematics.d_RobotParameter_L2, 2.0) - Math.Pow((InverseKinematics.d_RobotParameter_L1 * Math.Cos(d_Theta_Rad[i])), 2.0)) / (2.0 * d_L[i] * InverseKinematics.d_RobotParameter_L2));
                    //d_ActiveJointsAngles_rad[i] = (Math.PI / 2.0) - (d_Gamma_Rad[i] + d_Beta_Rad[i]);

                    bl_Valid_Result_InvKinematics = true;
                }

                //Convert to degress
                InverseKinematics.d_Gamma_Deg[i] = MatrixFunctions.RadToDegress(d_Gamma_Rad[i]);
                InverseKinematics.d_Beta_Deg[i] = MatrixFunctions.RadToDegress(d_Beta_Rad[i]);
                //d_ActiveJointsAngles_deg[i] = MatrixFunctions.RadToDegress(d_ActiveJointsAngles_rad[i]);

                /*
                //Show results
                if (bl_ShowResults)
                {
                    Debug.WriteLine("Gamma [rad] = " + d_Gamma_Rad[i] + " Gamma [deg] = " + d_Gamma_Deg[i]);
                    Debug.WriteLine("Beta [rad] = " + d_Beta_Rad[i] + " Beta [deg] = " + d_Beta_Deg[i]);
                    Debug.WriteLine("Active joint angle [rad] = " + d_ActiveJointsAngles_rad[i] + " Active joint angle [deg] = " + d_ActiveJointsAngles_deg[i]);
                }
                */
            }

            return bl_Valid_Result_InvKinematics;
        }
    }
}
