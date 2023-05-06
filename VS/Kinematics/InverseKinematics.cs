using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;

namespace DeltaRobot_WPF_NetCore.Kinematics
{
    public static class InverseKinematics
    {
        // CFPZ Robot Construido 2021 COMPROBADO OK
        public const double d_RobotParameter_rA = 160.0;    //OK (Radio base fija)
        public const double d_RobotParameter_rB = 56.45;    //OK (Radio base movil)
        public const double d_RobotParameter_L1 = 310.6;    //OK (Longitud brazo pasivo)
        public const double d_RobotParameter_L2 = 147.5;    //OK (Longitud brazo activo)
        public const double d_Robot_ZPos_HomePosition = -182.8832;

        // Posicion efector final cuando los brazos estan a 0°: X = 0, Y = 0, Z = -182.8832mm

        //DISEÑO FINAL
        //L1 + L2 < 458.1mm (Altura maxima que tiene el robot, incluso debe ser menor a ese valor)
        //L1 = Brazos mas largos = 310.6mm (Valor fijo ya que es la longitud que tienen los brazos comprados de la Nacional)
        //L1/2 <= L2 <= 3*L1/4     ->      155.3 <= L1 <= 232.95    (UNAL escogio la medida mas pequeña) = 154mm
        //r = rB = Radio base movil        20mm <= rB <= 40         = 56.45mm (Quedo mas grande que el diseño de la Nacional)
        //R = rA = Radio base fija         96mm <= rA <= 160mm      = ???

        public static double[] d_Alpha_Deg = new double[3];
        public static double[] d_Beta_Deg = new double[3];
        public static double[] d_Gamma_Deg = new double[3];
        public static double[] d_Theta_Deg = new double[3];

        public static (bool, double[]) CalculateInverseKinematics(double[] d_EndEffectorPosition_mm, bool bl_ShowResults)
        {
            if (bl_ShowResults)
            {
                Debug.WriteLine("CalculateInverseKinematics()");
                Debug.WriteLine("IK: Posicion deseada: X = " + d_EndEffectorPosition_mm[0] + " - Y = " + d_EndEffectorPosition_mm[1] + " - Z " + d_EndEffectorPosition_mm[2]);
            }

            double[] d_ActiveJointsAngles_deg = new double[3];
            double[] d_ActiveJointsAngles_rad = new double[3];
            bool bl_Valid_Result_InvKinematics = false;
            double d_TranslateX;
            double d_TranslateY;
            double d_TranslateZ;

            // ****** TOBILLO 1 ******
            d_TranslateX = d_EndEffectorPosition_mm[0] + d_RobotParameter_rB;
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
            d_TranslateX = d_EndEffectorPosition_mm[0] + (d_RobotParameter_rB * Math.Cos(MatrixFunctions.DegreesToRad(120.0)));
            d_TranslateY = d_EndEffectorPosition_mm[1] + (d_RobotParameter_rB * Math.Sin(MatrixFunctions.DegreesToRad(120.0)));
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
            d_TranslateX = d_EndEffectorPosition_mm[0] + (d_RobotParameter_rB * Math.Cos(MatrixFunctions.DegreesToRad(120.0)));
            d_TranslateY = d_EndEffectorPosition_mm[1] - (d_RobotParameter_rB * Math.Sin(MatrixFunctions.DegreesToRad(120.0)));
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
                
                d_Theta_Rad[i] = Math.Asin(f_Ankle123_PosY_REF123 / d_RobotParameter_L1);
                d_Theta_Deg[i] = MatrixFunctions.RadToDegress(d_Theta_Rad[i]);
                //Debug.WriteLine("Theta [rad] = " + d_Theta_Rad[i] + " Theta [deg] = " + d_Theta_Deg[i]);
                                
                d_Num = Math.Pow(f_Ankle123_PosX_REF123, 2.0) + Math.Pow(f_Ankle123_PosY_REF123, 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0) - Math.Pow(d_RobotParameter_L1, 2.0) - Math.Pow(d_RobotParameter_L2, 2.0);
                d_Den = 2.0 * d_RobotParameter_L1 * d_RobotParameter_L2 * Math.Cos(d_Theta_Rad[i]);
                d_Alpha_Rad[i] = Math.Acos(d_Num / d_Den);
                d_Alpha_Deg[i] = MatrixFunctions.RadToDegress(d_Alpha_Rad[i]);
                //Debug.WriteLine("Alpha [rad] = " + d_Alpha_Rad[i] + " Alpha [deg] = " + d_Alpha_Deg[i]);
                                
                if (0.0 <= f_Ankle123_PosX_REF123 && f_Ankle123_PosX_REF123 <= d_RobotParameter_rA)
                {
                    //Debug.WriteLine("Active joint " + (i + 1) + " case 1");

                    d_L[i] = Math.Sqrt(Math.Pow((d_RobotParameter_rA - f_Ankle123_PosX_REF123), 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0));
                    d_Gamma_Rad[i] = -1.0 * Math.Atan(f_Ankle123_PosZ_REF123 / (d_RobotParameter_rA - f_Ankle123_PosX_REF123));
                    d_Beta_Rad[i] = Math.Acos((Math.Pow(d_L[i], 2.0) + Math.Pow(d_RobotParameter_L2, 2.0) - Math.Pow((d_RobotParameter_L1 * Math.Cos(d_Theta_Rad[i])), 2.0)) / (2.0 * d_L[i] * d_RobotParameter_L2));
                    d_ActiveJointsAngles_rad[i] = Math.PI - (d_Gamma_Rad[i] + d_Beta_Rad[i]);

                    bl_Valid_Result_InvKinematics = true;
                }

                else if (f_Ankle123_PosX_REF123 < 0.0)
                {
                    //Debug.WriteLine("Active joint " + (i + 1) + " case 2");

                    d_L[i] = Math.Sqrt(Math.Pow((Math.Abs(f_Ankle123_PosX_REF123) + d_RobotParameter_rA), 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0));
                    d_Gamma_Rad[i] = -1.0 * Math.Atan(f_Ankle123_PosZ_REF123 / (Math.Abs(f_Ankle123_PosX_REF123) + d_RobotParameter_rA));
                    d_Beta_Rad[i] = Math.Acos((Math.Pow(d_L[i], 2.0) + Math.Pow(d_RobotParameter_L2, 2.0) - Math.Pow((d_RobotParameter_L1 * Math.Cos(d_Theta_Rad[i])), 2.0)) / (2.0 * d_L[i] * d_RobotParameter_L2));
                    d_ActiveJointsAngles_rad[i] = Math.PI - (d_Gamma_Rad[i] + d_Beta_Rad[i]);

                    bl_Valid_Result_InvKinematics = true;
                }

                else if (f_Ankle123_PosX_REF123 > d_RobotParameter_rA)
                {
                    //Debug.WriteLine("Active joint " + (i + 1) + " case 3");

                    d_L[i] = Math.Sqrt(Math.Pow((f_Ankle123_PosX_REF123 - d_RobotParameter_rA), 2.0) + Math.Pow(f_Ankle123_PosZ_REF123, 2.0));
                    d_Gamma_Rad[i] = -1.0 * Math.Atan((f_Ankle123_PosX_REF123 - d_RobotParameter_rA) / f_Ankle123_PosZ_REF123);
                    d_Beta_Rad[i] = Math.Acos((Math.Pow(d_L[i], 2.0) + Math.Pow(d_RobotParameter_L2, 2.0) - Math.Pow((d_RobotParameter_L1 * Math.Cos(d_Theta_Rad[i])), 2.0)) / (2.0 * d_L[i] * d_RobotParameter_L2));
                    d_ActiveJointsAngles_rad[i] = (Math.PI / 2.0) - (d_Gamma_Rad[i] + d_Beta_Rad[i]);

                    bl_Valid_Result_InvKinematics = true;
                }

                //Convert to degress
                d_Gamma_Deg[i] = MatrixFunctions.RadToDegress(d_Gamma_Rad[i]);
                d_Beta_Deg[i] = MatrixFunctions.RadToDegress(d_Beta_Rad[i]);
                d_ActiveJointsAngles_deg[i] = MatrixFunctions.RadToDegress(d_ActiveJointsAngles_rad[i]);

                //Show results
                if (bl_ShowResults)
                {
                    Debug.WriteLine("Gamma [rad] = " + d_Gamma_Rad[i] + " Gamma [deg] = " + d_Gamma_Deg[i]);
                    Debug.WriteLine("Beta [rad] = " + d_Beta_Rad[i] + " Beta [deg] = " + d_Beta_Deg[i]);
                    Debug.WriteLine("Active joint angle [rad] = " + d_ActiveJointsAngles_rad[i] + " Active joint angle [deg] = " + d_ActiveJointsAngles_deg[i]);
                }                
            }

            return (bl_Valid_Result_InvKinematics, d_ActiveJointsAngles_deg);
        }
        
    }
}
