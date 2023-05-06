using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
//using System.Threading.Tasks;
using System.Windows;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public static class Buffered_Motion
    {
        //*********************** PUBLIC VARIABLES ******************************
        public static bool bl_MotionInProgress = false;       

        // Measure lapsed time
        public static DateTime StartTime;
        public static DateTime StopTime;
        public static Stopwatch StopWatch;
        public static Stopwatch TotalWatch = new Stopwatch();

        public static bool bl_Robot_Busy = false;   
        public const double d_LinearPath_Time_Resol_s = 0.001;    //Resolucion de la generacion de la trayectoria
      
        // Velocidad inicial ejes
        public const double d_LinearPath_VelX_Initial_mm_s = 0.0;

        //*********************** PRIVATE VARIABLES ******************************       
        private const int i_MaxPointsToDownload = 60; //Download 60 points if fifo level fall below 150                
        private static volatile double[,] d_Buffer_Matrix;
        private static volatile double[,] d_MotorsAnglesSetPoint_Matrix_deg;
        private static volatile double[,] d_MotorsAnglesSetPoint_Matrix_turns;        
        private static volatile int i_TotalMotionRunTime_ms;  //Tiempo total del movimiento en ms        
        private static volatile int i_CurrentMotionTime_ms;        
        private static volatile double[] d_Buffer_MotorsStartPositions_Array_turns = new double[HiCONDLL.MAX_AXIS];

        public static double d_EndEffector_StartPosX_mm;
        public static double d_EndEffector_StartPosY_mm;
        public static double d_EndEffector_StartPosZ_mm;
        public static double d_EndEffector_TargetPosX_mm = 0.0;
        public static double d_EndEffector_TargetPosY_mm = 0.0;
        public static double d_EndEffector_TargetPosZ_mm = -200.0;

        public static void Robot_Linear_Interpolation(double d_EndEffector_TargetPosX_mm, double d_EndEffector_TargetPosY_mm, double d_EndEffector_TargetPosZ_mm, double d_SetPoint_Speed_mm_s, double d_SetPoint_Acceleration_mm_s_2)
        {
            Debug.WriteLine("XYZInterpolatedManualMovement_Click()");

            //GET DESIRED TARGET POSITIONS, SPEED AND ACCELERATION 
            //GUI.GetSetPoints();

            //MotionBasicMovements.i_AxisSelected = Form.cb_AxisSelection.SelectedIndex;    //Get selected axis to move   

            Buffered_Motion.d_EndEffector_TargetPosX_mm = d_EndEffector_TargetPosX_mm;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = d_EndEffector_TargetPosY_mm;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = d_EndEffector_TargetPosZ_mm;
            GUI.d_SetPoint_Speed_mm_s = d_SetPoint_Speed_mm_s;
            GUI.d_SetPoint_Acceleration_mm_s_2 = d_SetPoint_Acceleration_mm_s_2;

            /*
            if (MotionBasicMovements.i_AxisSelected == 3)
            {
                d_SetPoint_IndividualAxis = Buffered_Motion.d_EndEffector_TargetPosX_mm;
            }
            else if (MotionBasicMovements.i_AxisSelected == 4)
            {
                d_SetPoint_IndividualAxis = Buffered_Motion.d_EndEffector_TargetPosY_mm;
            }
            else if (MotionBasicMovements.i_AxisSelected == 5)
            {
                d_SetPoint_IndividualAxis = Buffered_Motion.d_EndEffector_TargetPosZ_mm;
            }
            */

            Debug.WriteLine("d_EndEffector_TargetPosX_mm = " + Buffered_Motion.d_EndEffector_TargetPosX_mm);
            Debug.WriteLine("d_EndEffector_TargetPosY_mm = " + Buffered_Motion.d_EndEffector_TargetPosY_mm);
            Debug.WriteLine("d_EndEffector_TargetPosZ_mm = " + Buffered_Motion.d_EndEffector_TargetPosZ_mm);

            //Check robot limits

            //Check set points
            if (Buffered_Motion.d_EndEffector_TargetPosX_mm < MotionBasicMovements.X_MinPosition_mm || Buffered_Motion.d_EndEffector_TargetPosX_mm > MotionBasicMovements.X_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje X está fuera del espacio de trabajo del robot");
                Debug.WriteLine("ERROR: La posicion deseada del eje X está fuera del espacio de trabajo del robot");
                Debug.WriteLine("d_EndEffector_TargetPosX_mm = " + Buffered_Motion.d_EndEffector_TargetPosX_mm);
                Debug.WriteLine("X_MinPosition_mm = " + MotionBasicMovements.X_MinPosition_mm);
                Debug.WriteLine("X_MaxPosition_mm = " + MotionBasicMovements.X_MaxPosition_mm);
                return;
            }
            else if (Buffered_Motion.d_EndEffector_TargetPosY_mm < MotionBasicMovements.Y_MinPosition_mm || Buffered_Motion.d_EndEffector_TargetPosY_mm > MotionBasicMovements.Y_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje Y está fuera del espacio de trabajo del robot");
                Debug.WriteLine("ERROR: La posicion deseada del eje Y está fuera del espacio de trabajo del robot");
                Debug.WriteLine("d_EndEffector_TargetPosY_mm = " + Buffered_Motion.d_EndEffector_TargetPosY_mm);
                Debug.WriteLine("Y_MinPosition_mm = " + MotionBasicMovements.Y_MinPosition_mm);
                Debug.WriteLine("Y_MaxPosition_mm = " + MotionBasicMovements.Y_MaxPosition_mm);
                return;
            }
            else if (Buffered_Motion.d_EndEffector_TargetPosZ_mm < MotionBasicMovements.Z_MinPosition_mm || Buffered_Motion.d_EndEffector_TargetPosZ_mm > MotionBasicMovements.Z_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje Z está fuera del espacio de trabajo del robot");
                Debug.WriteLine("ERROR: La posicion deseada del eje Z está fuera del espacio de trabajo del robot");
                Debug.WriteLine("d_EndEffector_TargetPosZ_mm = " + Buffered_Motion.d_EndEffector_TargetPosZ_mm);
                Debug.WriteLine("Z_MinPosition_mm = " + MotionBasicMovements.Z_MinPosition_mm);
                Debug.WriteLine("Z_MaxPosition_mm = " + MotionBasicMovements.Z_MaxPosition_mm);

                return;
            }
            else if (GUI.d_SetPoint_Speed_mm_s < GUI.d_Min_SetPoint_Speed_mm_s || GUI.d_SetPoint_Speed_mm_s > GUI.d_Max_SetPoint_Speed_mm_s)
            {
                MessageBox.Show("ERROR: Velocidad deseada fuera de rango");
                Debug.WriteLine("ERROR: Velocidad deseada fuera de rango");
                return;
            }
            else if (GUI.d_SetPoint_Acceleration_mm_s_2 < GUI.d_Min_SetPoint_Acceleration_mm_s_2 || GUI.d_SetPoint_Acceleration_mm_s_2 > GUI.d_Max_SetPoint_Acceleration_mm_s_2)
            {
                MessageBox.Show("ERROR: Aceleración deseada fuera de rango");
                Debug.WriteLine("ERROR: Aceleración deseada fuera de rango");
                return;
            }

            Buffered_Motion.BufferedMovement(false);
        }


        //Execute buffered movement
        public static void BufferedMovement(bool bl_SaveToExcel)
        {
            Debug.WriteLine("BufferedMovement()");

            if (HiCONDLL.vsiStatusIsMoving(-1)) //Axis 0 = still moving
            {
                Debug.WriteLine("BufferedMovement() - HiCONDLL.vsiStatusIsMoving: Axis is still moving");
                return;
            }

            // Verificar que no este ejecutandose algun movimiento            
            if (!bl_MotionInProgress)
            {
                Debug.WriteLine("!bl_MotionInProgress");

                if ((d_EndEffector_StartPosX_mm == d_EndEffector_TargetPosX_mm) && (d_EndEffector_StartPosY_mm == d_EndEffector_TargetPosY_mm) && (d_EndEffector_StartPosZ_mm == d_EndEffector_TargetPosZ_mm))
                {
                    Debug.WriteLine("No se requiere mover el robot");
                    MessageBox.Show("The actual position is the same as the destination position", "Movement information", MessageBoxButton.OK, MessageBoxImage.Information);
                }
                else
                {
                    //Leer posiciones iniciales
                    HiCONDLL.vsiStatusGetAxisPositions(d_Buffer_MotorsStartPositions_Array_turns); //Leer las posiciones de los motores
                    Debug.WriteLine("d_Buffer_MotorsStartPositions_Array_turns[0] = " + d_Buffer_MotorsStartPositions_Array_turns[0]);
                    Debug.WriteLine("d_Buffer_MotorsStartPositions_Array_turns[1] = " + d_Buffer_MotorsStartPositions_Array_turns[1]);
                    Debug.WriteLine("d_Buffer_MotorsStartPositions_Array_turns[2] = " + d_Buffer_MotorsStartPositions_Array_turns[2]);

                    //CALCULAR MOVIMIENTO                                       
                    CalculateRobotBufferedMotion(bl_SaveToExcel); //Calculate d_MotorsAnglesSetPoint_deg matrix (True if you want to save data to excel)

                    // MOVE ROBOT
                    Debug.WriteLine("*** MOVE ROBOT ***");

                    i_CurrentMotionTime_ms = 0;

                    //Get array row lenght (Es -1 debido a que el robot debe arrancar desde la posicion inicial)
                    i_TotalMotionRunTime_ms = d_MotorsAnglesSetPoint_Matrix_deg.GetLength(0) - 1;

                    Debug.WriteLine("i_TotalMotionRunTime_ms = " + i_TotalMotionRunTime_ms + " ms");

                    // Crear buffer: i_MaxPointsToDownload = 60. Buffer -> Filas = Cant de posiciones a descargar, columnas = cant ejes
                    d_Buffer_Matrix = new double[i_MaxPointsToDownload, HiCONDLL.MAX_AXIS];  //Matriz de 2 dimensiones (60 filas, 6 columnas). 

                    //Initialize lapsed time variables
                    StartTime = DateTime.Now;
                    StopWatch = Stopwatch.StartNew();
                    TotalWatch.Start();

                    //Launch buffered motion
                    Debug.WriteLine("d_MotorsAnglesSetPoint_Matrix_turns qty of rows = " + d_MotorsAnglesSetPoint_Matrix_turns.GetLength(0));
                    Debug.WriteLine("d_MotorsAnglesSetPoint_Matrix_turns qty of columns = " + d_MotorsAnglesSetPoint_Matrix_turns.GetLength(1));

                    bl_MotionInProgress = true;   //Variable checked by Timer_ControllerDataExchange_Event -> Activar movimiento usando metodo Buffered_Motion.ExecuteRobotBufferedMotion()                        
                }
            }
            else
            {
                Debug.WriteLine("ERROR: No es posible realizar un nuevo movimiento (Robot en movimiento)");
            }
        }

        public static void CalculateRobotBufferedMotion(bool bl_SaveToExcel)
        {
            Debug.WriteLine("CalculateRobotBufferedMotion()");

            //Configurar valores iniciales
            
            //GET INITIAL POSITIONS (Actual robot position XYZ)
            d_EndEffector_StartPosX_mm = GUI.d_EndEffectorPosition_mm[0];
            d_EndEffector_StartPosY_mm = GUI.d_EndEffectorPosition_mm[1];
            d_EndEffector_StartPosZ_mm = GUI.d_EndEffectorPosition_mm[2];

            Debug.WriteLine("d_EndEffector_StartPosX_mm = " + d_EndEffector_StartPosX_mm);
            Debug.WriteLine("d_EndEffector_StartPosY_mm = " + d_EndEffector_StartPosY_mm);
            Debug.WriteLine("d_EndEffector_StartPosZ_mm = " + d_EndEffector_StartPosZ_mm);
            
            // GENERATE PATH

            // Calcular magnitud del vector            
            double d_A = Math.Pow(d_EndEffector_TargetPosX_mm - d_EndEffector_StartPosX_mm, 2);
            double d_B = Math.Pow(d_EndEffector_TargetPosY_mm - d_EndEffector_StartPosY_mm, 2);
            double d_C = Math.Pow(d_EndEffector_TargetPosZ_mm - d_EndEffector_StartPosZ_mm, 2);
            double Vec_Mag = Math.Sqrt(d_A + d_B + d_C);
            Debug.WriteLine("Vec_Mag = " + Vec_Mag);

            // Calcular angulos del vector(Cosenos directores->Angulos a desplazar desde el vector unitario de cada eje)
            Debug.WriteLine("Cosenos directores");
            double alpha_rad = Math.Acos((d_EndEffector_TargetPosX_mm - d_EndEffector_StartPosX_mm) / Vec_Mag);
            double beta_rad = Math.Acos((d_EndEffector_TargetPosY_mm - d_EndEffector_StartPosY_mm) / Vec_Mag);
            double gamma_rad = Math.Acos((d_EndEffector_TargetPosZ_mm - d_EndEffector_StartPosZ_mm) / Vec_Mag);
            Debug.WriteLine("alpha_rad = " + alpha_rad);
            Debug.WriteLine("beta_rad = " + beta_rad);
            Debug.WriteLine("gamma_rad = " + gamma_rad);

            // ****************DATOS PARA EL CONTROLADOR DE MOVIMIENTO***************                        
            // Calcular tiempo de aceleracion            
            double LinearPath_AccTime_s = (GUI.d_SetPoint_Speed_mm_s - 0) / GUI.d_SetPoint_Acceleration_mm_s_2;
            Debug.WriteLine("LinearPath_AccTime_s = " + LinearPath_AccTime_s);

            // Calcular velocidad maxima de cada eje independientemente            
            double LinearPath_VelX_Max_mm_s = GUI.d_SetPoint_Speed_mm_s * Math.Cos(alpha_rad);
            double LinearPath_VelY_Max_mm_s = GUI.d_SetPoint_Speed_mm_s * Math.Cos(beta_rad);
            double LinearPath_VelZ_Max_mm_s = GUI.d_SetPoint_Speed_mm_s * Math.Cos(gamma_rad);
                        
            //Aproximar velocidad
            LinearPath_VelX_Max_mm_s = Math.Round(LinearPath_VelX_Max_mm_s, 12);
            LinearPath_VelY_Max_mm_s = Math.Round(LinearPath_VelY_Max_mm_s, 12);
            LinearPath_VelZ_Max_mm_s = Math.Round(LinearPath_VelZ_Max_mm_s, 12);

            Debug.WriteLine("LinearPath_VelX_Max_mm_s (Aprox) calculada segun vel GUI = " + LinearPath_VelX_Max_mm_s);
            Debug.WriteLine("LinearPath_VelY_Max_mm_s (Aprox) calculada segun vel GUI = " + LinearPath_VelY_Max_mm_s);
            Debug.WriteLine("LinearPath_VelZ_Max_mm_s (Aprox) calculada segun vel GUI = " + LinearPath_VelZ_Max_mm_s);

            //Detectar si el eje X debe moverse hacia los positivos o hacia los negativos (Velocidad positiva o negativa)
            bool bl_XAxis_MovingToPositive;
            if (LinearPath_VelX_Max_mm_s >= 0.0)
            {
                bl_XAxis_MovingToPositive = true;
                Debug.WriteLine("El movimiento en X es hacia los positivos");
            }
            else
            {
                bl_XAxis_MovingToPositive = false;
                Debug.WriteLine("El movimiento en X es hacia los negativos");
            }            

            // Calcular aceleracion de cada eje independiente
            double LinearPath_AccX_mm_s2 = (LinearPath_VelX_Max_mm_s - 0) / LinearPath_AccTime_s;
            double LinearPath_AccY_mm_s2 = (LinearPath_VelY_Max_mm_s - 0) / LinearPath_AccTime_s;
            double LinearPath_AccZ_mm_s2 = (LinearPath_VelZ_Max_mm_s - 0) / LinearPath_AccTime_s;
            Debug.WriteLine("LinearPath_AccX_mm_s2 calculada segun vel GUI = " + LinearPath_AccX_mm_s2);
            Debug.WriteLine("LinearPath_AccY_mm_s2 calculada segun vel GUI = " + LinearPath_AccY_mm_s2);
            Debug.WriteLine("LinearPath_AccZ_mm_s2 calculada segun vel GUI = " + LinearPath_AccZ_mm_s2);
                        
            //VERIFICAR SI EN EL RECORRIDO SE ALCANZA A ACELERAR COMPLETAMENTE

            // Calcular la posicion media del recorrido en X
            double d_LinearPath_XPosition_Middle_mm = (d_EndEffector_TargetPosX_mm + d_EndEffector_StartPosX_mm) / 2.0;
            Debug.WriteLine("d_LinearPath_XPosition_Middle_mm (Posicion media del recorrido en X) = " + d_LinearPath_XPosition_Middle_mm);

            // Calcular posicion final de X al finalizar aceleracion (Acelerando completamente)
            double LinearPath_XPosition_EndAccel_MaxAcel = d_EndEffector_StartPosX_mm + (d_LinearPath_VelX_Initial_mm_s * LinearPath_AccTime_s) + (0.5 * LinearPath_AccX_mm_s2 * Math.Pow(LinearPath_AccTime_s, 2));
            Debug.WriteLine("LinearPath_XPosition_EndAccel_MaxAcel (Posicion final de X al finalizar aceleracion (Acelerando completamente)) = " + LinearPath_XPosition_EndAccel_MaxAcel);

            bool bl_LinearPath_CanReachMaxVel;  //Indicador de si se alcanza la vel max

            double LinearPath_XPosition_EndAccel_mm;
            double LinearPath_XPosition_StartDeaccel_mm;

            if (Math.Abs(LinearPath_XPosition_EndAccel_MaxAcel) <= Math.Abs(d_LinearPath_XPosition_Middle_mm))
            {
                bl_LinearPath_CanReachMaxVel = true;
                Debug.WriteLine("Si se alcanza la velocidad maxima en el recorrido");

                //Posicion X al finalizar aceleracion
                LinearPath_XPosition_EndAccel_mm = LinearPath_XPosition_EndAccel_MaxAcel;
                Debug.WriteLine("LinearPath_XPosition_EndAccel_mm = " + LinearPath_XPosition_EndAccel_mm);
                
                // Calcular posicion de X al empezar a desacelerar (Acelerando completamente)
                LinearPath_XPosition_StartDeaccel_mm = d_EndEffector_TargetPosX_mm - LinearPath_XPosition_EndAccel_mm + d_EndEffector_StartPosX_mm;
                Debug.WriteLine("LinearPath_XPosition_StartDeaccel_mm = " + LinearPath_XPosition_StartDeaccel_mm);                
            }
            else
            {
                bl_LinearPath_CanReachMaxVel = false;
                Debug.WriteLine("No se alcanza la velocidad maxima en el recorrido");

                //Posicion X al finalizar aceleracion
                LinearPath_XPosition_EndAccel_mm = d_LinearPath_XPosition_Middle_mm;
                Debug.WriteLine("LinearPath_XPosition_EndAccel_mm = " + LinearPath_XPosition_EndAccel_mm);

                // Calcular posicion de X al empezar a desacelerar (Sin acelerar completamente)
                LinearPath_XPosition_StartDeaccel_mm = LinearPath_XPosition_EndAccel_mm;
                Debug.WriteLine("LinearPath_XPosition_StartDeaccel_mm = " + LinearPath_XPosition_StartDeaccel_mm);
            }
            
            // Inicializar variables
            int t = 0;
            double Ctrl_ActualTime_s = 0.0;

            double Ctrl_XPrevious_Vel_mm_s = 0.0;
            double Ctrl_XPrevious_Pos_mm = d_EndEffector_StartPosX_mm;
            double[] Ctrl_XActual_Acc_mm_s2 = new double[100000];
            double[] Ctrl_XActual_Vel_mm_s = new double[100000];
            double[] Ctrl_XActual_Pos_mm = new double[100000];

            double Ctrl_YPrevious_Vel_mm_s = 0.0;
            double Ctrl_YPrevious_Pos_mm = d_EndEffector_StartPosY_mm;
            double[] Ctrl_YActual_Acc_mm_s2 = new double[100000];
            double[] Ctrl_YActual_Vel_mm_s = new double[100000];
            double[] Ctrl_YActual_Pos_mm = new double[100000];

            double Ctrl_ZPrevious_Vel_mm_s = 0.0;
            double Ctrl_ZPrevious_Pos_mm = d_EndEffector_StartPosZ_mm;
            double[] Ctrl_ZActual_Acc_mm_s2 = new double[100000];
            double[] Ctrl_ZActual_Vel_mm_s = new double[100000];
            double[] Ctrl_ZActual_Pos_mm = new double[100000];

            Ctrl_XActual_Vel_mm_s[0] = 0.0;
            Ctrl_YActual_Vel_mm_s[0] = 0.0;
            Ctrl_ZActual_Vel_mm_s[0] = 0.0;
                      
            Ctrl_XActual_Pos_mm[0] = d_EndEffector_StartPosX_mm;
            Ctrl_YActual_Pos_mm[0] = d_EndEffector_StartPosY_mm;
            Ctrl_ZActual_Pos_mm[0] = d_EndEffector_StartPosZ_mm;
                        
            Debug.WriteLine("***************************************** ACELERAR *******************************************");
            
            //********************* EN EL WHILE HAY ERROR *******************                      
            // Realizar aceleracion
            while ( bl_XAxis_MovingToPositive          && (Ctrl_XActual_Pos_mm[t] < LinearPath_XPosition_EndAccel_mm && Ctrl_XActual_Pos_mm[t] < d_LinearPath_XPosition_Middle_mm) ||
                   (bl_XAxis_MovingToPositive == false && (Ctrl_XActual_Pos_mm[t] > LinearPath_XPosition_EndAccel_mm && Ctrl_XActual_Pos_mm[t] > d_LinearPath_XPosition_Middle_mm)))                                     
            {
                // Tiempo
                t++;
                //Debug.WriteLine("t actual = " + t);
                Ctrl_ActualTime_s = (t - 1) / 1000;

                // Grabar velocidades anteriores
                Ctrl_XPrevious_Vel_mm_s = Ctrl_XActual_Vel_mm_s[t - 1];
                Ctrl_YPrevious_Vel_mm_s = Ctrl_YActual_Vel_mm_s[t - 1];
                Ctrl_ZPrevious_Vel_mm_s = Ctrl_ZActual_Vel_mm_s[t - 1];

                // Grabar posiciones anteriores
                Ctrl_XPrevious_Pos_mm = Ctrl_XActual_Pos_mm[t - 1];
                Ctrl_YPrevious_Pos_mm = Ctrl_YActual_Pos_mm[t - 1];
                Ctrl_ZPrevious_Pos_mm = Ctrl_ZActual_Pos_mm[t - 1];

                Ctrl_XActual_Acc_mm_s2[t] = LinearPath_AccX_mm_s2;
                Ctrl_XActual_Vel_mm_s[t] = Ctrl_XPrevious_Vel_mm_s + (Ctrl_XActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                Ctrl_XActual_Pos_mm[t] = Ctrl_XPrevious_Pos_mm + (Ctrl_XPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_XActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));
                
                Ctrl_YActual_Acc_mm_s2[t] = LinearPath_AccY_mm_s2;
                Ctrl_YActual_Vel_mm_s[t] = Ctrl_YPrevious_Vel_mm_s + (Ctrl_YActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                Ctrl_YActual_Pos_mm[t] = Ctrl_YPrevious_Pos_mm + (Ctrl_YPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_YActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                Ctrl_ZActual_Acc_mm_s2[t] = LinearPath_AccZ_mm_s2;
                Ctrl_ZActual_Vel_mm_s[t] = Ctrl_ZPrevious_Vel_mm_s + (Ctrl_ZActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                Ctrl_ZActual_Pos_mm[t] = Ctrl_ZPrevious_Pos_mm + (Ctrl_ZPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_ZActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                //if (bl_LinearPath_CanReachMaxVel == false && (Math.Abs(Ctrl_XActual_Pos_mm[t]) >= Math.Abs(d_LinearPath_XPosition_Middle_mm)))
                if (bl_LinearPath_CanReachMaxVel == false && ((bl_XAxis_MovingToPositive          && (Ctrl_XActual_Pos_mm[t]) >= d_LinearPath_XPosition_Middle_mm) ||
                                                              (bl_XAxis_MovingToPositive == false && (Ctrl_XActual_Pos_mm[t]) <= d_LinearPath_XPosition_Middle_mm)))
                {
                    Debug.WriteLine("Correccion 0 = Al finalizar aceleracion se paso un poco de la velocidad");
                                        
                    Ctrl_XActual_Acc_mm_s2[t] = LinearPath_AccX_mm_s2 * -1.0;
                    Ctrl_XActual_Vel_mm_s[t] = Ctrl_XPrevious_Vel_mm_s + (Ctrl_XActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                    //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                    Ctrl_XActual_Pos_mm[t] = Ctrl_XPrevious_Pos_mm + (Ctrl_XPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_XActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                    Ctrl_YActual_Acc_mm_s2[t] = LinearPath_AccY_mm_s2 * -1.0;
                    Ctrl_YActual_Vel_mm_s[t] = Ctrl_YPrevious_Vel_mm_s + (Ctrl_YActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                    Ctrl_YActual_Pos_mm[t] = Ctrl_YPrevious_Pos_mm + (Ctrl_YPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_YActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                    Ctrl_ZActual_Acc_mm_s2[t] = LinearPath_AccZ_mm_s2 * -1.0;
                    Ctrl_ZActual_Vel_mm_s[t] = Ctrl_ZPrevious_Vel_mm_s + (Ctrl_ZActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                    Ctrl_ZActual_Pos_mm[t] = Ctrl_ZPrevious_Pos_mm + (Ctrl_ZPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_ZActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));                 
                }
                                
                if (bl_LinearPath_CanReachMaxVel && ((bl_XAxis_MovingToPositive          && Ctrl_XActual_Pos_mm[t] > LinearPath_XPosition_EndAccel_mm) ||
                                                     (bl_XAxis_MovingToPositive == false && Ctrl_XActual_Pos_mm[t] < LinearPath_XPosition_EndAccel_mm)))
                {
                    Debug.WriteLine("Correcion 1");

                    Ctrl_XActual_Vel_mm_s[t] = LinearPath_VelX_Max_mm_s;
                    //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                    Ctrl_XActual_Acc_mm_s2[t] = (Ctrl_XActual_Vel_mm_s[t] - Ctrl_XActual_Vel_mm_s[t - 1]) / d_LinearPath_Time_Resol_s;
                    Ctrl_XActual_Pos_mm[t] = Ctrl_XPrevious_Pos_mm + (Ctrl_XPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_XActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                    Ctrl_YActual_Vel_mm_s[t] = LinearPath_VelY_Max_mm_s;
                    Ctrl_YActual_Acc_mm_s2[t] = (Ctrl_YActual_Vel_mm_s[t] - Ctrl_YActual_Vel_mm_s[t - 1]) / d_LinearPath_Time_Resol_s;
                    Ctrl_YActual_Pos_mm[t] = Ctrl_YPrevious_Pos_mm + (Ctrl_YPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_YActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                    Ctrl_ZActual_Vel_mm_s[t] = LinearPath_VelZ_Max_mm_s;
                    Ctrl_ZActual_Acc_mm_s2[t] = (Ctrl_ZActual_Vel_mm_s[t] - Ctrl_ZActual_Vel_mm_s[t - 1]) / d_LinearPath_Time_Resol_s;
                    Ctrl_ZActual_Pos_mm[t] = Ctrl_ZPrevious_Pos_mm + (Ctrl_ZPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_ZActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));
                }
            }

            Debug.WriteLine("t al finalizar aceleracion: " + t);

            if (bl_LinearPath_CanReachMaxVel == true)
            {                
                Debug.WriteLine("********************** VELOCIDAD CRUCERO ***********************");
                                
                while (bl_XAxis_MovingToPositive          && (Ctrl_XActual_Pos_mm[t] < LinearPath_XPosition_StartDeaccel_mm) ||
                       bl_XAxis_MovingToPositive == false && (Ctrl_XActual_Pos_mm[t] > LinearPath_XPosition_StartDeaccel_mm))
                {
                    // Tiempo
                    t++;
                    //Debug.WriteLine("t actual = " + t);
                    Ctrl_ActualTime_s = (t - 1) / 1000;

                    // Grabar velocidades anterior
                    Ctrl_XPrevious_Vel_mm_s = Ctrl_XActual_Vel_mm_s[t - 1];
                    Ctrl_YPrevious_Vel_mm_s = Ctrl_YActual_Vel_mm_s[t - 1];
                    Ctrl_ZPrevious_Vel_mm_s = Ctrl_ZActual_Vel_mm_s[t - 1];

                    // Grabar posiciones anteriores
                    Ctrl_XPrevious_Pos_mm = Ctrl_XActual_Pos_mm[t - 1];
                    Ctrl_YPrevious_Pos_mm = Ctrl_YActual_Pos_mm[t - 1];
                    Ctrl_ZPrevious_Pos_mm = Ctrl_ZActual_Pos_mm[t - 1];
                    
                    Ctrl_XActual_Acc_mm_s2[t] = 0.0;
                    Ctrl_XActual_Vel_mm_s[t] = LinearPath_VelX_Max_mm_s;
                    //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                    Ctrl_XActual_Pos_mm[t] = Ctrl_XPrevious_Pos_mm + (Ctrl_XPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_XActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                    Ctrl_YActual_Acc_mm_s2[t] = 0;
                    Ctrl_YActual_Vel_mm_s[t] = LinearPath_VelY_Max_mm_s;
                    Ctrl_YActual_Pos_mm[t] = Ctrl_YPrevious_Pos_mm + (Ctrl_YPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_YActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                    Ctrl_ZActual_Acc_mm_s2[t] = 0;
                    Ctrl_ZActual_Vel_mm_s[t] = LinearPath_VelZ_Max_mm_s;
                    Ctrl_ZActual_Pos_mm[t] = Ctrl_ZPrevious_Pos_mm + (Ctrl_ZPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_ZActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));
                                        
                    if ((bl_XAxis_MovingToPositive          && Ctrl_XActual_Pos_mm[t] > LinearPath_XPosition_StartDeaccel_mm) ||
                        (bl_XAxis_MovingToPositive == false && Ctrl_XActual_Pos_mm[t] < LinearPath_XPosition_StartDeaccel_mm))
                    {
                        Debug.WriteLine("Correccion 2");

                        Ctrl_XActual_Acc_mm_s2[t] = LinearPath_AccX_mm_s2 * -1;
                        Ctrl_XActual_Vel_mm_s[t] = Ctrl_XPrevious_Vel_mm_s + (Ctrl_XActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                        //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                        Ctrl_XActual_Pos_mm[t] = Ctrl_XPrevious_Pos_mm + (Ctrl_XPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_XActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                        Ctrl_YActual_Acc_mm_s2[t] = LinearPath_AccY_mm_s2 * -1;
                        Ctrl_YActual_Vel_mm_s[t] = Ctrl_YPrevious_Vel_mm_s + (Ctrl_YActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                        Ctrl_YActual_Pos_mm[t] = Ctrl_YPrevious_Pos_mm + (Ctrl_YPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_YActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                        Ctrl_ZActual_Acc_mm_s2[t] = LinearPath_AccZ_mm_s2 * -1;
                        Ctrl_ZActual_Vel_mm_s[t] = Ctrl_ZPrevious_Vel_mm_s + (Ctrl_ZActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                        Ctrl_ZActual_Pos_mm[t] = Ctrl_ZPrevious_Pos_mm + (Ctrl_ZPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_ZActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));
                    }
                }

                Debug.WriteLine("t al finalizar velocidad crucero: " + t);
            }
                        
            Debug.WriteLine("************************************** DESACELERAR ****************************************");
            
            // Realizar desaceleracion
            while (bl_XAxis_MovingToPositive && (Ctrl_XActual_Pos_mm[t] < d_EndEffector_TargetPosX_mm) ||
                   bl_XAxis_MovingToPositive == false && (Ctrl_XActual_Pos_mm[t] > d_EndEffector_TargetPosX_mm))  
            {
                // Tiempo
                t++;
                //Debug.WriteLine("t actual = " + t);
                Ctrl_ActualTime_s = (t - 1) / 1000;

                // Grabar velocidades anterior
                Ctrl_XPrevious_Vel_mm_s = Ctrl_XActual_Vel_mm_s[t - 1];
                Ctrl_YPrevious_Vel_mm_s = Ctrl_YActual_Vel_mm_s[t - 1];
                Ctrl_ZPrevious_Vel_mm_s = Ctrl_ZActual_Vel_mm_s[t - 1];

                // Grabar posiciones anteriores
                Ctrl_XPrevious_Pos_mm = Ctrl_XActual_Pos_mm[t - 1];
                Ctrl_YPrevious_Pos_mm = Ctrl_YActual_Pos_mm[t - 1];
                Ctrl_ZPrevious_Pos_mm = Ctrl_ZActual_Pos_mm[t - 1];

                Ctrl_XActual_Acc_mm_s2[t] = LinearPath_AccX_mm_s2 * -1.0;
                Ctrl_XActual_Vel_mm_s[t] = Ctrl_XPrevious_Vel_mm_s + (Ctrl_XActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                Ctrl_XActual_Pos_mm[t] = Ctrl_XPrevious_Pos_mm + (Ctrl_XPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_XActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                Ctrl_YActual_Acc_mm_s2[t] = LinearPath_AccY_mm_s2 * -1.0;
                Ctrl_YActual_Vel_mm_s[t] = Ctrl_YPrevious_Vel_mm_s + (Ctrl_YActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                Ctrl_YActual_Pos_mm[t] = Ctrl_YPrevious_Pos_mm + (Ctrl_YPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_YActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));

                Ctrl_ZActual_Acc_mm_s2[t] = LinearPath_AccZ_mm_s2 * -1.0;
                Ctrl_ZActual_Vel_mm_s[t] = Ctrl_ZPrevious_Vel_mm_s + (Ctrl_ZActual_Acc_mm_s2[t] * d_LinearPath_Time_Resol_s);
                Ctrl_ZActual_Pos_mm[t] = Ctrl_ZPrevious_Pos_mm + (Ctrl_ZPrevious_Vel_mm_s * d_LinearPath_Time_Resol_s) + (0.5 * Ctrl_ZActual_Acc_mm_s2[t] * Math.Pow(d_LinearPath_Time_Resol_s, 2));
                                
                // Verificar si el eje X ya esta con velocidad 0 o velocidad contraria a la del movimiento -> Finalizar movimiento                               
                if ((bl_XAxis_MovingToPositive && Ctrl_XActual_Vel_mm_s[t] <= 0) || (bl_XAxis_MovingToPositive == false && Ctrl_XActual_Vel_mm_s[t] >= 0))
                {   
                    Debug.WriteLine("Correccion 3: Eje X paso a velocidad contraria al movimiento");                    

                    Ctrl_XActual_Pos_mm[t] = d_EndEffector_TargetPosX_mm;
                    Ctrl_XActual_Vel_mm_s[t] = 0;
                    //Debug.WriteLine("Ctrl_XActual_Vel_mm_s = " + Ctrl_XActual_Vel_mm_s[t]);
                    Ctrl_XActual_Acc_mm_s2[t] = (Ctrl_XActual_Vel_mm_s[t] - Ctrl_XPrevious_Vel_mm_s) / d_LinearPath_Time_Resol_s;

                    Ctrl_YActual_Pos_mm[t] = d_EndEffector_TargetPosY_mm;
                    Ctrl_YActual_Vel_mm_s[t] = 0;
                    Ctrl_YActual_Acc_mm_s2[t] = (Ctrl_YActual_Vel_mm_s[t] - Ctrl_YPrevious_Vel_mm_s) / d_LinearPath_Time_Resol_s;

                    Ctrl_ZActual_Pos_mm[t] = d_EndEffector_TargetPosZ_mm;
                    Ctrl_ZActual_Vel_mm_s[t] = 0;
                    Ctrl_ZActual_Acc_mm_s2[t] = (Ctrl_ZActual_Vel_mm_s[t] - Ctrl_ZPrevious_Vel_mm_s) / d_LinearPath_Time_Resol_s;
                }
            }

            Debug.WriteLine("t al finalizar desaceleracion: " + t);

            int CiclosTotales = t;

            Debug.WriteLine("Ciclos totales (sin tener en cuenta el momento de inicio) = " + CiclosTotales);    

            //Generar columna de tiempos
            double[] d_Tiempo = new double[CiclosTotales + 1];                        

            for (int i = 0; i <= CiclosTotales; i++)
            {
                d_Tiempo[i] = i;
                //Debug.WriteLine("d_Tiempo = " + d_Tiempo[i]);
            }
            
            // Calculate inverse kinematics
            double[] d_DesiredPositionXYZ_Array_mm = new double[3];
                        
            double[,] d_MotorsAngles_Matrix_deg = new double[CiclosTotales + 1, 3];

            double[] d_ActiveJointsAngles_Array_deg = new double[3];
            double[,] d_ActiveJointsAngles_Matrix_deg = new double[CiclosTotales + 1, 3];

            d_MotorsAnglesSetPoint_Matrix_deg = new double[CiclosTotales + 1, 6]; //Matriz de 2 dimensiones (X filas, 6 columnas)            
            d_MotorsAnglesSetPoint_Matrix_turns = new double[CiclosTotales + 1, 6];  //Matriz de 2 dimensiones (X filas, 6 columnas)
            
            //Calculate all position and save results in excel file
            for (int i = 0; i <= CiclosTotales; i++)
            {
                //Debug.WriteLine("****************** POSITION " + i + " *******************");

                //Debug.WriteLine("X = " + excel.d_DesiredPositionXYZ[i, 0] + " Y = " + excel.d_DesiredPositionXYZ[i, 1] + " Z = " + excel.d_DesiredPositionXYZ[i, 2]);

                //Debug.WriteLine("Posicion #: " + i);

                //XYZ POSITIONS
                
                //Save XYZ positions in array
                d_DesiredPositionXYZ_Array_mm[0] = Ctrl_XActual_Pos_mm[i];    //Get X position
                d_DesiredPositionXYZ_Array_mm[1] = Ctrl_YActual_Pos_mm[i];    //Get Y position
                d_DesiredPositionXYZ_Array_mm[2] = Ctrl_ZActual_Pos_mm[i];    //Get Z position

                //ACTIVE JOINT POSITIONS
                bool bl_Valid_Result_InvKinematics;

                //Calculate inverse kinematics                
                (bl_Valid_Result_InvKinematics, d_ActiveJointsAngles_Array_deg) = Kinematics.InverseKinematics.CalculateInverseKinematics(d_DesiredPositionXYZ_Array_mm, false);
                
                //Show invert kinematics result (Active joints positions)
                //Debug.WriteLine("IK: Active Joint 1 [°] = " + d_ActiveJointsAngles_Array_deg[0]);
                //Debug.WriteLine("IK: Active Joint 2 [°] = " + d_ActiveJointsAngles_Array_deg[1]);
                //Debug.WriteLine("IK: Active Joint 3 [°] = " + d_ActiveJointsAngles_Array_deg[2]);

                //Save active joints in matrix
                for (int j = 0; j < 3; j++)
                {
                    d_ActiveJointsAngles_Matrix_deg[i, j] = d_ActiveJointsAngles_Array_deg[j];
                }

                //MOTOR POSITIONS
                //Calculate motors positions
                for (int j = 0; j < 3; j++)
                {
                    d_MotorsAngles_Matrix_deg[i, j] = d_ActiveJointsAngles_Array_deg[j] * MotionControl.d_ReducerRelation;
                }

                //Show motors positions
                //Debug.WriteLine("Motor 1[°] = " + d_MotorsAngles_Matrix_deg[i, 0]);
                //Debug.WriteLine("Motor 2[°] = " + d_MotorsAngles_Matrix_deg[i, 1]); 
                //Debug.WriteLine("Motor 3[°] = " + d_MotorsAngles_Matrix_deg[i, 2]);

                //Move data to global variables
                d_MotorsAnglesSetPoint_Matrix_deg[i, 0] = d_MotorsAngles_Matrix_deg[i, 0];
                d_MotorsAnglesSetPoint_Matrix_deg[i, 1] = d_MotorsAngles_Matrix_deg[i, 1];
                d_MotorsAnglesSetPoint_Matrix_deg[i, 2] = d_MotorsAngles_Matrix_deg[i, 2];
                d_MotorsAnglesSetPoint_Matrix_turns[i, 0] = d_MotorsAnglesSetPoint_Matrix_deg[i, 0] / 360.0;
                d_MotorsAnglesSetPoint_Matrix_turns[i, 1] = d_MotorsAnglesSetPoint_Matrix_deg[i, 1] / 360.0;
                d_MotorsAnglesSetPoint_Matrix_turns[i, 2] = d_MotorsAnglesSetPoint_Matrix_deg[i, 2] / 360.0;

                //Debug.WriteLine("d_MotorsAnglesSetPoint_Matrix_turns[" + i + "," + j + "] = " + d_MotorsAnglesSetPoint_Matrix_turns[i, j] + " = " + d_MotorsAnglesSetPoint_Matrix_turns[i, j] * 360.0 + " deg");                       
            }

            //Grabar datos en excel
            if (bl_SaveToExcel)
            {
                string strDoc = @"C:\Users\carlo\Documents\Sheet1.xlsx";

                Excel excel;
                excel = new Excel();

                //EXCEL COL 1 TO 5: Save result on excel file (XYZ Positions)
                try
                {
                    //excel.ExcelFileWriterArray = Save an entire column
                    excel.ExcelFileWriterArray(strDoc, d_Tiempo, 1, 1, CiclosTotales + 1);  //Fist row to save = 1, fist column to save = 3, Qty data to save = 243     
                    excel.ExcelFileWriterArray(strDoc, Ctrl_XActual_Pos_mm, 1, 3, CiclosTotales + 1);  //Fist row to save = 1, fist column to save = 3, Qty data to save = 243             
                    excel.ExcelFileWriterArray(strDoc, Ctrl_YActual_Pos_mm, 1, 4, CiclosTotales + 1);  //Fist row to save = 1, fist column to save = 4, Qty data to save = 243
                    excel.ExcelFileWriterArray(strDoc, Ctrl_ZActual_Pos_mm, 1, 5, CiclosTotales + 1);  //Fist row to save = 1, fist column to save = 5, Qty data to save = 243
                }
                catch (System.IO.IOException e)
                {
                    Debug.WriteLine("EXCEPTION (ExcelFileWriterArray): Close excel file before execute the instruction");
                }

                Debug.WriteLine("Grabar angulos de los motores en el archivo de excel " + strDoc + " Hoja Trajectory");

                try
                {
                    //EXCEL COL 7 TO 9: Save active joint positions in excel file
                    excel.ExcelFileWriterMatrix(strDoc, "Trajectory", d_ActiveJointsAngles_Matrix_deg, 1, 7, CiclosTotales + 1, 3);  //Fist row to save = 1, fist column to save = 5

                    //EXCEL COL 11 TO 13: Save active joint positions in excel file
                    excel.ExcelFileWriterMatrix(strDoc, "Trajectory", d_MotorsAnglesSetPoint_Matrix_deg, 1, 11, CiclosTotales + 1, 3);  //Fist row to save = 1, fist column to save = 5

                }
                catch (System.IO.IOException e)
                {
                    Debug.WriteLine("EXCEPTION (ExcelFileWriterMatrix): Close excel file before execute the instruction");
                    MessageBox.Show("Please close excel file before execute the instruction", "Excel file error", MessageBoxButton.OK, MessageBoxImage.Information);
                }

                Debug.WriteLine("Fin grabacion excel");
            }            
        }

        public static void ExecuteRobotBufferedMotion()  //Called by timer MotionTimers.Timer_ControllerDataExchange_Event every 5ms
        {
            //Debug.WriteLine("ExecuteRobotBufferedMotion()");

            int currentFillLevel = 0;
            int maxFillLevel = 0;

            HiCONDLL.vsiStatusGetMotionBufferFillLevel(ref currentFillLevel, ref maxFillLevel); //Obtener el estado del buffer

            if (currentFillLevel > 200) //If buffer level is below 200ms, create new motion and download the buffer
            {
                return;
            }
            else
            {
                //Debug.WriteLine("****************** ExecuteRobotBufferedMotion: Create new motion ******************");
            }
                        
            int i_PointsToDownload = 0; //Cantidad de datos a enviar al controlador en este paquete
                        
            //Debug.WriteLine("i_CurrentMotionTime_ms = " + i_CurrentMotionTime_ms + " ms");

            if (i_CurrentMotionTime_ms < i_TotalMotionRunTime_ms) //Not done yet (No ha terminado el movimiento total)            
            {                                
                int i_TotalPointsRemainingToDownload = i_TotalMotionRunTime_ms - i_CurrentMotionTime_ms;
                //Debug.WriteLine("i_PointsRemainingToDownload = " + i_TotalPointsRemainingToDownload);
                                
                if (i_TotalPointsRemainingToDownload > i_MaxPointsToDownload)
                {
                    i_PointsToDownload = i_MaxPointsToDownload;
                }
                else
                {
                    i_PointsToDownload = i_TotalPointsRemainingToDownload;
                }
                
                //Debug.WriteLine("i_PointsToDownload (On this cycle) = " + i_PointsToDownload);
                                
                for (int vector = 0; vector < i_PointsToDownload; vector++)  //For de vector = 0 a vector = 59 or less (Depends on how many points remains to download)
                {
                    for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5            
                    {                       
                        d_Buffer_Matrix[vector, axis] = d_MotorsAnglesSetPoint_Matrix_turns[i_CurrentMotionTime_ms + 1, axis];                       
                    }
                    
                    i_CurrentMotionTime_ms++;   //Tiempo total del movimiento                  
                }
                
                //give the vectors to DLL for download
                //vsiCmdLoadBufferedMotion = Loads a set of buffered motion vectors(1 vector per millisec resolution) to the API.
                //The buffered motion is downloaded to the controller on the next call of vsiCmdDataExchange().
                HiCONDLL.ERROR error = HiCONDLL.vsiCmdLoadBufferedMotion(d_Buffer_Matrix, i_PointsToDownload);

                if (error != HiCONDLL.ERROR.NONE) //error occured
                {
                    string errorMessage;
                    HiCONDLL.vsiAPIGetLastNotification(out errorMessage);
                    //txtError.Text = errorMessage;
                    Debug.WriteLine("vsiAPIGetLastNotification Error: " + errorMessage);
                    bl_MotionInProgress = false;
                }
            }
            else
            {
                bl_MotionInProgress = false;

                // Stop lapsed time
                StopTime = DateTime.Now;
                StopWatch.Stop();
                TotalWatch.Stop();

                //Debug.WriteLine("Last axis 0 position = " + buffer[i_BufferPos, 0]);
                //Debug.WriteLine("buffer[" + vector + "," + axis + "] = " + buffer[vector, axis]);

                TimeSpan elapsed = StopTime.Subtract(StartTime);
                Debug.WriteLine("Total time [s] = " + elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + StopWatch.Elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + TotalWatch.Elapsed.TotalSeconds.ToString("0.000000"));

                Buffered_Motion.bl_Robot_Busy = false;  //Indicar que el robot ya no esta ocupado realizando movimientos
            }
        }

        /*
        public static void ExecuteRobotBufferedMotion()  //Called by timer MotionTimers.Timer_ControllerDataExchange_Event every 5ms
        {
            //Debug.WriteLine("ExecuteRobotBufferedMotion()");

            int currentFillLevel = 0;
            int maxFillLevel = 0;

            HiCONDLL.vsiStatusGetMotionBufferFillLevel(ref currentFillLevel, ref maxFillLevel); //Obtener el estado del buffer

            if (currentFillLevel > 200) //If buffer level is below 200ms, create new motion and download the buffer
            {
                return;
            }
            else
            {
                Debug.WriteLine("****************** ExecuteRobotBufferedMotion: Create new motion ******************");
            }

            //int downLoadCount = 0;
            int i_PointsToDownload = 0; //Cantidad de datos a enviar al controlador en este paquete

            Debug.WriteLine("i_CurrentMotionTime_ms = " + i_CurrentMotionTime_ms + " ms");

            if (i_CurrentMotionTime_ms < i_TotalMotionRunTime_ms) //Not done yet (No ha terminado el movimiento total)
            //if (i_CurrentMotionTime_ms < 60) //Not done yet (No ha terminado el movimiento total)
            {
                Debug.WriteLine("Entro!");

                int i_TotalPointsRemainingToDownload = i_TotalMotionRunTime_ms - i_CurrentMotionTime_ms;
                Debug.WriteLine("i_PointsRemainingToDownload = " + i_TotalPointsRemainingToDownload);

                if (i_TotalPointsRemainingToDownload > i_MaxPointsToDownload)
                {
                    i_PointsToDownload = i_MaxPointsToDownload;
                }
                else
                {
                    i_PointsToDownload = i_TotalPointsRemainingToDownload;
                }

                Debug.WriteLine("i_PointsToDownload (On this cycle) = " + i_PointsToDownload);

                //for (int vector = 0; vector < i_MaxPointsToDownload; vector++)  //For de vector = 0 a vector = 59
                for (int vector = 0; vector < i_PointsToDownload; vector++)  //For de vector = 0 a vector = 59 or less (Depends on how many points remains to download)
                {
                    for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5            
                    {
                        //Because buffer most be in turns we have to convert from degrees to turns
                        //buffer[vector, axis] = d_BufferMove_AxesStartPositions[axis] + (d_MotorsAngles2[i_BufferPos, axis] / 360.0);

                        //***
                        //d_Buffer_Matrix[vector, axis] = d_Buffer_MotorsStartPositions_Array_turns[axis] + (d_MotorsAnglesSetPoint_Matrix_turns[i_CurrentMotionTime_ms, axis]);  
                        d_Buffer_Matrix[vector, axis] = d_MotorsAnglesSetPoint_Matrix_turns[i_CurrentMotionTime_ms + 1, axis];

                        //Debug.WriteLine("d_MotorsAnglesSetPoint_Matrix_turns[" + i_CurrentMotionTime_ms + 1 + ", " + axis + "] = " + d_MotorsAnglesSetPoint_Matrix_turns[i_CurrentMotionTime_ms + 1, axis]);

                        //buffer[vector, axis] = d_MotorsAngles2[i_BufferPos, axis] / 16.0;  //In turns

                        //d_LastMotorAngles_Array_deg[axis] = d_Buffer_Matrix[vector, axis] * 360.0;

                        //buffer[vector, axis] = 0.0;  //Convert data from degrees to turns

                        //Debug.WriteLine("Row" + i_BufferPos);
                        //Debug.WriteLine("Col" + axis);

                        //double dTest = d_MotorsAngles2[i_BufferPos, axis] / 360.0;  //Convert data from degrees to turns

                        //Debug.WriteLine("Buffer = " + buffer[vector, axis]);
                        //Debug.WriteLine("buffer[" + vector + "," + axis + "] = " + buffer[vector, axis]);

                        //buffer[vector, axis] = 0.25 / 1.11;  //Convert data from degrees to turns
                        //buffer[vector, axis] = 0.25;  //Convert data from degrees to turns
                    }
                    //i_BufferPos++;   //Ir corriendo la posicion (fila) de la cantidad total de datos a enviar

                    //i_PointsToDownload++;    //Cantidad de datos a enviar al controlador en este paquete
                    i_CurrentMotionTime_ms++;   //Tiempo total del movimiento

                    
                    //if (i_BufferPos >= d_MotorsAngles2.GetLength(0))
                    //{
                    //    Debug.WriteLine("CORRECCION!!");
                    //    i_BufferPos = d_MotorsAngles2.GetLength(0) - 1;
                    //}                    
                }

                //downLoadCount++;
                //currMotionMilliseconds++;   //Verificar porque el hilo esta cada 10ms no cada ms
                //currMotionMilliseconds += MaxPointsDownload;    //Carga de datos de a 60ms (60 datos es decir el valor cte de MaxPointsDownload)

                
                // Remember the last position we generated
                //for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5
                //{
                    //d_Buffer_MotorsStartPositions_Array_turns[axis] = d_Buffer_Matrix[0, axis];
                //    d_Buffer_MotorsStartPositions_Array_turns[axis] = d_Buffer_Matrix[i_MaxPointsToDownload - 1, axis];
                //}
                

                //give the vectors to DLL for download
                //vsiCmdLoadBufferedMotion = Loads a set of buffered motion vectors(1 vector per millisec resolution) to the API.
                //The buffered motion is downloaded to the controller on the next call of vsiCmdDataExchange().
                HiCONDLL.ERROR error = HiCONDLL.vsiCmdLoadBufferedMotion(d_Buffer_Matrix, i_PointsToDownload);

                if (error != HiCONDLL.ERROR.NONE) //error occured
                {
                    string errorMessage;
                    HiCONDLL.vsiAPIGetLastNotification(out errorMessage);
                    //txtError.Text = errorMessage;
                    Debug.WriteLine("vsiAPIGetLastNotification Error: " + errorMessage);
                    bl_MotionInProgress = false;
                }
            }
            else
            {
                Debug.WriteLine("Salio!");

                bl_MotionInProgress = false;

                // Stop lapsed time
                StopTime = DateTime.Now;
                StopWatch.Stop();
                TotalWatch.Stop();

                //Debug.WriteLine("Last axis 0 position = " + buffer[i_BufferPos, 0]);

                //Debug.WriteLine("buffer[" + vector + "," + axis + "] = " + buffer[vector, axis]);

                TimeSpan elapsed = StopTime.Subtract(StartTime);
                Debug.WriteLine("Total time [s] = " + elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + StopWatch.Elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + TotalWatch.Elapsed.TotalSeconds.ToString("0.000000"));
            }
        }
        */


        /*
        public static void ExecuteTimedBufferedMotion2()  //Called by timer Timer_ControllerDataExchange_Event every 5ms
        {
            //Debug.WriteLine("ExecuteTimedBufferedMotion()");

            //MotionRunTime = 5000;

            int currentFillLevel = 0;
            int maxFillLevel = 0;

            HiCONDLL.vsiStatusGetMotionBufferFillLevel(ref currentFillLevel, ref maxFillLevel);

            if (currentFillLevel > 200) //if buffer level is below 200ms, create new motion and download the buffer
            {
                return;
            }
            else
            {
                //i_PackOfDataToSend++;
                //Debug.WriteLine("***********************************************************************************");
                //Debug.WriteLine("ExecuteTimedBufferedMotion2: Create new motion. Motion # " + i_PackOfDataToSend);
            }

            //Calcular cuantos puntos falta por enviar
            int i_PointsToSend;

            if (i_PointsRemainingToSend > MaxPointsDownload)
            {
                i_PointsToSend = MaxPointsDownload;                
            }
            else
            {
                i_PointsToSend = i_PointsRemainingToSend;
            }

            //Debug.WriteLine("i_PointsToSend = " + i_PointsToSend);

            //Debug.WriteLine("currMotionMilliseconds valor actual desde donde inicia el movimiento = " + currMotionMilliseconds);

            int downLoadCount = 0;

            //Debug.WriteLine("*** MotionRunTime = " + MotionRunTime + " ms");

            //MotionRunTime = Tiempo total del movimiento
            //currMotionMilliseconds = Tiempo actual del movimiento            
            //if (currMotionMilliseconds < 720) //not done yet (No ha terminado el movimiento)
            //if (currMotionMilliseconds < MotionRunTime) //not done yet (No ha terminado el movimiento)
            if (i_PointsRemainingToSend > 0) //not done yet (No ha terminado el movimiento)
            {
                for (int vector = 0; vector < i_PointsToSend; vector++)  //For de 0 a 59 or less (Depende si faltan menos de 60 puntos x enviar)
                {
                    for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5
                    //for (int axis = 0; axis < 1; axis++)
                    {
                        //buffer[vector, axis] = d_BufferMove_AxesStartPositions[axis] + (positionToMove * Multiplier * -2);
                        buffer[vector, axis] = buffer_turns[vector + currMotion_ms, axis];
                        d_LastMotorAngles[axis] = buffer[vector, axis] * 360.0;
                    }
                    downLoadCount++;                    
                }

                //Debug.WriteLine("downLoadCount = " + downLoadCount);

                //currMotionMilliseconds += 72;   //Verificar porque el hilo esta cada 10ms no cada ms
                currMotion_ms += i_PointsToSend;   //Verificar porque el hilo esta cada 10ms no cada ms
                
                //remember the last position we sent
                //for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5
                //{
                //    //d_BufferMove_AxesStartPositions[axis] = buffer[MaxPointsDownload - 1, axis];
                //    //d_BufferMove_AxesStartPositions[axis] = buffer[59, axis];
                //    d_BufferMove_AxesStartPositions[axis] = buffer[i_PointsToSend - 1, axis];
                //}                

                i_PointsRemainingToSend -= i_PointsToSend;
                //Debug.WriteLine("i_PointsRemainingToSend = " + i_PointsRemainingToSend);

                //give the vectors to DLL for download
                //vsiCmdLoadBufferedMotion = Loads a set of buffered motion vectors(1 vector per millisec resolution) to the API.
                //The buffered motion is downloaded to the controller on the next call of vsiCmdDataExchange().
                //HiCONDLL.ERROR error = HiCONDLL.vsiCmdLoadBufferedMotion(buffer, downLoadCount);
                HiCONDLL.ERROR error = HiCONDLL.vsiCmdLoadBufferedMotion(buffer, i_PointsToSend);

                if (error != HiCONDLL.ERROR.NONE) //error occured
                {
                    string errorMessage;
                    HiCONDLL.vsiAPIGetLastNotification(out errorMessage);
                    //txtError.Text = errorMessage;
                    Debug.WriteLine("Error" + errorMessage);
                    bl_MotionInProgress2 = false;
                }               
            }
            else
            {
                Debug.WriteLine("***********************************************************************************");
                Debug.WriteLine("FIN MOVIMIENTO");
                bl_MotionInProgress2 = false;
                //currMotionMilliseconds = 0;
                //Multiplier *= -1;

                // Stop lapsed time
                StopTime = DateTime.Now;
                StopWatch.Stop();
                TotalWatch.Stop();

                //Debug.WriteLine("Last axis 0 position = " + buffer[59, 0]);

                TimeSpan elapsed = StopTime.Subtract(StartTime);
                Debug.WriteLine("Total time [s] = " + elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + StopWatch.Elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + TotalWatch.Elapsed.TotalSeconds.ToString("0.000000"));                
            }
            
            //if (i_PointsRemainingToSend == 0) 
            //{
            //    Debug.WriteLine("***********************************************************************************");
            //    Debug.WriteLine("FIN 2 MOVIMIENTO");

            //    System.Threading.Thread.Sleep(1000);

            //    bl_MotionInProgress2 = false;
            //   //currMotionMilliseconds = 0;
            //    //Multiplier *= -1;

            //    // Stop lapsed time
            //    StopTime = DateTime.Now;
            //    StopWatch.Stop();
            //    TotalWatch.Stop();

            //    //Debug.WriteLine("Last axis 0 position = " + buffer[59, 0]);

            //    TimeSpan elapsed = StopTime.Subtract(StartTime);
            //    Debug.WriteLine("Total time [s] = " + elapsed.TotalSeconds.ToString("0.000000"));
            //    Debug.WriteLine("Total time [s] = " + StopWatch.Elapsed.TotalSeconds.ToString("0.000000"));
            //    Debug.WriteLine("Total time [s] = " + TotalWatch.Elapsed.TotalSeconds.ToString("0.000000"));
            //}            
        }
        */

        /*
        public static void ExecuteTimedBufferedMotion()  //Called by timer Timer_ControllerDataExchange_Event every 5ms
        {
            //Debug.WriteLine("ExecuteTimedBufferedMotion()");

            MotionRunTime = 5000;

            int currentFillLevel = 0;
            int maxFillLevel = 0;

            HiCONDLL.vsiStatusGetMotionBufferFillLevel(ref currentFillLevel, ref maxFillLevel);

            if (currentFillLevel > 200) //if buffer level is below 200ms, create new motion and download the buffer
            {
                return;
            }
            else
            {
                //Debug.WriteLine("Create new motion");
            }

            double counter = 0;
            int downLoadCount = 0;

            //Debug.WriteLine("*** MotionRunTime = " + MotionRunTime + " ms");

            //MotionRunTime = Tiempo total del movimiento
            //currMotionMilliseconds = Tiempo actual del movimiento
            if (currMotionMilliseconds < MotionRunTime) //not done yet (No ha terminado el movimiento)
            {
                for (int vector = 0; vector < MaxPointsDownload; vector++)  //For de 0 a 59
                {
                    if (MotionRunTime - currMotionMilliseconds <= accelTime) //apply deceleration 
                    {
                        velocity -= acceleration;
                        if (velocity < 0)
                        {
                            velocity = 0;
                        }
                    }
                    else if (velocity < maxVelocity) //apply acceleration
                    {
                        velocity += acceleration;
                        if (velocity >= maxVelocity)
                        {
                            velocity = maxVelocity;
                        }

                        if (currMotionMilliseconds >= (MotionRunTime / 2) || velocity == maxVelocity)
                        {
                            accelTime = velocity / acceleration;
                        }
                    }

                    counter += velocity;

                    //Debug.WriteLine("******** downLoadCount = " + downLoadCount + " *********");

                    for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5
                    //for (int axis = 0; axis < 1; axis++)
                    {
                        if (axis == 0)
                        {
                            //buffer most be in turns
                            buffer[vector, axis] = d_BufferMove_AxesStartPositions[axis] + (counter * Multiplier * -2);
                            d_LastMotorAngles[axis] = buffer[vector, axis] * 360.0;
                        }
                        else
                        {
                            //Llenar toda la fila[vector] con los datos de los 6 ejes
                            //buffer most be in turns
                            buffer[vector, axis] = d_BufferMove_AxesStartPositions[axis] + (counter * Multiplier);
                            d_LastMotorAngles[axis] = buffer[vector, axis] * 360.0;
                        }
                        //Debug.WriteLine("Buffer: Vector = " + vector + " Axis = " + axis + " Value = " + buffer[vector, axis]);
                    }

                    downLoadCount++;
                    currMotionMilliseconds++;   //Verificar porque el hilo esta cada 10ms no cada ms
                }

                //remember the last position we generated
                for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)    //For desde 0 hasta 5
                {
                    d_BufferMove_AxesStartPositions[axis] = buffer[MaxPointsDownload - 1, axis];
                }

                //give the vectors to DLL for download
                //vsiCmdLoadBufferedMotion = Loads a set of buffered motion vectors(1 vector per millisec resolution) to the API.
                //The buffered motion is downloaded to the controller on the next call of vsiCmdDataExchange().
                HiCONDLL.ERROR error = HiCONDLL.vsiCmdLoadBufferedMotion(buffer, downLoadCount);

                if (error != HiCONDLL.ERROR.NONE) //error occured
                {
                    string errorMessage;
                    HiCONDLL.vsiAPIGetLastNotification(out errorMessage);
                    //txtError.Text = errorMessage;
                    Debug.WriteLine("Error" + errorMessage);
                    bl_MotionInProgress2 = false;                    
                }
            }
            else
            {
                bl_MotionInProgress2 = false;                
                //currMotionMilliseconds = 0;
                //Multiplier *= -1;

                // Stop lapsed time
                StopTime = DateTime.Now;
                StopWatch.Stop();
                TotalWatch.Stop();

                //Debug.WriteLine("Last axis 0 position = " + buffer[59, 0]);

                TimeSpan elapsed = StopTime.Subtract(StartTime);
                Debug.WriteLine("Total time [s] = " + elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + StopWatch.Elapsed.TotalSeconds.ToString("0.000000"));
                Debug.WriteLine("Total time [s] = " + TotalWatch.Elapsed.TotalSeconds.ToString("0.000000"));
            }
        }
        */
    }
}
