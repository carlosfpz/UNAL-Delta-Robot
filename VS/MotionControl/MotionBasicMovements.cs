using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public class MotionBasicMovements
    {
        //*********************** PUBLIC VARIABLES ******************************
        public static int i_AxisSelected = 0;
        public static int i_AxisSelected_Keyboard = 0;

        //*********************** PRIVATE VARIABLES ******************************

        private static HiCONDLL.DeviceStatus newDeviceStatus = new HiCONDLL.DeviceStatus();

        //Max axes positions
        public const double X_MinPosition_mm = -125.0;
        public const double X_MaxPosition_mm = 125.0;
        public const double Y_MinPosition_mm = -125.0;
        public const double Y_MaxPosition_mm = 125.0;
        public const double Z_MinPosition_mm = -325.0;
        public const double Z_MaxPosition_mm = -180.0;

        //Max q1 positions
        public const double Arm1_Q1_MinPosition_Deg = -12.5;
        public const double Arm1_Q1_MaxPosition_Deg = 50.0;
        public const double Arm2_Q1_MinPosition_Deg = -12.5;
        public const double Arm2_Q1_MaxPosition_Deg = 50.0;
        public const double Arm3_Q1_MinPosition_Deg = -12.5;
        public const double Arm3_Q1_MaxPosition_Deg = 50.0;

        private Thread bufferedMotionThread;    //Hilo de ejecucion de movimientos con buffer     

        //public static void JOGPosMove()
        public void JOGPosMove()
        {
            GUI.GetSetPoints();

            // Check q1 limits
            if (MotionBasicMovements.i_AxisSelected == 0 && GUI.d_Arms_Q_Positions_Deg[0] >= Arm1_Q1_MaxPosition_Deg
                || MotionBasicMovements.i_AxisSelected == 1 && GUI.d_Arms_Q_Positions_Deg[1] >= Arm2_Q1_MaxPosition_Deg
                || MotionBasicMovements.i_AxisSelected == 2 && GUI.d_Arms_Q_Positions_Deg[2] >= Arm3_Q1_MaxPosition_Deg)
            {
                Debug.WriteLine("JOGPosMove: Software limit exceeded");
                JOGPosStop();
            }
            else
            {
                if (MotionBasicMovements.i_AxisSelected >= 0 && MotionBasicMovements.i_AxisSelected <= 2)   //Movimiento articulaciones activas
                {
                    HiCONDLL.vsiCmdExecuteMove(MotionBasicMovements.i_AxisSelected,
                                               9999,
                                               GUI.d_JOG_Axes_SetPoint_Speed_rev_min,
                                               GUI.d_JOG_Axes_SetPoint_Acceleration_rev_min_2,
                                               HiCONDLL.MoveType.ABSOLUTE,
                                               9999);
                }
                else //Movimiento lineal X, Y, Z
                {
                    //GET INITIAL POSITIONS (Actual robot position XYZ)
                    
                    if (MotionBasicMovements.i_AxisSelected == 3)    //Mover eje X
                    {
                        Debug.WriteLine("JOGPosMove: Move end effector X+");
                        Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] + 1.0;
                        Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1];
                        Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2];
                    }

                    else if (MotionBasicMovements.i_AxisSelected == 4)    //Mover eje Y
                    {
                        Debug.WriteLine("JOGPosMove: Move end effector Y+");
                        Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] + 0.000000000001; //Movimiento adicional en X para que efectue el movimiento del otro eje
                        Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1] + 1.0;
                        Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2];         
                    }

                    else if (MotionBasicMovements.i_AxisSelected == 5)    //Mover eje Z
                    {
                        Debug.WriteLine("JOGPosMove: Move end effector Z+");
                        Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] + 0.000000000001; //Movimiento adicional en X para que efectue el movimiento del otro eje
                        Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1];
                        Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2] + 1.0;                       
                    }

                    Debug.WriteLine(Buffered_Motion.d_EndEffector_TargetPosX_mm);
                    Debug.WriteLine(Buffered_Motion.d_EndEffector_TargetPosY_mm);
                    Debug.WriteLine(Buffered_Motion.d_EndEffector_TargetPosZ_mm);

                    /*
                    GUI.d_SetPoint_Speed_mm_s = 50.0;
                    GUI.d_SetPoint_Acceleration_mm_s_2 = 50.0;
                    Buffered_Motion.bl_Robot_Busy = true;
                    Buffered_Motion.BufferedMovement(false);
                    while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                    { }
                    */

                    GUI.d_SetPoint_Speed_mm_s = 100.0;
                    GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;

                    var motion_Thread = new Motion_Thread();

                    bufferedMotionThread = new Thread(new ThreadStart(motion_Thread.BufferedMotionThread));   //Crear hilo adicional para realizar movimiento con buffer
                    bufferedMotionThread.Start();   //Ejecutar movimiento en otro hilo  
                }
            }
        }

        public static void JOGPosMove_Keyboard()
        {
            GUI.GetSetPoints();

            //Debug.WriteLine("JOGPosMove_Keyboard");

            // Check q1 limits
            if (MotionBasicMovements.i_AxisSelected_Keyboard == 0 && GUI.d_Arms_Q_Positions_Deg[0] >= Arm1_Q1_MaxPosition_Deg
                || MotionBasicMovements.i_AxisSelected_Keyboard == 1 && GUI.d_Arms_Q_Positions_Deg[1] >= Arm2_Q1_MaxPosition_Deg
                || MotionBasicMovements.i_AxisSelected_Keyboard == 2 && GUI.d_Arms_Q_Positions_Deg[2] >= Arm3_Q1_MaxPosition_Deg)
            {
                Debug.WriteLine("JOGPosMove_Keyboard: Software limit exceeded");
                JOGPosStop_Keyboard();
            }
            else
            {
                HiCONDLL.vsiCmdExecuteMove(MotionBasicMovements.i_AxisSelected_Keyboard,
                                           9999,
                                           GUI.d_JOG_Axes_SetPoint_Speed_rev_min,
                                           GUI.d_JOG_Axes_SetPoint_Acceleration_rev_min_2,
                                           HiCONDLL.MoveType.ABSOLUTE,
                                           9999);
            }            
        }

        public static void JOGPosStop()
        {
            //Debug.WriteLine("Released");            
            uint sequenceID = 0;
            if (HiCONDLL.vsiStatusGetMotionSequenceID(MotionBasicMovements.i_AxisSelected, ref sequenceID) != HiCONDLL.ERROR.NONE)
            {
                return;
            }

            if (sequenceID != 9999)
            {
                return;
            }

            if (HiCONDLL.vsiCmdCancelMove(MotionBasicMovements.i_AxisSelected, false) != HiCONDLL.ERROR.NONE)
            {
                //DisplayLastError();
                //lbl_ControllerMessages
            }
        }

        public static void JOGPosStop_Keyboard()
        {
            //Debug.WriteLine("Released");            
            uint sequenceID = 0;
            if (HiCONDLL.vsiStatusGetMotionSequenceID(MotionBasicMovements.i_AxisSelected_Keyboard, ref sequenceID) != HiCONDLL.ERROR.NONE)
            {
                return;
            }

            if (sequenceID != 9999)
            {
                return;
            }

            if (HiCONDLL.vsiCmdCancelMove(MotionBasicMovements.i_AxisSelected_Keyboard, false) != HiCONDLL.ERROR.NONE)
            {
                //DisplayLastError();
                //lbl_ControllerMessages
            }
        }

        public void JOGNegMove()
        {
            GUI.GetSetPoints();

            // Check q1 limits
            if (MotionBasicMovements.i_AxisSelected == 0 && GUI.d_Arms_Q_Positions_Deg[0] <= Arm1_Q1_MinPosition_Deg
                || MotionBasicMovements.i_AxisSelected == 1 && GUI.d_Arms_Q_Positions_Deg[1] <= Arm2_Q1_MinPosition_Deg
                || MotionBasicMovements.i_AxisSelected == 2 && GUI.d_Arms_Q_Positions_Deg[2] <= Arm3_Q1_MinPosition_Deg)
            {
                Debug.WriteLine("JOGNegMove: Software limit exceeded");
                JOGNegStop();
            }
            else
            {
                if (MotionBasicMovements.i_AxisSelected >= 0 && MotionBasicMovements.i_AxisSelected <= 2)   //Movimiento articulaciones activas
                {
                    HiCONDLL.vsiCmdExecuteMove(MotionBasicMovements.i_AxisSelected,
                                               -9999,
                                               GUI.d_JOG_Axes_SetPoint_Speed_rev_min,
                                               GUI.d_JOG_Axes_SetPoint_Acceleration_rev_min_2,
                                               HiCONDLL.MoveType.ABSOLUTE,
                                               9999);
                }
                else //Movimiento lineal X, Y, Z
                {
                    if (MotionBasicMovements.i_AxisSelected == 3)    //Mover eje X
                    {
                        Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] - 1.0;
                        Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1];
                        Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2];
                    }

                    else if (MotionBasicMovements.i_AxisSelected == 4)    //Mover eje Y
                    {
                        Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] - 0.000000000001; //Movimiento adicional en X para que efectue el movimiento del otro eje
                        Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1] -1.0;
                        Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2];
                    }

                    else if (MotionBasicMovements.i_AxisSelected == 5)    //Mover eje Z
                    {
                        Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] - 0.000000000001; //Movimiento adicional en X para que efectue el movimiento del otro eje
                        Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1];
                        Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2] - 1.0;
                    }

                    /*
                    GUI.d_SetPoint_Speed_mm_s = 50.0;
                    GUI.d_SetPoint_Acceleration_mm_s_2 = 50.0;
                    Buffered_Motion.bl_Robot_Busy = true;
                    Buffered_Motion.BufferedMovement(false);
                    while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                    { }
                    */

                    GUI.d_SetPoint_Speed_mm_s = 100.0;
                    GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;

                    var motion_Thread = new Motion_Thread();

                    bufferedMotionThread = new Thread(new ThreadStart(motion_Thread.BufferedMotionThread));   //Crear hilo adicional para realizar movimiento con buffer
                    bufferedMotionThread.Start();   //Ejecutar movimiento en otro hilo  
                }
            }

        }

        public static void JOGNegMove_Keyboard()
        {
            GUI.GetSetPoints();

            // Check q1 limits
            if (MotionBasicMovements.i_AxisSelected_Keyboard == 0 && GUI.d_Arms_Q_Positions_Deg[0] <= Arm1_Q1_MinPosition_Deg
                || MotionBasicMovements.i_AxisSelected_Keyboard == 1 && GUI.d_Arms_Q_Positions_Deg[1] <= Arm2_Q1_MinPosition_Deg
                || MotionBasicMovements.i_AxisSelected_Keyboard == 2 && GUI.d_Arms_Q_Positions_Deg[2] <= Arm3_Q1_MinPosition_Deg)
            {
                Debug.WriteLine("JOGNegMove_Keyboard: Software limit exceeded");
                JOGNegStop_Keyboard();
            }
            else
            {
                HiCONDLL.vsiCmdExecuteMove(MotionBasicMovements.i_AxisSelected_Keyboard,
                                           -9999,
                                           GUI.d_JOG_Axes_SetPoint_Speed_rev_min,
                                           GUI.d_JOG_Axes_SetPoint_Acceleration_rev_min_2,
                                           HiCONDLL.MoveType.ABSOLUTE,
                                           9999);
            }
        }

        public static void JOGNegStop()
        {
            //Debug.WriteLine("Released");            
            uint sequenceID = 0;
            if (HiCONDLL.vsiStatusGetMotionSequenceID(MotionBasicMovements.i_AxisSelected, ref sequenceID) != HiCONDLL.ERROR.NONE)
            {
                return;
            }

            if (sequenceID != 9999)
            {
                return;
            }

            if (HiCONDLL.vsiCmdCancelMove(MotionBasicMovements.i_AxisSelected, false) != HiCONDLL.ERROR.NONE)
            {
                //DisplayLastError();
                //lbl_ControllerMessages
            }
        }

        public static void JOGNegStop_Keyboard()
        {
            //Debug.WriteLine("Released");            
            uint sequenceID = 0;
            if (HiCONDLL.vsiStatusGetMotionSequenceID(MotionBasicMovements.i_AxisSelected_Keyboard, ref sequenceID) != HiCONDLL.ERROR.NONE)
            {
                return;
            }

            if (sequenceID != 9999)
            {
                return;
            }

            if (HiCONDLL.vsiCmdCancelMove(MotionBasicMovements.i_AxisSelected_Keyboard, false) != HiCONDLL.ERROR.NONE)
            {
                //DisplayLastError();
                //lbl_ControllerMessages
            }
        }

        public static void Stop()
        {
            if (MotionTimers.motionSequence != null)
            {
                MotionTimers.motionSequence.Cancel();
            }

            if (HiCONDLL.vsiCmdCancelMove(-1, false) != HiCONDLL.ERROR.NONE)
            {
                //DisplayLastError();
            }
        }

        public static void StopAll()
        {
            if (MotionTimers.motionSequence != null)
            {
                MotionTimers.motionSequence.Cancel();
            }

            /*
            if (HiCONDLL.vsiCmdCancelMove(GUI.cb_AxisSelection.SelectedIndex, false) != HiCONDLL.ERROR.NONE)
            {
                //DisplayLastError();
            }
            */
        }

        public static void SetAllAxesPositionsToZero()
        {
            HiCONDLL.vsiCmdSetAxisPosition(0, Convert.ToSingle(0.0));   //X axis set to 0 position
            HiCONDLL.vsiCmdSetAxisPosition(1, Convert.ToSingle(0.0));   //Y axis set to 0 position
            HiCONDLL.vsiCmdSetAxisPosition(2, Convert.ToSingle(0.0));   //Z axis set to 0 position
        }

        /*
        public static void GoHome()
        {
            HiCONDLL.vsiCmdExecuteHoming(0, 0.0, 0.25, 0.25, 0.01);   //Go home axis 0
            //HiCONDLL.vsiCmdExecuteHoming(1, 360.0, 0.25, 0.25, 0.01);   //Go home axis 1
            //HiCONDLL.vsiCmdExecuteHoming(2, 0.0, 0.25, 0.25, 0.01);   //Go home axis 1            
        }
        */

        public static void RelativeMove()
        {
            //Get axis to move            
            uint motionSeqID = newDeviceStatus.motors[i_AxisSelected].MotionSequenceID;
                        
            GUI.GetSetPoints();
            
            //Move axis            
            if (HiCONDLL.vsiCmdExecuteMove(i_AxisSelected, GUI.d_SetPoint_IndividualAxis, GUI.d_SetPoint_Speed_rev_min, GUI.d_SetPoint_Acceleration_rev_min_2, HiCONDLL.MoveType.RELATIVE, ++motionSeqID) != HiCONDLL.ERROR.NONE)
            {
                HiCONDLL.vsiAPIGetLastNotification(out MotionControl.s_ControllerMessage);    //Movement error (Get error info)
            }
        }

        public static void AbsoluteMove()
        {
            //Get axis to move            
            uint motionSeqID = MotionTimers.newDeviceStatus.motors[MotionBasicMovements.i_AxisSelected].MotionSequenceID;

            GUI.GetSetPoints();

            //Move axis            
            if (HiCONDLL.vsiCmdExecuteMove(MotionBasicMovements.i_AxisSelected, GUI.d_SetPoint_IndividualAxis, GUI.d_SetPoint_Speed_rev_min, GUI.d_SetPoint_Acceleration_rev_min_2, HiCONDLL.MoveType.ABSOLUTE, ++motionSeqID) != HiCONDLL.ERROR.NONE)
            {
                HiCONDLL.vsiAPIGetLastNotification(out MotionControl.s_ControllerMessage);    //Movement error (Get error info)
            }
        }

        /*
        public static void LinearInterpolation()
        {            
            if (MotionTimers.motionSequence != null)
            {
                MotionTimers.motionSequence.Cancel();
            }

            GUI.GetSetPoints();

            Debug.WriteLine("X Setpoint [mm] = " + GUI.d_SetPoint_PositionsXYZ_mm[0]);
            Debug.WriteLine("Y Setpoint [mm] = " + GUI.d_SetPoint_PositionsXYZ_mm[1]);
            Debug.WriteLine("Z Setpoint [mm] = " + GUI.d_SetPoint_PositionsXYZ_mm[2]);

            //Check set points
            if (GUI.d_SetPoint_PositionsXYZ_mm[0] < X_MinPosition_mm || GUI.d_SetPoint_PositionsXYZ_mm[0] > X_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje X está fuera del espacio de trabajo del robot");
                return;
            }
            else if (GUI.d_SetPoint_PositionsXYZ_mm[1] < Y_MinPosition_mm || GUI.d_SetPoint_PositionsXYZ_mm[1] > Y_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje Y está fuera del espacio de trabajo del robot");
                return;
            }
            else if (GUI.d_SetPoint_PositionsXYZ_mm[2] < Z_MinPosition_mm || GUI.d_SetPoint_PositionsXYZ_mm[2] > Z_MaxPosition_mm)
            {
                MessageBox.Show("ERROR: La posicion deseada del eje Z está fuera del espacio de trabajo del robot");
                return;
            }

            double[] d_MotorsAngles = new double[3];

            //Initialize lapsed time variables
            
            //StartTime = DateTime.Now;
            //StopWatch = Stopwatch.StartNew();
            //TotalWatch.Start();
            
            Buffered_Motion.StartTime = DateTime.Now;
            Buffered_Motion.StopWatch = Stopwatch.StartNew();
            Buffered_Motion.TotalWatch.Start();

            //Calculate inverse kinematics
            d_MotorsAngles = Kinematics.InverseKinematics.CalculateInverseKinematics(GUI.d_SetPoint_PositionsXYZ_mm);

            // Stop lapsed time
            
            //StopTime = DateTime.Now;
            //StopWatch.Stop();
            //TotalWatch.Stop();
            
            Buffered_Motion.StopTime = DateTime.Now;
            Buffered_Motion.StopWatch.Stop();
            Buffered_Motion.TotalWatch.Stop();

            //TimeSpan elapsed = StopTime.Subtract(StartTime);
            TimeSpan elapsed = Buffered_Motion.StopTime.Subtract(Buffered_Motion.StartTime);
            Debug.WriteLine("Total time [s] = " + elapsed.TotalSeconds.ToString("0.000000"));           
            Debug.WriteLine("Total time [s] = " + Buffered_Motion.StopWatch.Elapsed.TotalSeconds.ToString("0.000000"));
            Debug.WriteLine("Total time [s] = " + Buffered_Motion.TotalWatch.Elapsed.TotalSeconds.ToString("0.000000"));

            //Show inverse kinematics result
            Debug.WriteLine("IK: Axis1[°] = " + d_MotorsAngles[0]);
            Debug.WriteLine("IK: Axis2[°] = " + d_MotorsAngles[1]);
            Debug.WriteLine("IK: Axis3[°] = " + d_MotorsAngles[2]);

            //Calculate motor turns
            double[] d_MotorsTurns = new double[3];
            d_MotorsTurns[0] = d_MotorsAngles[0] / 360.0 * MotionControl.d_ReducerRelation;
            d_MotorsTurns[1] = d_MotorsAngles[1] / 360.0 * MotionControl.d_ReducerRelation;
            d_MotorsTurns[2] = d_MotorsAngles[2] / 360.0 * MotionControl.d_ReducerRelation;

            if (MotionTimers.motionSequence != null)
            {
                MotionTimers.motionSequence.Cancel();
            }

            //motionSequence = new LinearMotionSequence(d_MotorsAngles, d_SetPoint_Speed, d_SetPoint_Acceleration);
            MotionTimers.motionSequence = new LinearMotionSequence(d_MotorsTurns, GUI.d_SetPoint_Speed, GUI.d_SetPoint_Acceleration);

            MotionTimers.motionSequence.ExecuteAsync();  //Execute interpolated linear movement                
        }
        */

        /*
        public static void CircularInterpolation()
        {
            if (motionSequence != null)
            {
                motionSequence.Cancel();
            }

            GetSetPoints();

            motionSequence = new ArcMotionSequence(d_SetPoint_PositionsXYZ_mm, d_SetPoint_Speed, d_SetPoint_Acceleration);

            motionSequence.ExecuteAsync();            
        }
        */
    }
}
