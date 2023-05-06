using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public static class MotionTimers
    {
        //*********************** PUBLIC VARIABLES ******************************
        public static bool bl_ControllerArmedState = false;

        public static double d_FifoVectorLevel_ms;    //Normal value = 0
        public static double d_CmdFIFOLevel;  //Normal value = 0
        public static double d_CmdFIFOSize;   //Normal value = 2048
        public static double d_ElapsedTimeBufferDownload_ms;  //Normal value = Aprox 60 
        public static double d_VectorsDownloadedInOneCycle;   //Normal value = 0

        public const int i_ControllerDataExchangeTime_ms = 5; //5ms (Minimum)

        public static HiCONDLL.DeviceStatus newDeviceStatus = new HiCONDLL.DeviceStatus();

        public static MotionSequence motionSequence = null;
       
        public static Timer timer_ControllerDataExchange;
        public static Timer timer_ControllerReadData;
        public static Timer timer_CalculateForwardKinematics;
        public static Timer timer_CalculateVelAccel;

        //*********************** PRIVATE VARIABLES ******************************
        private static bool bl_ControllerOnline = false;
        private static bool isArmed = false;
        private const int i_ControllerReadDataTime_ms = 5; //5ms minimo
        private const int i_CalculateForwardKinematicsTime_ms = 25;
        private const int i_CalculateVelAccelTime_ms = 100;
        
        //***********************
        //DispatcherTimer -> Uses UI thread (Can modify UI)
        //Timer -> Not uses UI thread (Cannot modify UI)
        //***********************

        // ************************************ TIMER CONTROLLER DATA EXCHANGE (Timer) *********************************************
        public static void SetTimer_ControllerDataExchange()
        {
            timer_ControllerDataExchange = new System.Timers.Timer(i_ControllerDataExchangeTime_ms);   //Periodic timer in ms             
            timer_ControllerDataExchange.Elapsed += Timer_ControllerDataExchange_Event;
            timer_ControllerDataExchange.AutoReset = true;
            timer_ControllerDataExchange.Enabled = true;
        }

        public static void UnsetTimer_ControllerDataExchange()
        {
            timer_ControllerDataExchange.Stop();
            timer_ControllerDataExchange.Enabled = false;
        }

        public static void Timer_ControllerDataExchange_Event(Object source, ElapsedEventArgs e)
        {
            //Debug.WriteLine("Timer_ControllerDataExchange_Event()");
            /*
            if (!bl_ControllerOnline)  //Controller offline? -> Do nothing
            {
                return;
            }
            */

            /*          
            HiCONDLL.vsiCmdDataExchange(ref newDeviceStatus);   // //Required to exchange data and maintain connection with the device

            bl_ControllerOnline = newDeviceStatus.IsOnline;    //Get if controller is online
            */

            /*   
            for (int axis = 0; axis < HiCONDLL.MAX_AXIS; axis++)
            {
                controllerAxisMoving[axis] = HiCONDLL.vsiStatusIsMoving(axis);
                controllerAxisHomed[axis] = HiCONDLL.vsiStatusIsHomeFound(axis);
            }

            HiCONDLL.vsiStatusGetFollowErrorBits(ref controllerFollowingErrorBits);
            HiCONDLL.vsiStatusGetAxisPositions(controllerAxisPositions);
            */

            // Buffered timer
            //string ver = "";
            //HiCONDLL.vsiAPIGetVersion(ref ver);
            //lblVersion.Text = "HiCON API Version: " + ver;

            //OnlineLED.BackColor = HiCONDLL.vsiStatusIsOnline() ? Color.Green : Color.Red;
            //MovingLED.BackColor = HiCONDLL.vsiStatusIsMoving(-1) ? Color.Green : Color.Red;

            if (HiCONDLL.vsiStatusIsOnline())
            {
                //Debug.WriteLine("Timer_ControllerDataExchange_Event() - HiCONDLL.vsiStatusIsOnline = true");

                //Get controller status
                bl_ControllerOnline = newDeviceStatus.IsOnline;    //Get if controller is online
                bl_ControllerArmedState = HiCONDLL.vsiStatusIsArmed();
                //ArmLED.BackColor = armedState ? Color.Green : Color.Red;

                if (bl_ControllerArmedState != isArmed) //Arm state changed
                {
                    if (!bl_ControllerArmedState) //Disarmed somehow
                    {
                        string error;
                        HiCONDLL.vsiAPIGetLastNotification(out error);
                        Debug.WriteLine("Timer_ControllerDataExchange_Event ERROR: Controller disarmed suddenly -> " + error);
                        //txtError.Text = error;
                    }
                    else
                    {
                        //txtError.Text = "";
                    }
                }

                //vsiCmdDataExchange: Sends all outgoing data to the motion controller and receives all input data.
                //This function must be called in the application update loop
                HiCONDLL.vsiCmdDataExchange(ref newDeviceStatus);

                //Read motors positions
                double[] b_MotorsPositions_Turns = new double[HiCONDLL.MAX_AXIS]; //6 rows vector
                HiCONDLL.vsiStatusGetAxisPositions(b_MotorsPositions_Turns);  //Reads the current position of all motors [Turns]
                /*
                Debug.WriteLine("Motor0 = " + b_MotorsPositions_Turns[0]);
                Debug.WriteLine("Motor1 = " + b_MotorsPositions_Turns[1]);
                Debug.WriteLine("Motor2 = " + b_MotorsPositions_Turns[2]);
                Debug.WriteLine("Motor3 = " + b_MotorsPositions_Turns[3]);
                Debug.WriteLine("Motor4 = " + b_MotorsPositions_Turns[4]);
                Debug.WriteLine("Motor5 = " + b_MotorsPositions_Turns[5]);
                */

                /*
                XPos.Text = string.Format("{0:n4}", axisPositions[0]);
                YPos.Text = string.Format("{0:n4}", axisPositions[1]);
                ZPos.Text = string.Format("{0:n4}", axisPositions[2]);
                APos.Text = string.Format("{0:n4}", axisPositions[3]);
                BPos.Text = string.Format("{0:n4}", axisPositions[4]);
                CPos.Text = string.Format("{0:n4}", axisPositions[5]);
                */

                //show some diagnostics 
                double[] systemVars = new double[10];
                HiCONDLL.vsiAPIGetSysVars(systemVars);  //Esta funcion esta repetida
                
                if (isArmed && Buffered_Motion.bl_MotionInProgress)    //Verificar si se desea realizar un movimiento usando buffer
                {                                 
                    Buffered_Motion.ExecuteRobotBufferedMotion();
                }                
            }            

            isArmed = bl_ControllerArmedState;
        }

        // ************************************ TIMER CONTROLLER READ DATA (Timer) *********************************************
                
        public static void SetTimer_ControllerReadData()
        {
            timer_ControllerReadData = new System.Timers.Timer(i_ControllerReadDataTime_ms);   //Peridic timer in ms     
            timer_ControllerReadData.Elapsed += Timer_ControllerReadData_Event;
            timer_ControllerReadData.AutoReset = true;
            timer_ControllerReadData.Enabled = true;
        }

        public static void UnsetTimer_ControllerReadData()
        {
            timer_ControllerReadData.Stop();
            timer_ControllerReadData.Enabled = false;
        }

        public static void Timer_ControllerReadData_Event(Object source, ElapsedEventArgs e)
        {
            if (newDeviceStatus.DriveEnable != MotionControl.bl_ControllerArmed) //Controller arm status changed? -> Check status
            {
                if (!newDeviceStatus.DriveEnable)   //Now controller is not armed
                {
                    HiCONDLL.vsiAPIGetLastNotification(out MotionControl.s_ControllerMessage);    //Controller disarmed because of an error
                }
                else
                {
                    MotionControl.s_ControllerMessage = "";
                }

                MotionControl.bl_ControllerArmed = newDeviceStatus.DriveEnable;  //Update controller arm status
            }

            if (motionSequence != null && motionSequence.Complete)
            {
                MotionControl.s_ControllerMessage = motionSequence.ErrorMessage;
                motionSequence.Cancel();
                motionSequence = null;
            }

            /*
            if (motionSequence != null && motionSequence.Complete)
            {                
                controllerMessage = motionSequence.ErrorMessage;
                motionSequence.Cancel();
                motionSequence = null;
            }
            */

            if (!bl_ControllerOnline)
            {
                //MotionControl.DisableDeviceControl(false);
                return;
            }

            /*
            //FollowErrorLED.BackColor = (newDeviceStatus.FollowingErrorActive) ? Color.Red : Color.Lime;

            if (newDeviceStatus.motors != null)
            {
                //.BackColor = newDeviceStatus.motors[cbAxis.SelectedIndex].IsMotionActive ? Color.Red : Color.Lime;
                //picHomeStatus.BackColor = newDeviceStatus.motors[cbAxis.SelectedIndex].IsHomed ? Color.LightGreen : Color.Red;
            }
            */

            MotionControl_DIO.ReadDigitalInputs_Status();
            MotionControl_DIO.ReadDigitalOutputs_Status();
            MotionControl_DIO.Beacon_Set_Reset(); //Modificar status baliza
                        
            // Check limit switches reached
            if (MotionControl_DIO.bl_LimitSwitch_J1_Pos_Shadow && MotionControl_DIO.bl_LimitSwitch_J1_Pos == false)
            {
                //MotionBasicMovements.StopAll();
                Debug.WriteLine("LimitSwitch_J1_Pos reached");
                MotionControl_DIO.bl_LimitSwitch_J1_Pos_Shadow = false;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J2_Pos_Shadow && MotionControl_DIO.bl_LimitSwitch_J2_Pos == false)
            {
                //MotionBasicMovements.StopAll();
                Debug.WriteLine("LimitSwitch_J2_Pos reached");
                MotionControl_DIO.bl_LimitSwitch_J2_Pos_Shadow = false;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J3_Pos_Shadow && MotionControl_DIO.bl_LimitSwitch_J3_Pos == false)
            {
                //MotionBasicMovements.StopAll();
                Debug.WriteLine("LimitSwitch_J3_Pos reached");
                MotionControl_DIO.bl_LimitSwitch_J3_Pos_Shadow = false;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J1_Neg_Shadow && MotionControl_DIO.bl_LimitSwitch_J1_Neg == false)
            {
                //MotionBasicMovements.StopAll();
                Debug.WriteLine("LimitSwitch_J1_Neg reached");
                MotionControl_DIO.bl_LimitSwitch_J1_Neg_Shadow = false;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J2_Neg_Shadow && MotionControl_DIO.bl_LimitSwitch_J2_Neg == false)
            {
                //MotionBasicMovements.StopAll();
                Debug.WriteLine("LimitSwitch_J2_Neg reached");
                MotionControl_DIO.bl_LimitSwitch_J2_Neg_Shadow = false;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J3_Neg_Shadow && MotionControl_DIO.bl_LimitSwitch_J3_Neg == false)
            {
                //MotionBasicMovements.StopAll();
                Debug.WriteLine("LimitSwitch_J3_Neg reached");
                MotionControl_DIO.bl_LimitSwitch_J3_Neg_Shadow = false;
            }

            // Check limit switches released
            if (MotionControl_DIO.bl_LimitSwitch_J1_Pos_Shadow == false && MotionControl_DIO.bl_LimitSwitch_J1_Pos)
            {               
                Debug.WriteLine("LimitSwitch_J1_Pos sensor released");
                MotionControl_DIO.bl_LimitSwitch_J1_Pos_Shadow = true;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J2_Pos_Shadow == false && MotionControl_DIO.bl_LimitSwitch_J2_Pos)
            {                
                Debug.WriteLine("LimitSwitch_J2_Pos sensor released");
                MotionControl_DIO.bl_LimitSwitch_J2_Pos_Shadow = true;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J3_Pos_Shadow == false && MotionControl_DIO.bl_LimitSwitch_J3_Pos)
            {               
                Debug.WriteLine("LimitSwitch_J3_Pos sensor released");
                MotionControl_DIO.bl_LimitSwitch_J3_Pos_Shadow = true;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J1_Neg_Shadow == false && MotionControl_DIO.bl_LimitSwitch_J1_Neg)
            {               
                Debug.WriteLine("LimitSwitch_J1_Neg sensor released");
                MotionControl_DIO.bl_LimitSwitch_J1_Neg_Shadow = true;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J2_Neg_Shadow == false && MotionControl_DIO.bl_LimitSwitch_J2_Neg)
            {                
                Debug.WriteLine("LimitSwitch_J2_Neg sensor released");
                MotionControl_DIO.bl_LimitSwitch_J2_Neg_Shadow = true;
            }

            if (MotionControl_DIO.bl_LimitSwitch_J3_Neg_Shadow == false && MotionControl_DIO.bl_LimitSwitch_J3_Neg)
            {                
                Debug.WriteLine("LimitSwitch_J3_Neg sensor released");
                MotionControl_DIO.bl_LimitSwitch_J3_Neg_Shadow = true;
            }
            
            //Get axes positions
            if (newDeviceStatus.motors != null)
            {
                MotionControl.bl_SelectedMotorIsMoving = newDeviceStatus.motors[MotionBasicMovements.i_AxisSelected].IsMotionActive;  //Check if selected motor is moving
                
                //Get axes positions (Turns, then convert to degrees)
                GUI.d_MotorPositions_Deg[0] = newDeviceStatus.motors[0].ActualPosition * 360.0; //Convert turns to degrees
                GUI.d_MotorPositions_Deg[1] = newDeviceStatus.motors[1].ActualPosition * 360.0; //Convert turns to degrees
                GUI.d_MotorPositions_Deg[2] = newDeviceStatus.motors[2].ActualPosition * 360.0; //Convert turns to degrees
            }

            double[] systemVars = new double[10];
            HiCONDLL.vsiAPIGetSysVars(systemVars);  //Leer el estado del controlador de movimiento

            //Get controller status variables
            d_FifoVectorLevel_ms = systemVars[0];
            d_CmdFIFOLevel = systemVars[1];
            d_CmdFIFOSize = systemVars[2];
            d_ElapsedTimeBufferDownload_ms = systemVars[3];
            d_VectorsDownloadedInOneCycle = systemVars[4];            
        }

        // ************************************ TIMER CALCULATE FORWARD KINEMATICS (Timer) *********************************************
        public static void SetTimer_CalculateForwardKinematics()
        {
            timer_CalculateForwardKinematics = new System.Timers.Timer(i_CalculateForwardKinematicsTime_ms);   //Peridic timer in ms     
            timer_CalculateForwardKinematics.Elapsed += Timer_CalculateForwardKinematics_Event;
            timer_CalculateForwardKinematics.AutoReset = true;
            timer_CalculateForwardKinematics.Enabled = true;
        }

        public static void UnsetTimer_CalculateForwardKinematics()
        {
            timer_CalculateForwardKinematics.Stop();
            timer_CalculateForwardKinematics.Enabled = false;
        }

        public static void Timer_CalculateForwardKinematics_Event(Object source, ElapsedEventArgs e)
        {
            if (Kinematics.ForwardKinematics.bl_CalculateForwardKinematics)
            {
                Kinematics.ForwardKinematics.bl_CalculateForwardKinematics = false;

                // Calculate forward kinematics (End effector position based on active joints angles)
                bool bl_Valid_Result_FwdKinematics = false;
                double[] d_Arms_Q_Positions_Deg_Temp = new double[3];

                (bl_Valid_Result_FwdKinematics, d_Arms_Q_Positions_Deg_Temp) = Kinematics.ForwardKinematics.CalculateForwardKinematics(GUI.d_Arms_Q_Positions_Deg, false);
                
                if (bl_Valid_Result_FwdKinematics)
                {
                    GUI.d_EndEffectorPosition_mm = d_Arms_Q_Positions_Deg_Temp;
                    bl_Valid_Result_FwdKinematics = false;
                }

                //Calculate pasive angles
                bool bl_Valid_Result_InvKinematics;
                bl_Valid_Result_InvKinematics = Kinematics.ForwardKinematics.CalculatePasiveAngles();

                if (bl_Valid_Result_InvKinematics)
                {
                    //GUI.
                    bl_Valid_Result_InvKinematics = false;
                }
            }
        }

        // ************************************ TIMER CALCULATE VELOCITY / ACCELERATION (Timer) *********************************************
        public static void SetTimer_CalculateVelAccel()
        {
            timer_CalculateVelAccel = new System.Timers.Timer(i_CalculateVelAccelTime_ms);   //Peridic timer in ms     
            timer_CalculateVelAccel.Elapsed += Timer_CalculateVelAccel_Event;
            timer_CalculateVelAccel.AutoReset = true;
            timer_CalculateVelAccel.Enabled = true;
        }

        public static void UnsetTimer_CalculateVelAccel()
        {
            timer_CalculateVelAccel.Stop();
            timer_CalculateVelAccel.Enabled = false;
        }

        public static void Timer_CalculateVelAccel_Event(Object source, ElapsedEventArgs e)
        {
            //ACTIVE JOINTS SPEED
            double d_Delta_Angle_deg;

            //Calcular velocidad angulo activo 1
            d_Delta_Angle_deg = GUI.d_Arms_Q_Positions_Deg[0] - GUI.d_Arms_Q_Positions_Deg_Shadow[0];
            GUI.d_ActiveJointsSpeed_deg_s[0] = (d_Delta_Angle_deg * 1000.0) / i_CalculateVelAccelTime_ms;

            //Calcular velocidad angulo activo 2
            d_Delta_Angle_deg = GUI.d_Arms_Q_Positions_Deg[1] - GUI.d_Arms_Q_Positions_Deg_Shadow[1];
            GUI.d_ActiveJointsSpeed_deg_s[1] = (d_Delta_Angle_deg * 1000.0) / i_CalculateVelAccelTime_ms;

            //Calcular velocidad angulo activo 3
            d_Delta_Angle_deg = GUI.d_Arms_Q_Positions_Deg[2] - GUI.d_Arms_Q_Positions_Deg_Shadow[2];
            GUI.d_ActiveJointsSpeed_deg_s[2] = (d_Delta_Angle_deg * 1000.0) / i_CalculateVelAccelTime_ms;

            //Update shadow positions
            GUI.d_Arms_Q_Positions_Deg_Shadow[0] = GUI.d_Arms_Q_Positions_Deg[0];
            GUI.d_Arms_Q_Positions_Deg_Shadow[1] = GUI.d_Arms_Q_Positions_Deg[1];
            GUI.d_Arms_Q_Positions_Deg_Shadow[2] = GUI.d_Arms_Q_Positions_Deg[2];

            //END EFFECTOR VELOCITY            
            double d_Delta_Pos_mm;

            //Calcular velocidad X
            d_Delta_Pos_mm = GUI.d_EndEffectorPosition_mm[0] - GUI.d_EndEffectorPosition_mm_Shadow[0];
            GUI.d_EndEffectorVelocity_mm_s[0] = (d_Delta_Pos_mm * 1000.0) / i_CalculateVelAccelTime_ms;

            //Calcular velocidad Y
            d_Delta_Pos_mm = GUI.d_EndEffectorPosition_mm[1] - GUI.d_EndEffectorPosition_mm_Shadow[1];
            GUI.d_EndEffectorVelocity_mm_s[1] = (d_Delta_Pos_mm * 1000.0) / i_CalculateVelAccelTime_ms;

            //Calcular velocidad Z
            d_Delta_Pos_mm = GUI.d_EndEffectorPosition_mm[2] - GUI.d_EndEffectorPosition_mm_Shadow[2];
            GUI.d_EndEffectorVelocity_mm_s[2] = (d_Delta_Pos_mm * 1000.0) / i_CalculateVelAccelTime_ms;
            
            //Update shadow positions
            GUI.d_EndEffectorPosition_mm_Shadow[0] = GUI.d_EndEffectorPosition_mm[0];
            GUI.d_EndEffectorPosition_mm_Shadow[1] = GUI.d_EndEffectorPosition_mm[1];
            GUI.d_EndEffectorPosition_mm_Shadow[2] = GUI.d_EndEffectorPosition_mm[2];            
        }

        //public static void SetTimer_BufferedMove()
        //{
        //timer_BufferedMove = new System.Timers.Timer(10);   //Peridic timer in ms     
        //timer_BufferedMove.Elapsed += Timer_BufferedMove_Event;
        //timer_BufferedMove.AutoReset = true;
        //timer_BufferedMove.Enabled = true;
        //}

        /*
        public static void Timer_BufferedMove_Event(Object source, ElapsedEventArgs e)    //Timer periodico cada 10ms
        {            
            string ver = "";
            HiCONDLL.vsiAPIGetVersion(ref ver);
            //lblVersion.Text = "HiCON API Version: " + ver;

            //OnlineLED.BackColor = HiCONDLL.vsiStatusIsOnline() ? Color.Green : Color.Red;
            //MovingLED.BackColor = HiCONDLL.vsiStatusIsMoving(-1) ? Color.Green : Color.Red;

            bool armedState = false;

            if (HiCONDLL.vsiStatusIsOnline())
            {
                armedState = HiCONDLL.vsiStatusIsArmed();

                //ArmLED.BackColor = armedState ? Color.Green : Color.Red;

                if (armedState != isArmed) //Arm state changed
                {
                    if (!armedState) //disarmed somehow
                    {
                        string error;
                        HiCONDLL.vsiAPIGetLastNotification(out error);
                        Debug.WriteLine("Error");
                        //txtError.Text = error;
                    }
                    else
                    {
                        //txtError.Text = "";
                    }
                }

                //vsiCmdDataExchange: Sends all outgoing data to the motion controller and receives all input data.
                //This function must be called in the application update loop
                HiCONDLL.vsiCmdDataExchange(ref newDeviceStatus);   //Esta funcion esta repetida

                double[] axisPositions = new double[HiCONDLL.MAX_AXIS]; //Vector de 6 posiciones

                HiCONDLL.vsiStatusGetAxisPositions(axisPositions);  //Reads the current position of all axes and returns an array of doubles

                //XPos.Text = string.Format("{0:n4}", axisPositions[0]);
                //YPos.Text = string.Format("{0:n4}", axisPositions[1]);
                //ZPos.Text = string.Format("{0:n4}", axisPositions[2]);
                //APos.Text = string.Format("{0:n4}", axisPositions[3]);
                //BPos.Text = string.Format("{0:n4}", axisPositions[4]);
                //CPos.Text = string.Format("{0:n4}", axisPositions[5]);

                //show some diagnostics 
                double[] systemVars = new double[10];
                HiCONDLL.vsiAPIGetSysVars(systemVars);  //Esta funcion esta repetida

                //textBox1.Text = systemVars[0].ToString(); //FifoVectorLevel milliseconds
                //textBox2.Text = systemVars[1].ToString(); //CmdFIFOLevel 
                //textBox3.Text = systemVars[2].ToString(); //CmdFIFOSize
                //textBox4.Text = systemVars[3].ToString(); //Elapsed millisec between buffer download
                //textBox5.Text = systemVars[4].ToString(); //vectors downloaded in one cycle
                //textBox6.Text = systemVars[5].ToString(); //DLL internal buffer current fill level
                //textBox7.Text = systemVars[6].ToString(); //DLL internal buffer max size

                btnGO.Enabled = !HiCONDLL.vsiStatusIsMoving(-1) && !MotionInProgress;   //Habilitar el boton de realizar movimiento

                // Controller is enabled.  perform any motion command
                if (isArmed && MotionInProgress)
                {
                    GenerateMotionBuffer();
                }
            }
            else
            {
                //ArmLED.BackColor = Color.Red;
            }

            isArmed = armedState;            
        }
        */
    }
}
