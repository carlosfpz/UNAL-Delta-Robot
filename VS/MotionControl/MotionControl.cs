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
    public static class MotionControl
    {
        //*********************** PUBLIC VARIABLES ******************************
        public static bool bl_ControllerOnline = false;
        public static string s_ControllerMessage;
        public const double d_ReducerRelation = 5.0;
        public static bool bl_ControllerArmed = false;
        public static bool bl_SelectedMotorIsMoving = false;

        //*********************** PRIVATE VARIABLES ******************************
        private static string xmlPath;
        //private static double[] controllerDACVoltage = new double[HiCONDLL.MAX_DAC_CHANNELS];
        //private static HiCONDLL.ERROR result = HiCONDLL.ERROR.NONE;
        //public static HiCONDLL.DeviceStatus newDeviceStatus = new HiCONDLL.DeviceStatus();

        //Initialize motion controller variables
        public static void MotionControl_Initialization()
        {
            xmlPath = @"..\..\..\hiconConfig.xml";
            s_ControllerMessage = "";

            string sHiCONDLLused = HiCONDLL.DLL_PATH;
            //Console.WriteLine("UNAL: HiCONDLL dll used -> " + sHiCONDLLused);
            Debug.WriteLine("UNAL: HiCONDLL dll used -> " + sHiCONDLLused);

            //Controller API Initialization
            HiCONDLL.vsiAPIInitialize();    //Start Vital API
            HiCONDLL.vsiAPIOpenConsole();   //Open Vital API console

            /*
            InitializeIO(digitalInputGroup, digitalInputs);
            InitializeIO(digitalOutputGroup, digitalOutputs);
            */

            //Get controller API Version            
            string s_ControllerAPIVersion = "";
            HiCONDLL.vsiAPIGetVersion(ref s_ControllerAPIVersion);
            Console.WriteLine("UNAL: Controller API version -> " + s_ControllerAPIVersion);
        }

        //Connect motion controller via TCP/IP
        public static void Connect()
        {
            HiCONDLL.vsiAPILoadXMLConfig(xmlPath);    //Load controller configuration            
            string serial = ""; //Get an specific serial controller to connect given by the user

            int i_ConnectionTimeOut_ms = 2000;

            if (HiCONDLL.vsiAPIConnect(serial, MotionTimers.i_ControllerDataExchangeTime_ms, i_ConnectionTimeOut_ms, false) != HiCONDLL.ERROR.NONE)  //Connection error
            {
                HiCONDLL.vsiAPIGetLastNotification(out s_ControllerMessage);    //Get error

                GUI.MotionControl_ConnectionFailed();
            }
            else    //Connection OK
            {
                MotionControl.bl_ControllerOnline = true;
                MotionTimers.SetTimer_ControllerDataExchange();  //Activate periodic controller data exchange timer
                MotionTimers.SetTimer_ControllerReadData();  //Activate periodic controller data read timer
                MotionTimers.SetTimer_CalculateForwardKinematics(); //Activate periodic forward kinematics calculation
                MotionTimers.SetTimer_CalculateVelAccel(); //Activate periodic forward kinematics calculation

                GUI.MotionControl_ConnectionSuccess();  //Activate GUI periodic update              
                //MotionTimers.SetTimer_BufferedMove();

                MotionControl_DIO.SetTimer_GreenLight();    //Toggle beacon light
            }
        }

        //Disconnect motion controller 
        public static void Disconnect()
        {
            bl_ControllerOnline = false;

            MotionBasicMovements.StopAll();

            //Disable timers
            MotionTimers.UnsetTimer_ControllerDataExchange();
            MotionTimers.UnsetTimer_ControllerReadData();
            MotionTimers.UnsetTimer_CalculateForwardKinematics();
            MotionTimers.UnsetTimer_CalculateVelAccel();
            MotionControl_DIO.UnsetTimer_GreenLight();

            // Turn off green light
            MotionControl_DIO.UnsetOutput(MotionControl_DIO.i_DO_GreenLight_Pin);
            MotionControl_DIO.bl_DO_Green_Light = false;

            GUI.timer_GUI_Update.Stop();

            try
            {                
                HiCONDLL.vsiAPIDisconnect();    //Disconect API
            }
            catch (Exception ex)
            {
                Debug.WriteLine("Error en HiCONDLL.vsiAPIDisconnect()" + ex);
            }

            GUI.MotionControl_Disconnected();   //Update GUI
        }
        
        //Arm motion controller
        public static void Arm()
        {
            uint i_AxisToArm = 0b00000111;  //Axes to arm (X, Y, Z)

            try
            {
                HiCONDLL.vsiCmdArm(i_AxisToArm);
                //HiCONDLL.vsiCmdArm(HiCONDLL.AxisMask.X | HiCONDLL.AxisMask.Y | HiCONDLL.AxisMask.Z | HiCONDLL.AxisMask.A | HiCONDLL.AxisMask.B | HiCONDLL.AxisMask.C);

                GUI.MotionControl_Armed();  //Update GUI

                bl_ControllerArmed = true;
            }
            catch (Exception ex)
            {
                Debug.WriteLine("Arm error -> " + ex);                        
            }           
        }

        //Disarm motion controller
        public static void Disarm()
        {
            HiCONDLL.vsiCmdDisarm();
            bl_ControllerArmed = false;
            GUI.MotionControl_Disarmed();   //Update GUI
        }

        //public static void DisableDeviceControl(bool enable)
        //{
            /*
            FollowErrorLED.BackColor = Color.LightGray;
            MotionDoneLED.BackColor = Color.LightGray;

            picHomeStatus.BackColor = Color.LightGray;

            btnARMPID.BackColor = Color.LightGray;
            btnARMPID.Enabled = false;
            btnARMPID.Text = "ARM";

            serialTxtBox.Enabled = true;

            foreach (PictureBox pictureBox in digitalInputs)
                pictureBox.BackColor = Color.LightGray;
            foreach (PictureBox pictureBox in digitalOutputs)
                pictureBox.BackColor = Color.LightGray;
            */
        //}
    }
}
