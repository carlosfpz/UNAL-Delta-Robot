using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Threading;
using VSI;

namespace DeltaRobot_WPF_NetCore
{
    public static class GUI
    {
        //*********************** PUBLIC VARIABLES ******************************
        public static DispatcherTimer timer_GUI_Update;
        //public static double[] d_SetPoint_PositionsXYZ_mm = new double[3];
        //public static double d_SetPoint_Speed = 100.0;
        public static double d_JOG_Axes_SetPoint_Speed_rev_min = 5.0;    //JOG Speed
        public static double d_JOG_Axes_SetPoint_Acceleration_rev_min_2 = 7.5; //JOG acceleration

        public static double d_SetPoint_Speed_rev_min = 5.0;    //Axes movements speed
        public static double d_SetPoint_Acceleration_rev_min_2 = 7.5;    //Axes movements acceleration

        public static double d_SetPoint_Speed_mm_s = 7.5;    //Movements speed
        public static double d_SetPoint_Acceleration_mm_s_2 = 7.5;    //Movements speed

        public static double d_Max_SetPoint_Speed_mm_s = 7500.0;    //Max movements speed
        public static double d_Min_SetPoint_Speed_mm_s = 0.1;    //Min movements speed

        public static double d_Max_SetPoint_Acceleration_mm_s_2 = 12500.0;    //Max Movements acceleration
        public static double d_Min_SetPoint_Acceleration_mm_s_2 = 0.1;    //Min movements acceleration

        public static double[] d_MotorPositions_Deg = new double[3];
        private static double[] d_MotorPositions_Deg_Shadow = new double[3];

        public static double d_SetPoint_IndividualAxis; 

        public static double[] d_Arms_Q_Positions_Deg = new double[3];
        public static double[] d_Arms_Q_Positions_Deg_Shadow = new double[3];

        public static double[] d_EndEffectorPosition_mm = new double[3];
        public static double[] d_EndEffectorPosition_mm_Shadow = new double[3];

        public static double[] d_EndEffectorVelocity_mm_s = new double[3];

        public static double[] d_ActiveJointsSpeed_deg_s = new double[3];

        //*********************** PRIVATE VARIABLES ******************************
        public static MainWindow Form = Application.Current.Windows[0] as MainWindow;
        //Bear in mind the array! Ensure it's the correct Window you're trying to catch.
        
        private const int i_GUI_Update_ms = 20;
        
        public static void InitializeUI()
        {
            //Initialize with dummy values

            d_Arms_Q_Positions_Deg_Shadow[0] = 120.0;
            d_Arms_Q_Positions_Deg_Shadow[1] = 120.0;
            d_Arms_Q_Positions_Deg_Shadow[2] = 120.0;

            d_MotorPositions_Deg_Shadow[0] = -500.0;
            d_MotorPositions_Deg_Shadow[1] = -500.0;
            d_MotorPositions_Deg_Shadow[2] = -500.0;
            
            d_EndEffectorPosition_mm_Shadow[0] = -10.0;
            d_EndEffectorPosition_mm_Shadow[1] = -10.0;
            d_EndEffectorPosition_mm_Shadow[2] = -200.0;

            /*
            Form.tb_SetPoint_XPosition_mm.Text = d_SetPoint_PositionsXYZ_mm[0].ToString("0.0");
            Form.tb_SetPoint_YPosition_mm.Text = d_SetPoint_PositionsXYZ_mm[1].ToString("0.0");
            Form.tb_SetPoint_ZPosition_mm.Text = d_SetPoint_PositionsXYZ_mm[2].ToString("-200.0");
            */
            Form.tb_SetPoint_XPosition_mm.Text = Buffered_Motion.d_EndEffector_TargetPosX_mm.ToString("0.0");
            Form.tb_SetPoint_YPosition_mm.Text = Buffered_Motion.d_EndEffector_TargetPosY_mm.ToString("0.0");
            Form.tb_SetPoint_ZPosition_mm.Text = Buffered_Motion.d_EndEffector_TargetPosZ_mm.ToString("0.0");
            
            Form.tb_SetPoint_Speed_mm_s.Text = d_SetPoint_Speed_mm_s.ToString("0.0");
            Form.tb_SetPoint_Acceleration_mm_s_2.Text = d_SetPoint_Acceleration_mm_s_2.ToString("0.0");
            /*
            cbAxis.SelectedIndex = 0;
            
            for (int dac = 0; dac < HiCONDLL.MAX_DAC_CHANNELS; dac++)
                cbDACChannel.Items.Add(dac);

            cbDACChannel.SelectedIndex = 0;

            txtAccel.Text = "10.00";
            txtPosition.Text = "30.00";
            txtVelocity.Text = "100.00";

            XPos.Tag = 0;
            YPos.Tag = 1;
            ZPos.Tag = 2;
            APos.Tag = 3;
            BPos.Tag = 4;
            CPos.Tag = 5;

            btnARMPID.Enabled = false;
            */
        }

        /*
        public static void DIO_TurnedOn()
        {
            Form.tbt_ToggleDigitalOutput.Content = "Digital output: Turn on";
        }

        public static void DIO_TurnedOff()
        {
            Form.tbt_ToggleDigitalOutput.Content = "Digital output: Turn off";
        }
        */

        public static void Start()
        {            
            /*
            Form.Label1.Content = "Yay! You made it!";
            Form.Top = 0;
            Form.Button1.Width = 50;
            //Et voilá! You have now access to the MainWindow and all it's controls
            //from a separate class/file!

            CreateLabel(text, count); //Creating a control to be added to "Form".
            */
        }

        public static void ChangeSetPointsToPos()
        {
            Form.tb_SetPoint_XPosition_mm.Text = "30";
            Form.tb_SetPoint_YPosition_mm.Text = "30";
            Form.tb_SetPoint_ZPosition_mm.Text = "-300";            
        }

        public static void ChangeSetPointsToNeg()
        {
            Form.tb_SetPoint_XPosition_mm.Text = "-30";
            Form.tb_SetPoint_YPosition_mm.Text = "-30";
            Form.tb_SetPoint_ZPosition_mm.Text = "-300";
        }

        private static void CreateLabel(string Text, int Count)
        {
            /*
            Label aLabel = new Label();
            aLabel.Name = Text.Replace(" ", "") + "Label";
            aLabel.Content = Text + ": ";
            aLabel.HorizontalAlignment = HorizontalAlignment.Right;
            aLabel.VerticalAlignment = VerticalAlignment.Center;
            aLabel.Margin = new Thickness(0);
            aLabel.FontFamily = Form.DIN;
            aLabel.FontSize = 29.333;

            Grid.SetRow(aLabel, Count);
            Grid.SetColumn(aLabel, 0);
            Form.MainGrid.Children.Add(aLabel); //Haha! We're adding it to a Grid in "Form"!
            */
        }

        public static void MotionControl_ConnectionSuccess()
        {
            Console.WriteLine("UNAL: Controller connection sucessfully");

            string serial = new string(new char[10]);
            HiCONDLL.vsiStatusGetSerial(ref serial);

            Console.WriteLine("UNAL: Controller serial -> " + serial.Trim().ToUpper());
            //lbl_Serial.Content = "Serial number: " + serial.Trim().ToUpper();
            Form.lbl_Serial.Content = "Serial number: " + serial.Trim().ToUpper();

            /*
            foreach (PictureBox pictureBox in digitalInputs)
                pictureBox.BackColor = Color.Red;
            foreach (PictureBox pictureBox in digitalOutputs)
                pictureBox.BackColor = Color.Red;
            */

            Form.btn_Connect.Content = "Disconnect";
            Form.btn_Connect.Background = Brushes.Yellow;
            Form.btn_Arm.Visibility = Visibility.Visible;
            Form.btn_SetAllAxesPositionsToZero.Visibility = Visibility.Visible;
            //Form.tbt_ToggleDigitalOutput.Visibility = Visibility.Visible;
            Form.btn_TCP_Connect.Visibility = Visibility.Visible;

            SetTimer_GUI_Update();     //Activate periodic GUI update timer                 
        }

        public static void MotionControl_ConnectionFailed()
        {
            MessageBox.Show("Connection failed: " + MotionControl.s_ControllerMessage, "Controller");
            Form.btn_Arm.Visibility = Visibility.Hidden;
            Form.btn_SetAllAxesPositionsToZero.Visibility = Visibility.Hidden;
            Form.btn_GoToZeroPosition.Visibility = Visibility.Hidden;
            Form.btn_TCP_Connect.Visibility = Visibility.Hidden;
            //Form.tbt_ToggleDigitalOutput.Visibility = Visibility.Hidden;
        }

        public static void MotionControl_Disconnected()
        {            
            Form.btn_Connect.Content = "Connect";
            Form.btn_Connect.Background = Brushes.LightGreen;
            Form.btn_Arm.Visibility = Visibility.Hidden;
            Form.btn_SetAllAxesPositionsToZero.Visibility = Visibility.Hidden;
            //Form.tbt_ToggleDigitalOutput.Visibility = Visibility.Hidden;
            Form.btn_GoToZeroPosition.Visibility = Visibility.Hidden;
            Form.btn_TCP_Connect.Visibility = Visibility.Hidden;
        }

        public static void MotionControl_Armed()
        {                                   
            Form.btn_JOGNeg.Visibility = Visibility.Visible;
            Form.btn_JOGPos.Visibility = Visibility.Visible;               
            //Form.btn_SetInitialPosition.Visibility = Visibility.Visible;
            Form.btn_XYZInterpolatedManualMovement.Visibility = Visibility.Visible;
            Form.btn_GCode.Visibility = Visibility.Visible;
            //Form.btn_GoHome.Visibility = Visibility.Visible;
            Form.btn_GoToZeroPosition.Visibility = Visibility.Visible;
            Form.btn_Connect.Visibility = Visibility.Hidden;
            Form.btn_DemoRoutine_Start.Visibility = Visibility.Visible;
            //Form.btn_ArtificialVision_Start.Visibility = Visibility.Visible;
            
            Form.btn_Arm.Content = "Disarm"; 
            Form.btn_Arm.Background = Brushes.Yellow;
        }

        public static void MotionControl_Disarmed()
        {
            //GUI           
            Form.btn_JOGNeg.Visibility = Visibility.Hidden;
            Form.btn_JOGPos.Visibility = Visibility.Hidden;
            Form.btn_SetInitialPosition.Visibility = Visibility.Hidden;
            Form.btn_XYZInterpolatedManualMovement.Visibility = Visibility.Hidden;
            Form.btn_GCode.Visibility = Visibility.Hidden;
            Form.btn_GoHome.Visibility = Visibility.Hidden;
            Form.btn_DemoRoutine_Start.Visibility = Visibility.Hidden;
            //Form.btn_ArtificialVision_Start.Visibility = Visibility.Hidden;
            Form.btn_Connect.Visibility = Visibility.Visible;
            Form.btn_GoToZeroPosition.Visibility = Visibility.Hidden;

            Form.btn_Arm.Content = "Arm";
            Form.btn_Arm.Background = Brushes.LightGreen;
        }

        public static void GetSetPoints()
        {
            MotionBasicMovements.i_AxisSelected = Form.cb_AxisSelection.SelectedIndex;    //Get selected axis to move   
            
            Buffered_Motion.d_EndEffector_TargetPosX_mm = Convert.ToDouble(Form.tb_SetPoint_XPosition_mm.Text);
            Buffered_Motion.d_EndEffector_TargetPosY_mm = Convert.ToDouble(Form.tb_SetPoint_YPosition_mm.Text);
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = Convert.ToDouble(Form.tb_SetPoint_ZPosition_mm.Text);
            d_SetPoint_Speed_mm_s = Convert.ToDouble(Form.tb_SetPoint_Speed_mm_s.Text);
            d_SetPoint_Acceleration_mm_s_2 = Convert.ToDouble(Form.tb_SetPoint_Acceleration_mm_s_2.Text);

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
        }

        //TIMER GUI UPDATE (Dispatcher timer)
        private static void SetTimer_GUI_Update()
        {
            timer_GUI_Update = new DispatcherTimer();
            timer_GUI_Update.Interval = TimeSpan.FromMilliseconds(i_GUI_Update_ms);  //Periodic timer in ms
            timer_GUI_Update.Tick += Timer_GUI_Update_Event;
            timer_GUI_Update.Start();
        }      

        private static void Timer_GUI_Update_Event(object sender, EventArgs e)
        {
            //Debug.WriteLine("Timer_GUI_Update_Event()");

            //Convert motor positions from double to string
            string[] s_MotorActualPosition_Deg = new string[3];
            s_MotorActualPosition_Deg[0] = string.Format("{0:0.0000}", d_MotorPositions_Deg[0]);    //Get only 4 decimal position
            s_MotorActualPosition_Deg[1] = string.Format("{0:0.0000}", d_MotorPositions_Deg[1]);    //Get only 4 decimal position
            s_MotorActualPosition_Deg[2] = string.Format("{0:0.0000}", d_MotorPositions_Deg[2]);    //Get only 4 decimal position

            //Show motor positions            
            Form.lbl_Arm1_M_Position.Content = s_MotorActualPosition_Deg[0] + " °";
            Form.lbl_Arm2_M_Position.Content = s_MotorActualPosition_Deg[1] + " °";
            Form.lbl_Arm3_M_Position.Content = s_MotorActualPosition_Deg[2] + " °";

            //Show q1 positions (Hips positions -> Speed reducer output) 
            d_Arms_Q_Positions_Deg[0] = d_MotorPositions_Deg[0] / MotionControl.d_ReducerRelation;
            d_Arms_Q_Positions_Deg[1] = d_MotorPositions_Deg[1] / MotionControl.d_ReducerRelation;
            d_Arms_Q_Positions_Deg[2] = d_MotorPositions_Deg[2] / MotionControl.d_ReducerRelation;

            string s_Arm1_Q_Position_Deg = string.Format("{0:0.0000}", d_Arms_Q_Positions_Deg[0]);    //Get only 4 decimal position
            string s_Arm2_Q_Position_Deg = string.Format("{0:0.0000}", d_Arms_Q_Positions_Deg[1]);    //Get only 4 decimal position
            string s_Arm3_Q_Position_Deg = string.Format("{0:0.0000}", d_Arms_Q_Positions_Deg[2]);    //Get only 4 decimal position

            Form.lbl_Arm1_Q1_Position.Content = s_Arm1_Q_Position_Deg + " °"; ;
            Form.lbl_Arm2_Q1_Position.Content = s_Arm2_Q_Position_Deg + " °"; ;
            Form.lbl_Arm3_Q1_Position.Content = s_Arm3_Q_Position_Deg + " °"; ;

            //Show passive positions arm 1
            string s_Arm1_Alpha_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Alpha_Deg[0]);    //Get only 4 decimal position
            string s_Arm1_Beta_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Beta_Deg[0]);    //Get only 4 decimal position
            string s_Arm1_Gamma_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Gamma_Deg[0]);    //Get only 4 decimal position
            string s_Arm1_Theta_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Theta_Deg[0]);    //Get only 4 decimal position

            Form.lbl_Arm1_Alpha_Position.Content = s_Arm1_Alpha_Position_Deg + " °";
            Form.lbl_Arm1_Beta_Position.Content = s_Arm1_Beta_Position_Deg + " °";
            Form.lbl_Arm1_Gamma_Position.Content = s_Arm1_Gamma_Position_Deg + " °";
            Form.lbl_Arm1_Theta_Position.Content = s_Arm1_Theta_Position_Deg + " °";

            //Show passive positions arm 2
            string s_Arm2_Alpha_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Alpha_Deg[1]);    //Get only 4 decimal position
            string s_Arm2_Beta_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Beta_Deg[1]);    //Get only 4 decimal position
            string s_Arm2_Gamma_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Gamma_Deg[1]);    //Get only 4 decimal position
            string s_Arm2_Theta_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Theta_Deg[1]);    //Get only 4 decimal position

            Form.lbl_Arm2_Alpha_Position.Content = s_Arm2_Alpha_Position_Deg + " °";
            Form.lbl_Arm2_Beta_Position.Content = s_Arm2_Beta_Position_Deg + " °";
            Form.lbl_Arm2_Gamma_Position.Content = s_Arm2_Gamma_Position_Deg + " °";
            Form.lbl_Arm2_Theta_Position.Content = s_Arm2_Theta_Position_Deg + " °";

            //Show passive positions arm 3
            string s_Arm3_Alpha_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Alpha_Deg[2]);    //Get only 4 decimal position
            string s_Arm3_Beta_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Beta_Deg[2]);    //Get only 4 decimal position
            string s_Arm3_Gamma_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Gamma_Deg[2]);    //Get only 4 decimal position
            string s_Arm3_Theta_Position_Deg = string.Format("{0:0.0000}", Kinematics.InverseKinematics.d_Theta_Deg[2]);    //Get only 4 decimal position

            Form.lbl_Arm3_Alpha_Position.Content = s_Arm3_Alpha_Position_Deg + " °";
            Form.lbl_Arm3_Beta_Position.Content = s_Arm3_Beta_Position_Deg + " °";
            Form.lbl_Arm3_Gamma_Position.Content = s_Arm3_Gamma_Position_Deg + " °";
            Form.lbl_Arm3_Theta_Position.Content = s_Arm3_Theta_Position_Deg + " °";
            
            //Motor position has changed?
            if (d_MotorPositions_Deg[0] != d_MotorPositions_Deg_Shadow[0] ||
                d_MotorPositions_Deg[1] != d_MotorPositions_Deg_Shadow[1] ||
                d_MotorPositions_Deg[2] != d_MotorPositions_Deg_Shadow[2])
            {
                //Debug.WriteLine("Motor position has changed");
                
                //Update shadow positions
                d_MotorPositions_Deg_Shadow[0] = d_MotorPositions_Deg[0];
                d_MotorPositions_Deg_Shadow[1] = d_MotorPositions_Deg[1];
                d_MotorPositions_Deg_Shadow[2] = d_MotorPositions_Deg[2];

                //Calculate forward kinematics
                Kinematics.ForwardKinematics.bl_CalculateForwardKinematics = true;
                //d_MotorsAngles = DirectKinematic.CalculateDirectKinematics(d_MotorPositions_Deg);               
            }

            //Show active joints speeds
            string s_ActiveJoint1Speed_deg_s = string.Format("{0:0.00}", d_ActiveJointsSpeed_deg_s[0]);    //Get only 2 decimal position
            string s_ActiveJoint2Speed_deg_s = string.Format("{0:0.00}", d_ActiveJointsSpeed_deg_s[1]);    //Get only 2 decimal position
            string s_ActiveJoint3Speed_deg_s = string.Format("{0:0.00}", d_ActiveJointsSpeed_deg_s[2]);    //Get only 2 decimal position

            Form.lbl_ActiveJoint1_Speed_deg_s.Content = s_ActiveJoint1Speed_deg_s + " °/s";
            Form.lbl_ActiveJoint2_Speed_deg_s.Content = s_ActiveJoint2Speed_deg_s + " °/s";
            Form.lbl_ActiveJoint3_Speed_deg_s.Content = s_ActiveJoint3Speed_deg_s + " °/s";

            //Show end effector position            
            string s_EndEffectorPosition_X_mm = string.Format("{0:0.0}", d_EndEffectorPosition_mm[0]);    //Get only 1 decimal position
            string s_EndEffectorPosition_Y_mm = string.Format("{0:0.0}", d_EndEffectorPosition_mm[1]);    //Get only 1 decimal position
            string s_EndEffectorPosition_Z_mm = string.Format("{0:0.0}", d_EndEffectorPosition_mm[2]);    //Get only 1 decimal position

            Form.lbl_XPosition.Content = s_EndEffectorPosition_X_mm + " mm";
            Form.lbl_YPosition.Content = s_EndEffectorPosition_Y_mm + " mm";
            Form.lbl_ZPosition.Content = s_EndEffectorPosition_Z_mm + " mm";

            //Show end effector velocities
            string s_EndEffectorVelocity_X_mm_s = string.Format("{0:0.00}", d_EndEffectorVelocity_mm_s[0]);    //Get only 2 decimal position
            string s_EndEffectorVelocity_Y_mm_s = string.Format("{0:0.00}", d_EndEffectorVelocity_mm_s[1]);    //Get only 2 decimal position
            string s_EndEffectorVelocity_Z_mm_s = string.Format("{0:0.00}", d_EndEffectorVelocity_mm_s[2]);    //Get only 2 decimal position
                        
            Form.lbl_EndEffector_X_Vel_mm_s.Content = s_EndEffectorVelocity_X_mm_s + " mm/s";
            Form.lbl_EndEffector_Y_Vel_mm_s.Content = s_EndEffectorVelocity_Y_mm_s + " mm/s";
            Form.lbl_EndEffector_Z_Vel_mm_s.Content = s_EndEffectorVelocity_Z_mm_s + " mm/s";

            //Show controller messages            
            Form.lbl_ControllerMessages.Content = MotionControl.s_ControllerMessage;

            //Show controller status
            Form.lbl_FifoVectorLevel_ms.Content = "FifoVectorLevel [ms] " + MotionTimers.d_FifoVectorLevel_ms;
            Form.lbl_CmdFIFOLevel.Content = "Cmd FIFO Level " + MotionTimers.d_CmdFIFOLevel;
            Form.lbl_CmdFIFOSize.Content = "FIFO Size " + MotionTimers.d_CmdFIFOSize;
            Form.lbl_ElapsedTimeBufferDownload_ms.Content = "ElapsedTimeBufferDownload [ms]" + MotionTimers.d_ElapsedTimeBufferDownload_ms;
            Form.lbl_VectorsDownloadedInOneCycle.Content = "VectorsDownloadedInOneCycle " + MotionTimers.d_VectorsDownloadedInOneCycle;

            //Activate / deactivate buttons motion control
            if (MotionControl.bl_ControllerArmed && MotionControl.bl_SelectedMotorIsMoving)
            {                
                Form.btn_StopMovement.Visibility = Visibility.Visible;
                Form.btn_StopAllMovements.Visibility = Visibility.Visible;

                /*
                moveButton.Enabled = enable;
                homeButton.Enabled = enable;
                zeroAllButton.Enabled = enable;

                executeTestButton.Enabled = enable;

                cancelMoveButton.Enabled = !enable;
                cancelAllButton.Enabled = !enable;
                clearPositionButton.Enabled = !enable;
                */
            }
            else if (MotionControl.bl_ControllerArmed && !MotionControl.bl_SelectedMotorIsMoving)
            {                
                Form.btn_JOGNeg.Visibility = Visibility.Visible;
                Form.btn_JOGPos.Visibility = Visibility.Visible;
                Form.btn_StopMovement.Visibility = Visibility.Hidden;
                Form.btn_StopAllMovements.Visibility = Visibility.Hidden;
            }

            /*
            Debug.WriteLine("d_LastMotorAngles[0] = " + Buffered_Motion.d_LastMotorAngles[0]);
            Debug.WriteLine("d_LastMotorAngles[1] = " + Buffered_Motion.d_LastMotorAngles[1]);
            Debug.WriteLine("d_LastMotorAngles[2] = " + Buffered_Motion.d_LastMotorAngles[2]);
            Debug.WriteLine("d_LastMotorAngles[3] = " + Buffered_Motion.d_LastMotorAngles[3]);
            Debug.WriteLine("d_LastMotorAngles[4] = " + Buffered_Motion.d_LastMotorAngles[4]);
            Debug.WriteLine("d_LastMotorAngles[5] = " + Buffered_Motion.d_LastMotorAngles[5]);
            */

            /*
            Debug.WriteLine("LimitSwitch_J1_Pos = " + MotionControl_DIO.bl_LimitSwitch_J1_Pos);
            Debug.WriteLine("LimitSwitch_J2_Pos = " + MotionControl_DIO.bl_LimitSwitch_J2_Pos);
            Debug.WriteLine("LimitSwitch_J3_Pos = " + MotionControl_DIO.bl_LimitSwitch_J3_Pos);
            Debug.WriteLine("LimitSwitch_J1_Neg = " + MotionControl_DIO.bl_LimitSwitch_J1_Neg);
            Debug.WriteLine("LimitSwitch_J2_Neg = " + MotionControl_DIO.bl_LimitSwitch_J2_Neg);
            Debug.WriteLine("LimitSwitch_J3_Neg = " + MotionControl_DIO.bl_LimitSwitch_J3_Neg);
            */

            /*
            Debug.WriteLine("HomeSwitch_J1 = " + MotionControl_DIO.bl_HomeSwitch_J1);
            Debug.WriteLine("HomeSwitch_J1 = " + MotionControl_DIO.bl_HomeSwitch_J1);
            Debug.WriteLine("HomeSwitch_J1 = " + MotionControl_DIO.bl_HomeSwitch_J1);
            */
        }
    }
}
