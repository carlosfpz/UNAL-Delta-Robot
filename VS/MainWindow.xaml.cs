using ExcelDataReader;
using Microsoft.Win32;
using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Threading;
using System.Timers;
using System.Windows;
using System.Windows.Input;
using VSI;

// NOTA: Para correr el proyecto se requiere instalar:
// 1. NET Core 5.0 SDK
// 2. Al instalar Visual Studio adicionar  opcion "Desktop development with C++" de lo contrario saca error al cargar la libreria HiCONMotionControlAPI(x64).dll

namespace DeltaRobot_WPF_NetCore
{
    public partial class MainWindow : Window
    {
        //float: 7 digits (Uses 2 base system)
        //double: 15 - 16 digits (Uses 2 base system)
        //decimal: 28-29 digits (Uses 10 base system)

        //*********************** PUBLIC VARIABLES ******************************

        //*********************** PRIVATE VARIABLES ******************************
        //private PictureBox[,] digitalInputs = new PictureBox[4, 16];
        //private PictureBox[,] digitalOutputs = new PictureBox[4, 8];  
        private const double d_MaxSpeed = 250.0;
        //private GCodes gCodes;
        //private string sSelectedFile;
        private Excel excel;

        //private Thread bufferedMotionThread;    //Hilo de ejecucion de movimientos con buffer        

        private Thread bufferedMotionThread_Demo;    //Hilo de ejecucion de movimientos con buffer
        private bool bl_ExecuteInfiniteMovements = false;

        public MainWindow()
        {
            InitializeComponent();  //Starts all controls created with the XAML Designer

            GUI.InitializeUI();

            MotionControl.MotionControl_Initialization();

            Console.WriteLine("UNAL: Program started");
            Debug.WriteLine("UNAL: Program started");
            string sOS = System.Environment.GetEnvironmentVariable("PROCESSOR_ARCHITECTURE");    //Get processor architecture (x86 / x64)
            Console.WriteLine("UNAL: Processor architecture -> " + sOS);
            Debug.WriteLine("UNAL: Processor architecture -> " + sOS);

            //Class instances            
            //gCodes = new GCodes();
            excel = new Excel();

            //Check lapsed time variables
            Debug.WriteLine("Stopwatch -> Frequency = " + Stopwatch.Frequency.ToString());
            Debug.WriteLine("Stopwatch -> ns per tick = " + (1000000000 / Stopwatch.Frequency).ToString());
            Debug.WriteLine("Stopwatch -> High resolution = " + Stopwatch.IsHighResolution.ToString());
        }

        /*
        protected override void OnClosing(CancelEventArgs e)
        {
            Console.WriteLine("UNAL: Closing app");
            Debug.WriteLine("UNAL: Closing app");

            MotionControl.Disarm();
            MotionControl.APIDisconnect();

            base.OnClosing(e);
        }
        */

        /*
        private void BufferedMotionThread()
        {               
            Debug.WriteLine("BufferedMotionThread()");

            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }            
        }
        */

        private void BufferedMotionThreadDemo()
        {
            //Movimiento inicial (Ir a home)
            Debug.WriteLine("Demo initial movement: Return to 0");
            Buffered_Motion.d_EndEffector_TargetPosX_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = Kinematics.InverseKinematics.d_Robot_ZPos_HomePosition;
            GUI.d_SetPoint_Speed_mm_s = 100.0;
            GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;
            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }

            while (bl_ExecuteInfiniteMovements)
            {
                //1st movement (Movimiento rapido)
                Debug.WriteLine("Demo movement: 1");
                Buffered_Motion.d_EndEffector_TargetPosX_mm = 75.0;
                Buffered_Motion.d_EndEffector_TargetPosY_mm = 75.0;
                Buffered_Motion.d_EndEffector_TargetPosZ_mm = -175.0;
                GUI.d_SetPoint_Speed_mm_s = 7500.0;
                GUI.d_SetPoint_Acceleration_mm_s_2 = 7500.0;
                Buffered_Motion.bl_Robot_Busy = true;
                Buffered_Motion.BufferedMovement(false);
                while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                { }

                //2nd movement (Movimiento extremo)
                Debug.WriteLine("Demo movement: 2");
                Buffered_Motion.d_EndEffector_TargetPosX_mm = -75.0;
                Buffered_Motion.d_EndEffector_TargetPosY_mm = -75.0;
                Buffered_Motion.d_EndEffector_TargetPosZ_mm = -240.0;
                GUI.d_SetPoint_Speed_mm_s = 400.0;
                GUI.d_SetPoint_Acceleration_mm_s_2 = 1000.0;
                Buffered_Motion.bl_Robot_Busy = true;
                Buffered_Motion.BufferedMovement(false);
                while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                { }

                //3rd movement
                Debug.WriteLine("Demo movement: 3");
                Buffered_Motion.d_EndEffector_TargetPosX_mm = 60.0;
                Buffered_Motion.d_EndEffector_TargetPosY_mm = -10.0;
                Buffered_Motion.d_EndEffector_TargetPosZ_mm = -180.0;
                GUI.d_SetPoint_Speed_mm_s = 2000.0;
                GUI.d_SetPoint_Acceleration_mm_s_2 = 1500.0;
                Buffered_Motion.bl_Robot_Busy = true;
                Buffered_Motion.BufferedMovement(false);
                while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                { }

                //4th movement (Movimiento vertical)
                Debug.WriteLine("Demo movement: 4");
                Buffered_Motion.d_EndEffector_TargetPosX_mm = 60.0;
                Buffered_Motion.d_EndEffector_TargetPosY_mm = -10.0;
                Buffered_Motion.d_EndEffector_TargetPosZ_mm = -275.0;
                GUI.d_SetPoint_Speed_mm_s = 40.0;
                GUI.d_SetPoint_Acceleration_mm_s_2 = 50.0;
                Buffered_Motion.bl_Robot_Busy = true;
                Buffered_Motion.BufferedMovement(false);
                while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                { }

                //5th movement
                Debug.WriteLine("Demo movement: 5");
                Buffered_Motion.d_EndEffector_TargetPosX_mm = -75.0;
                Buffered_Motion.d_EndEffector_TargetPosY_mm = 75.0;
                Buffered_Motion.d_EndEffector_TargetPosZ_mm = -220.0;
                GUI.d_SetPoint_Speed_mm_s = 2500.0;
                GUI.d_SetPoint_Acceleration_mm_s_2 = 1250.0;
                Buffered_Motion.bl_Robot_Busy = true;
                Buffered_Motion.BufferedMovement(false);
                while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                { }

                //6th movement (Movimiento horizontal)
                Debug.WriteLine("Demo movement: 6");
                Buffered_Motion.d_EndEffector_TargetPosX_mm = 75.0;
                Buffered_Motion.d_EndEffector_TargetPosY_mm = -75.0;
                Buffered_Motion.d_EndEffector_TargetPosZ_mm = -220.0;
                GUI.d_SetPoint_Speed_mm_s = 40.0;
                GUI.d_SetPoint_Acceleration_mm_s_2 = 50.0;
                Buffered_Motion.bl_Robot_Busy = true;
                Buffered_Motion.BufferedMovement(false);
                while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
                { }
            }

            //Movimiento final (Retornar a home)
            Debug.WriteLine("Demo final movement: Return to 0");
            Buffered_Motion.d_EndEffector_TargetPosX_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = Kinematics.InverseKinematics.d_Robot_ZPos_HomePosition;
            GUI.d_SetPoint_Speed_mm_s = 100.0;
            GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;
            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }

            //bufferedMotionThread_Demo.Abort();  //Finalizar hilo (No sirve en .NET 6)

        }

        //************************************************************ BUTTONS ************************************************************************

        // https://docs.microsoft.com/en-us/dotnet/desktop/wpf/app-development/dialog-boxes-overview?view=netframeworkdesktop-4.8
        private void Btn_OpenFile_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_OpenFile_Click()");

            /*
            Debug.WriteLine("************************ READ EXCEL FILE *****************************");

            //string strDoc = @"C:\Users\Public\Documents\Sheet11.xlsx";
            OpenFileDialog openFileDialog = new OpenFileDialog();
            openFileDialog.ShowDialog();   //Pop up file open window and get file path
            string path = openFileDialog.FileName.ToString();
            excel.ExcelFileReader(path);    //Read excel data

            double[] d_MotorsAngles = new double[3];
            double[] d_DesiredPositionXYZArray = new double[3];

            //Calculate all position data on excel file
            for (int i = 0; i < excel.i_PathPlanning_QtyData; i++)
            {
                Debug.WriteLine("****************** POSITION " + i + " *******************");
                ;
                Debug.WriteLine("X = " + excel.d_DesiredPositionXYZ[i, 0] + " Y = " + excel.d_DesiredPositionXYZ[i, 1] + " Z = " + excel.d_DesiredPositionXYZ[i, 2]);

                d_DesiredPositionXYZArray[0] = excel.d_DesiredPositionXYZ[i, 0];    //Get X position
                d_DesiredPositionXYZArray[1] = excel.d_DesiredPositionXYZ[i, 1];    //Get Y position
                d_DesiredPositionXYZArray[2] = excel.d_DesiredPositionXYZ[i, 2];    //Get Z position

                //Calculate inverse kinematics
                bool bl_Valid_Result_InvKinematics;
                d_MotorsAngles = Kinematics.InverseKinematics.CalculateInverseKinematics(d_DesiredPositionXYZArray);

                //Show invert kinematics result
                Debug.WriteLine("IK: Axis1[°] = " + d_MotorsAngles[0]);
                Debug.WriteLine("IK: Axis2[°] = " + d_MotorsAngles[1]);
                Debug.WriteLine("IK: Axis3[°] = " + d_MotorsAngles[2]);

                //Save kinematics invert result on excel file
                excel.ExcelFileWriter(path, d_MotorsAngles, (i + 3), 5);  //Fist row to save = 3, fist column to save = 5                
                            
                //******************** EXECUTE MOVEMENT ************************
                //Calculate motor turns
                //double[] d_MotorsTurns = new double[3];
                //d_MotorsTurns[0] = d_MotorsAngles[0] / 360.0;
                //d_MotorsTurns[1] = d_MotorsAngles[1] / 360.0;
                //d_MotorsTurns[2] = d_MotorsAngles[2] / 360.0;

                //motionSequence = new LinearMotionSequence(d_MotorsAngles, d_SetPoint_Speed, d_SetPoint_Acceleration);
                //motionSequence = new LinearMotionSequence(d_MotorsTurns, d_SetPoint_Speed, d_SetPoint_Acceleration);

                //motionSequence.ExecuteAsync();  //Execute interpolated linear movement       
                
            }
            MessageBox.Show("Cinematica inversa calculada correctamente");
            */
        }

        //Connect to motion controller
        private void Btn_Connect_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_Connect_Click()");

            if (MotionControl.bl_ControllerOnline)   //Controller online? -> Disconnect controller
            {
                MotionControl.Disconnect();

                //Check if TCP comms are open
                if (TCP_Comms.bl_TXClientConnected)
                {
                    TCP_Comms.TCP_Disconnect();
                }
            }
            else // -> Connect controller 
            {
                MotionControl.Connect();
            }
        }

        //Reset alarms 
        private void Btn_Reset_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_Reset_Click()");
        }

        //Create TCP server to connect to MCD or Matlab
        private void Btn_TCP_Connect_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_TCP_Connect_Click()");
            TCP_Comms.TCP_Connect();  //Mechatronics concept designer comms (Complete CFS exercise: Pick and place factory)
            //TCP_Comms2.TCP_Connect();   //Mechatronics concept designer comms (Only basic delta robot)
        }

        //Close TCP server and go home (MCD and Matlab connection)
        private void Btn_TCP_Disconnect_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_TCP_Disconnect_Click()");
            TCP_Comms.TCP_Disconnect();  //Mechatronics concept designer comms (Complete CFS exercise: Pick and place factory)
            //TCP_Comms2.TCP_Disconnect();   //Mechatronics concept designer comms (Only basic delta robot)
        }


        private void Btn_SetInitialPosition_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_SetInitialPosition_Click()");
            
            /*
            //Correct final position (Absolute movement)
            Debug.WriteLine("Correct final position");
            //Calculate motor turns
            
            //double[] d_MotorsTurns = new double[3];
            //d_MotorsTurns[0] = 360 / 360.0 * MotionControl.d_ReducerRelation;
            //d_MotorsTurns[1] = 90 / 360.0 * MotionControl.d_ReducerRelation;
            //d_MotorsTurns[2] = 720 / 360.0 * MotionControl.d_ReducerRelation;
            
            double[] d_MotorsTurns = new double[3];
            
            //d_MotorsTurns[0] = buffer_turns[242, 0];
            //d_MotorsTurns[1] = buffer_turns[242, 1];
            //d_MotorsTurns[2] = buffer_turns[242, 2];
                        
            //d_MotorsTurns[0] = -0.12098662324078166;
            //d_MotorsTurns[1] = -0.022778326117180826;
            //d_MotorsTurns[2] = 0.1874011069207189;
            
            d_MotorsTurns[0] = -0.12098;
            d_MotorsTurns[1] = -0.02277;
            d_MotorsTurns[2] = 0.18740;

            Debug.WriteLine("d_MotorsTurns[0]] = " + d_MotorsTurns[0] + " = " + d_MotorsTurns[0] * 360.0 + " deg");
            Debug.WriteLine("d_MotorsTurns[1]] = " + d_MotorsTurns[1] + " = " + d_MotorsTurns[1] * 360.0 + " deg");
            Debug.WriteLine("d_MotorsTurns[2]] = " + d_MotorsTurns[2] + " = " + d_MotorsTurns[2] * 360.0 + " deg");


            if (MotionTimers.motionSequence != null)
            {
                MotionTimers.motionSequence.Cancel();
            }

            //motionSequence = new LinearMotionSequence(d_MotorsAngles, d_SetPoint_Speed, d_SetPoint_Acceleration);
            MotionTimers.motionSequence = new LinearMotionSequence(d_MotorsTurns, 100, 7.5);

            MotionTimers.motionSequence.ExecuteAsync();  //Execute interpolated linear movement    
            */
        }

        private void Btn_GoToZeroPosition_Click(object sender, RoutedEventArgs e)
        {
            double[] d_MotorsTurns = new double[3];
            d_MotorsTurns[0] = 0.0;
            d_MotorsTurns[1] = 0.0;
            d_MotorsTurns[2] = 0.0;

            //Realizar interpolacion de los 3 ejes hasta la posicion 0°, 0°, 0° de los brazos activos
            MotionTimers.motionSequence = new LinearMotionSequence(d_MotorsTurns, 10.0, 10.0);    //(d_MotorsAngles, d_SetPoint_Speed, d_SetPoint_Acceleration);

            MotionTimers.motionSequence.ExecuteAsync();  //Execute interpolated movement    
        }

        private void Btn_ArtificialVision_Start_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_ArtificialVision_Start_Click()");

            // Pruebas pick and place con vision artificial 
            if (RS232_Comms.RS232_Opened() == false)
            {
                RS232_Comms.RS232_Setup();    //Inicializar comunicacion RS232 con sistema vision artificial
            }

            RS232_Comms.RS232_TX("Start");
        }
            
        private void Btn_Test_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_Test_Click()");

            /*
            double[] d_Test = new double[3];
            d_Test[0] = 10.0;
            d_Test[1] = 20.0;
            d_Test[2] = -175.0;

            Kinematics.InverseKinematics.CalculateInverseKinematics(d_Test, true);
            */

            /*
            Buffered_Motion.d_EndEffector_TargetPosX_mm = GUI.d_EndEffectorPosition_mm[0] + 1.0;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = GUI.d_EndEffectorPosition_mm[1];
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = GUI.d_EndEffectorPosition_mm[2];
            GUI.d_SetPoint_Speed_mm_s = 100.0;
            GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;

            var motion_Thread = new Motion_Thread();                

            bufferedMotionThread = new Thread(new ThreadStart(motion_Thread.BufferedMotionThread));   //Crear hilo adicional para realizar movimiento con buffer
            bufferedMotionThread.Start();   //Ejecutar movimiento en otro hilo           
            */
        }
                
        private void Btn_DemoRoutine_Start_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("btn_DemoRoutine_Start_Click()");

            bl_ExecuteInfiniteMovements = true;

            bufferedMotionThread_Demo = new Thread(new ThreadStart(BufferedMotionThreadDemo));   //Crear hilo adicional para realizar movimiento con buffer
            bufferedMotionThread_Demo.Start();   //Ejecutar movimiento en otro hilo

            GUI.Form.btn_DemoRoutine_Start.Visibility = Visibility.Hidden;
            GUI.Form.btn_DemoRoutine_Stop.Visibility = Visibility.Visible;
        }

        private void Btn_DemoRoutine_Stop_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("btn_DemoRoutine_Stop_Click()");

            bl_ExecuteInfiniteMovements = false;

            GUI.Form.btn_DemoRoutine_Start.Visibility = Visibility.Visible;
            GUI.Form.btn_DemoRoutine_Stop.Visibility = Visibility.Hidden;            
        }

        /*
        private void Btn_ChangeSetPointsToPos_Click(object sender, RoutedEventArgs e)
        {
            GUI.ChangeSetPointsToPos();
        }

        private void Btn_ChangeSetPointsToNeg_Click(object sender, RoutedEventArgs e)
        {
            GUI.ChangeSetPointsToNeg();
        }
        */

        
        private void Btn_XYZInterpolatedManualMovement_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("XYZInterpolatedManualMovement_Click()");

            //GUI.GetSetPoints();
            //MotionBasicMovements.i_AxisSelected = Form.cb_AxisSelection.SelectedIndex;    //Get selected axis to move   

            double d_EndEffector_TargetPosX_mm = Convert.ToDouble(GUI.Form.tb_SetPoint_XPosition_mm.Text);
            double d_EndEffector_TargetPosY_mm = Convert.ToDouble(GUI.Form.tb_SetPoint_YPosition_mm.Text);
            double d_EndEffector_TargetPosZ_mm = Convert.ToDouble(GUI.Form.tb_SetPoint_ZPosition_mm.Text);
            double d_SetPoint_Speed_mm_s = Convert.ToDouble(GUI.Form.tb_SetPoint_Speed_mm_s.Text);
            double d_SetPoint_Acceleration_mm_s_2 = Convert.ToDouble(GUI.Form.tb_SetPoint_Acceleration_mm_s_2.Text);           

            //Ejecutar movimiento lineal interpolado
            Buffered_Motion.Robot_Linear_Interpolation(d_EndEffector_TargetPosX_mm, d_EndEffector_TargetPosY_mm, d_EndEffector_TargetPosZ_mm, d_SetPoint_Speed_mm_s, d_SetPoint_Acceleration_mm_s_2);
        }

        private void Btn_TCP_Send_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_TCP_Send_Click()");
            //TCP_Comms.TCP_Send();
        }

        private void Btn_Arm_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_Arm_Click()");

            if (MotionControl.bl_ControllerArmed)    //Controller armed? -> Disarm controller   
            {
                MotionControl.Disarm();

                //Check if TCP comms are open
                if (TCP_Comms.bl_TXClientConnected)
                {
                    TCP_Comms.TCP_Disconnect();
                }
            }
            else //Arm controller
            {
                MotionControl.Arm();
            }

            /*
            // Si no esta armado -> Armar y esperar 25ms (No es necesario!!)
            if (!HiCONDLL.vsiStatusIsArmed())
            {
                HiCONDLL.vsiCmdArm((uint)HiCONDLL.AxisMask.ALL);
                System.Threading.Thread.Sleep(25);
            }
            */
        }
        
        private void Btn_StopMovement_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_StopMovement_Click()");
            MotionBasicMovements.Stop();
        }

        private void Btn_StopAllMovements_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_StopAllMovements_Click()");
            MotionBasicMovements.StopAll();
        }

        private void Btn_SetAllAxesPositionsToZero_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_SetAllAxesPositionsToZero_Click()");
            MotionBasicMovements.SetAllAxesPositionsToZero();
        }

        private void Btn_GoHome_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_GoHome_Click()");
            //MotionBasicMovements.GoHome();
        }

        //JOG: Positive movement
        private void Btn_JOGPosMove_PreviewMouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_JOGPosMove_PreviewMouseLeftButtonDown()");
            //MotionBasicMovements.JOGPosMove();

            var jogPosMove = new MotionBasicMovements();
            jogPosMove.JOGPosMove();
        }

        //JOG: Stop positive movement
        private void Btn_JOGPosMove_PreviewMouseLeftButtonUp(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_JOGPosMove_PreviewMouseLeftButtonUp()");
            MotionBasicMovements.JOGPosStop();
        }

        //JOG: Negative movement
        private void Btn_JOGNegMove_PreviewMouseLeftButtonDown(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_JOGNegMove_PreviewMouseLeftButtonDown()");
            //MotionBasicMovements.JOGNegMove();

            var jogPosMove = new MotionBasicMovements();
            jogPosMove.JOGNegMove();
        }

        //JOG: Stop negative movement
        private void Btn_JOGNegMove_PreviewMouseLeftButtonUp(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_JOGNegMove_PreviewMouseLeftButtonUp()");
            MotionBasicMovements.JOGNegStop();
        }

        private void Tbt_ToggleDigitalOutput_Checked(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Tbt_ToggleDigitalOutput_Checked()");            
        }

        private void Tbt_ToggleDigitalOutput_Unchecked(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Tbt_ToggleDigitalOutput_Unchecked()");            
        }

        //Decode and execute GCode
        private void Btn_GCode_Click(object sender, RoutedEventArgs e)
        {
            Debug.WriteLine("Btn_GCode_Click()");

            GCodes.GCode_Decoder();           

            /*
            //Update calculated data
            tb_SetPoint_XPosition_mm.Text = gCodes.s_SetPoint_Positions[0];
            tb_SetPoint_YPosition_mm.Text = gCodes.s_SetPoint_Positions[1];
            tb_SetPoint_ZPosition_mm.Text = gCodes.s_SetPoint_Positions[2];
            tb_SetPoint_Speed.Text = gCodes.s_SetPoint_Speed;
            */
        }

        // KEYBOARD DETECTION
        
        //Key pressed
        private void Window_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.Key)
            {
                //Motor 0
                case Key.Q:
                    Debug.WriteLine("Boton Q presionado");                    
                    MotionBasicMovements.i_AxisSelected_Keyboard = 0;
                    MotionBasicMovements.JOGNegMove_Keyboard();
                    break;
                case Key.A:                    
                    Debug.WriteLine("Boton A presionado");                    
                    MotionBasicMovements.i_AxisSelected_Keyboard = 0;
                    MotionBasicMovements.JOGPosMove_Keyboard();
                    break;

                //Motor 1
                case Key.W:
                    Debug.WriteLine("Boton W presionado");                    
                    MotionBasicMovements.i_AxisSelected_Keyboard = 1;
                    MotionBasicMovements.JOGNegMove_Keyboard();
                    break;
                case Key.S:
                    Debug.WriteLine("Boton S presionado");                    
                    MotionBasicMovements.i_AxisSelected_Keyboard = 1;
                    MotionBasicMovements.JOGPosMove_Keyboard();
                    break;

                //Motor 2
                case Key.E:
                    Debug.WriteLine("Boton E presionado");                    
                    MotionBasicMovements.i_AxisSelected_Keyboard = 2;
                    MotionBasicMovements.JOGNegMove_Keyboard();
                    break;
                case Key.D:
                    Debug.WriteLine("Boton D presionado");                    
                    MotionBasicMovements.i_AxisSelected_Keyboard = 2;
                    MotionBasicMovements.JOGPosMove_Keyboard();
                    break;
            }
        }

        //Key released
        private void Window_KeyUp(object sender, KeyEventArgs e)
        {
            switch (e.Key)
            {
                //Motor 0
                case Key.Q:
                    Debug.WriteLine("Boton Q liberado");
                    MotionBasicMovements.JOGNegStop_Keyboard();                                        
                    break;
                case Key.A:
                    Debug.WriteLine("Boton A liberado");
                    MotionBasicMovements.JOGPosStop_Keyboard();                    
                    break;

                //Motor 1
                case Key.W:
                    Debug.WriteLine("Boton W liberado");
                    MotionBasicMovements.JOGNegStop_Keyboard();                    
                    break;
                case Key.S:
                    Debug.WriteLine("Boton S liberado");
                    MotionBasicMovements.JOGPosStop_Keyboard();                    
                    break;

                //Motor 2
                case Key.E:
                    Debug.WriteLine("Boton E liberado");
                    MotionBasicMovements.JOGNegStop_Keyboard();                    
                    break;
                case Key.D:
                    Debug.WriteLine("Boton D liberado");
                    MotionBasicMovements.JOGPosStop_Keyboard();                    
                    break;
            }
        }
    }
}
