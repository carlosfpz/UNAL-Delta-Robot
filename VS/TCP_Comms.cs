using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using VSI;
using System.Windows;

namespace DeltaRobot_WPF_NetCore
{
    public static class TCP_Comms
    {
        //*********************** PUBLIC VARIABLES ******************************

        //*********************** PRIVATE VARIABLES ******************************
        private static TcpListener serverTX, serverRX;
        private static TcpClient clientTX, clientRX;
        private static NetworkStream streamTX, streamRX;
        private static System.Timers.Timer timer_TCPSendData;
        private static System.Timers.Timer timer_TCPReceiveData;        
        private static Thread tcp_CommsRX_Thread;    //Hilo de ejecucion de comunicacion RX TCP
        private static Thread tcp_CommsTX_Thread;    //Hilo de ejecucion de comunicacion TX TCP
        private static byte[] bLastDataRX = new byte[1];
        //private static string sIPAddress_RX = "192.168.1.51";
        //private static string sIPAddress_TX = "192.168.1.51";
        private static string sIPAddress_RX = "127.0.0.1";
        private static string sIPAddress_TX = "127.0.0.1";        

        private static Socket s;        

        //POSITIONS
        //Row: 0 = Ready position / 1 = Get position / 2 = Middle position / 3 = Release position
        //Columns: 0 = X coordinate / 1 = Y coordinate / 2 = Z coordinate
        private static double[,] d_PickAndPlaceSetPositions = new double[4, 3];
        private static double[,] d_PickAndPlacePositions_Dice = new double[4, 3];
        private static double[,] d_PickAndPlacePositions_Car = new double[4, 3];
        private static double[,] d_PickAndPlacePositions_Cup = new double[4, 3];

        //SPEED
        //Row: 0 = Dice speeds / 1 = Car speeds / 2 = Cup speeds
        //Columns: 0 = Going to ready position / 1 = Going to get position / 2 = Going to middle position / 4 = Going to release position
        private static double[,] d_PickAndPlace_Speed = new double [3, 4];

        //ACCELERATIONS
        //Row: 0 = Dice accels / 1 = Car accels / 2 = Cup accels
        //Columns: 0 = Going to ready position / 1 = Going to get position / 2 = Going to middle position / 4 = Going to release position
        private static double[,] d_PickAndPlace_Accel = new double[3, 4];

        private static bool bl_GripperStatus = false;

        public static bool bl_RXClientConnected = false;
        public static bool bl_TXClientConnected = false;

        //*********************** PRIVATE VARIABLES ******************************
        public static MainWindow Form = Application.Current.Windows[0] as MainWindow;
        //Bear in mind the array! Ensure it's the correct Window you're trying to catch.
                

        public static void TCP_Disconnect_And_Go_Home()
        {
            TCP_Disconnect();            
            
            //Go to home position
            double[] d_MotorsTurns = new double[3];
            d_MotorsTurns[0] = 0.0;
            d_MotorsTurns[1] = 0.0;
            d_MotorsTurns[2] = 0.0;

            //Realizar interpolacion de los 3 ejes hasta la posicion 0°, 0°, 0° de los brazos activos
            MotionTimers.motionSequence = new LinearMotionSequence(d_MotorsTurns, 10.0, 10.0);    //(d_MotorsAngles, d_SetPoint_Speed, d_SetPoint_Acceleration);

            MotionTimers.motionSequence.ExecuteAsync();  //Execute interpolated movement    
        }

        public static void TCP_Disconnect()
        {
            Form.btn_TCP_Connect.Visibility = Visibility.Visible;
            Form.btn_TCP_Disconnect.Visibility = Visibility.Hidden;

            bl_RXClientConnected = false;
            bl_TXClientConnected = false;

            //Close comms
            streamTX.Close();
            streamRX.Close();

            clientTX.Close();
            clientRX.Close();

            MotionBasicMovements.StopAll();          
        }


        public static void TCP_Connect()
        {
            Form.btn_TCP_Connect.Visibility = Visibility.Hidden;
            Form.btn_TCP_Disconnect.Visibility = Visibility.Visible;

            //******************************************************** SET PICK AND PLACE POSITIONS *******************************************

            //***************** DICE **********************
            //Ready position
            d_PickAndPlacePositions_Dice[0, 0] = 0.0;   //X coordinate
            d_PickAndPlacePositions_Dice[0, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Dice[0, 2] = -252.5;   //Z coordinate

            //Get position
            d_PickAndPlacePositions_Dice[1, 0] = 0.0;   //X coordinate
            d_PickAndPlacePositions_Dice[1, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Dice[1, 2] = -307.879;   //Z coordinate

            //Middle position
            d_PickAndPlacePositions_Dice[2, 0] = -52.763;   //X coordinate
            d_PickAndPlacePositions_Dice[2, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Dice[2, 2] = -275.0;   //Z coordinate

            //Place position
            d_PickAndPlacePositions_Dice[3, 0] = -105.526;   //X coordinate
            d_PickAndPlacePositions_Dice[3, 1] = 0.5;   //Y coordinate
            d_PickAndPlacePositions_Dice[3, 2] = -290.0;   //Z coordinate            

            //***************** CAR **********************
            //Ready position
            d_PickAndPlacePositions_Car[0, 0] = 0.0;   //X coordinate
            d_PickAndPlacePositions_Car[0, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Car[0, 2] = -252.5;   //Z coordinate

            //Get position
            d_PickAndPlacePositions_Car[1, 0] = 0.0;   //X coordinate
            d_PickAndPlacePositions_Car[1, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Car[1, 2] = -315.932;   //Z coordinate

            //Middle position
            d_PickAndPlacePositions_Car[2, 0] = 17.297;   //X coordinate
            d_PickAndPlacePositions_Car[2, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Car[2, 2] = -195.0;   //Z coordinate

            //Place position
            d_PickAndPlacePositions_Car[3, 0] = 17.297;   //X coordinate
            d_PickAndPlacePositions_Car[3, 1] = -77.541;   //Y coordinate
            d_PickAndPlacePositions_Car[3, 2] = -195.0;   //Z coordinate

            //***************** CUP **********************
            //Ready position
            d_PickAndPlacePositions_Cup[0, 0] = 0.0;   //X coordinate
            d_PickAndPlacePositions_Cup[0, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Cup[0, 2] = -252.5;   //Z coordinate

            //Get position
            d_PickAndPlacePositions_Cup[1, 0] = 0.0;   //X coordinate
            d_PickAndPlacePositions_Cup[1, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Cup[1, 2] = -257.879;   //Z coordinate

            //Middle position
            d_PickAndPlacePositions_Cup[2, 0] = 60.0;   //X coordinate
            d_PickAndPlacePositions_Cup[2, 1] = 85.5;   //Y coordinate
            d_PickAndPlacePositions_Cup[2, 2] = -175.0;   //Z coordinate

            //Place position
            d_PickAndPlacePositions_Cup[3, 0] = 120.0;   //X coordinate
            d_PickAndPlacePositions_Cup[3, 1] = 0.5;   //Y coordinate
            //d_PickAndPlacePositions_Cup[3, 1] = 0.202;   //Y coordinate (Con este valor falla)
            d_PickAndPlacePositions_Cup[3, 2] = -255.0;   //Z coordinate

            //******************************************* SET SPEEDS AND ACCELS **************************************

            //Dice
            /*
            d_PickAndPlace_Speed[0, 0] = 2000.0;    //Speed going to Ready position
            d_PickAndPlace_Speed[0, 1] = 7200.0;    //Speed going to pick position
            d_PickAndPlace_Speed[0, 2] = 2000.0;    //Speed going to middle position
            d_PickAndPlace_Speed[0, 3] = 2000.0;     //Speed going to place position

            d_PickAndPlace_Accel[0, 0] = 2000.0;    //Accel going to Ready position
            d_PickAndPlace_Accel[0, 1] = 7200.0;    //Accel going to pick position
            d_PickAndPlace_Accel[0, 2] = 2000.0;    //Accel going to middle position
            d_PickAndPlace_Accel[0, 3] = 2000.0;     //Accel going to place position
            */
            d_PickAndPlace_Speed[0, 0] = 1500.0;    //Speed going to Ready position
            d_PickAndPlace_Speed[0, 1] = 11500.0;    //Speed going to pick position
            d_PickAndPlace_Speed[0, 2] = 1500.0;    //Speed going to middle position
            d_PickAndPlace_Speed[0, 3] = 1500.0;     //Speed going to place position

            d_PickAndPlace_Accel[0, 0] = 1500.0;    //Accel going to Ready position
            d_PickAndPlace_Accel[0, 1] = 11500.0;    //Accel going to pick position
            d_PickAndPlace_Accel[0, 2] = 1500.0;    //Accel going to middle position
            d_PickAndPlace_Accel[0, 3] = 1500.0;     //Accel going to place position

            //Car
            d_PickAndPlace_Speed[1, 0] = 500.0;    //Speed going to Ready position
            d_PickAndPlace_Speed[1, 1] = 1500.0;    //Speed going to pick position
            d_PickAndPlace_Speed[1, 2] = 500.0;    //Speed going to middle position
            d_PickAndPlace_Speed[1, 3] = 500.0;     //Speed going to place position

            d_PickAndPlace_Accel[1, 0] = 500.0;    //Accel going to Ready position
            d_PickAndPlace_Accel[1, 1] = 1500.0;    //Accel going to pick position
            d_PickAndPlace_Accel[1, 2] = 500.0;    //Accel going to middle position
            d_PickAndPlace_Accel[1, 3] = 500.0;     //Accel going to place position

            //Cup
            /*
            d_PickAndPlace_Speed[2, 0] = 700.0;    //Speed going to Ready position
            d_PickAndPlace_Speed[2, 1] = 2000.0;    //Speed going to pick position
            d_PickAndPlace_Speed[2, 2] = 1000.0;    //Speed going to middle position
            d_PickAndPlace_Speed[2, 3] = 500.0;     //Speed going to place position

            d_PickAndPlace_Accel[2, 0] = 700.0;    //Accel going to Ready position
            d_PickAndPlace_Accel[2, 1] = 2000.0;    //Accel going to pick position
            d_PickAndPlace_Accel[2, 2] = 1000.0;    //Accel going to middle position
            d_PickAndPlace_Accel[2, 3] = 500.0;     //Accel going to place position
            */

            d_PickAndPlace_Speed[2, 0] = 750.0;    //Speed going to Ready position
            d_PickAndPlace_Speed[2, 1] = 2250.0;    //Speed going to pick position
            d_PickAndPlace_Speed[2, 2] = 400.0;    //Speed going to middle position
            d_PickAndPlace_Speed[2, 3] = 400.0;     //Speed going to place position

            d_PickAndPlace_Accel[2, 0] = 750.0;    //Accel going to Ready position
            d_PickAndPlace_Accel[2, 1] = 2250.0;    //Accel going to pick position
            d_PickAndPlace_Accel[2, 2] = 400.0;    //Accel going to middle position
            d_PickAndPlace_Accel[2, 3] = 400.0;     //Accel going to place position

            //******************************************* GO TO READY POSITION **************************************
            /*
            //Go to ready position            
            Debug.WriteLine("MCD - Pick and place: Go to ready position");            
            //Pos X, Pos Y, Pos Z, Speed, Accel
            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Dice[0, 0], d_PickAndPlacePositions_Dice[0, 1], d_PickAndPlacePositions_Dice[0, 2], 25.0, 25.0);
            */

            //1st movement (Movimiento rapido)
            Debug.WriteLine("Go to ready position");
            Buffered_Motion.d_EndEffector_TargetPosX_mm = 0.0;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = 85.5;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = -250.0;
            GUI.d_SetPoint_Speed_mm_s = 100.0;
            GUI.d_SetPoint_Acceleration_mm_s_2 = 100.0;
            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }

            //********************************************** START COMMS ********************************************

            //Crear hilo adicional para realizar transmision de comunicaciones por TCP
            tcp_CommsTX_Thread = new Thread(new ThreadStart(TCP_CommsTX_Thread));   //Crear hilo adicional para realizar comunicaciones por TCP
            tcp_CommsTX_Thread.Start();   //Ejecutar comunicacion TCP en otro hilo   

            //Crear hilo adicional para realizar recepcion de comunicaciones por TCP
            tcp_CommsRX_Thread = new Thread(new ThreadStart(TCP_CommsRX_Thread));   //Crear hilo adicional para realizar comunicaciones por TCP
            tcp_CommsRX_Thread.Start();   //Ejecutar comunicacion TCP en otro hilo
        }

        //*****************************************************************************************************************************************
        //*****************************************************************************************************************************************
        //************************************************************ TX TCP *********************************************************************
                

        //TIMER CONTROLLER TCP SENDING DATA (Timer)
        private static void TCP_CommsTX_Thread()
        {            
            serverTX = new TcpListener(IPAddress.Parse(sIPAddress_TX), 6001);

            serverTX.Start();
            Debug.WriteLine("El server TX se ha iniciado en " + sIPAddress_TX + ":6001. Esperando una conexión...", Environment.NewLine);

            //TcpClient client = server.AcceptTcpClient();
            clientTX = serverTX.AcceptTcpClient();

            Debug.WriteLine("Un cliente TX conectado");
            bl_TXClientConnected = true;

            // TCP send data
            streamTX = clientTX.GetStream();

            timer_TCPSendData = new System.Timers.Timer(10);   //Peridic timer in ms
            //timer_TCPSendData = new System.Timers.Timer(1);   //Peridic timer in ms     
            timer_TCPSendData.Elapsed += Timer_TCPSendData_Event;
            timer_TCPSendData.AutoReset = true;
            timer_TCPSendData.Enabled = true;
        }

        private static void Timer_TCPSendData_Event(Object source, ElapsedEventArgs e)
        {            
            try
            {
                if (bl_TXClientConnected)
                {
                    int iQtyBytesToSend = 25;

                    //Debug.WriteLine("Timer_TCPSendData_Event");

                    byte[] bDato1 = new byte[8];
                    byte[] bDato2 = new byte[8];
                    byte[] bDato3 = new byte[8];
                    byte[] bDato4 = new byte[1];

                    bDato1 = ConvertDoubleToByteArray(GUI.d_Arms_Q_Positions_Deg[0]);
                    bDato2 = ConvertDoubleToByteArray(GUI.d_Arms_Q_Positions_Deg[1]);
                    bDato3 = ConvertDoubleToByteArray(GUI.d_Arms_Q_Positions_Deg[2]);

                    //Gripper status: 0x00000001 = Suck = 1d / 0x00000010 = Release = 2d

                    if (bl_GripperStatus)
                    {
                        bDato4[0] = Convert.ToByte(1);  //Gripper suck                   
                    }
                    else
                    {
                        bDato4[0] = Convert.ToByte(2);  //Gripper release
                    }

                    //byte[] bSendBytes = new byte[24];
                    byte[] bSendBytes = new byte[iQtyBytesToSend];

                    // Concatenate 3 double data to byte array[24]
                    System.Buffer.BlockCopy(bDato1, 0, bSendBytes, 0, bDato1.Length);
                    System.Buffer.BlockCopy(bDato2, 0, bSendBytes, bDato1.Length, bDato2.Length);
                    System.Buffer.BlockCopy(bDato3, 0, bSendBytes, bDato1.Length + bDato2.Length, bDato3.Length);
                    System.Buffer.BlockCopy(bDato4, 0, bSendBytes, bDato1.Length + bDato2.Length + bDato3.Length, bDato4.Length);

                    /*
                    bSendBytes = System.Text.Encoding.ASCII.GetBytes(bSendBytes[]);
                    Debug.WriteLine("bSendBytes = " + bSendBytes);
                    */

                    /*
                    //FUNCIONA PARA VER TODOS LOS DATOS EN HEXA
                    string sData = BitConverter.ToString(bSendBytes); //To convert the whole array
                    Debug.WriteLine("bSendBytes = " + sData);
                    */

                    /*
                    string sData = BitConverter.ToString(bSendBytes);
                    Debug.WriteLine("bSendBytes = " + sData);
                    */

                    //streamTX.Write(bSendBytes, 0, 24);

                    streamTX.Write(bSendBytes, 0, iQtyBytesToSend);
                }                
            }
            catch (Exception ex)
            {
                Debug.WriteLine("Error!\n" + ex.Message + "\n" + ex.StackTrace);                
            }             
        }

        //*****************************************************************************************************************************************
        //*****************************************************************************************************************************************
        //************************************************************ RX TCP *********************************************************************
               
        //TIMER CONTROLLER TCP RECEIVE DATA (Timer)
        private static void TCP_CommsRX_Thread()
        {                       
            serverRX = new TcpListener(IPAddress.Parse(sIPAddress_RX), 6000);
            
            serverRX.Start();
            Debug.WriteLine("El server RX se ha iniciado en " + sIPAddress_RX + ":6000. Esperando una conexión...", Environment.NewLine);

            //TcpClient client = server.AcceptTcpClient();
            clientRX = serverRX.AcceptTcpClient();

            Debug.WriteLine("Un cliente RX conectado");
            bl_RXClientConnected = true;

            // TCP send data
            streamRX = clientRX.GetStream();

            timer_TCPReceiveData = new System.Timers.Timer(1);   //Peridic timer in ms   
            //timer_TCPReceiveData = new System.Timers.Timer(10);   //Peridic timer in ms     
            timer_TCPReceiveData.Elapsed += Timer_TCPReceiveData_Event;
            timer_TCPReceiveData.AutoReset = true;
            timer_TCPReceiveData.Enabled = true;   
        }
        
        //MCD RX data (Objects sensor RX data)
        private static void Timer_TCPReceiveData_Event(Object source, ElapsedEventArgs e)
        {
            if (bl_RXClientConnected)
            {
                // Check to see if this NetworkStream is readable.
                if (streamRX.CanRead)
                {
                    byte[] b_BufferRX = new byte[100];
                    //StringBuilder myCompleteMessage = new StringBuilder();
                    //int numberOfBytesRead = 0;

                    int numberOfBytesRead;

                    numberOfBytesRead = streamRX.Read(b_BufferRX, 0, b_BufferRX.Length);    //Read TCP data


                    if (numberOfBytesRead == 1 && b_BufferRX[0] != 0 && b_BufferRX[0] != bLastDataRX[0])   //New object detection
                    {
                        Debug.WriteLine("******************************************************************************************");
                        Debug.WriteLine("******************************************************************************************");
                        Debug.WriteLine("******************************************************************************************");
                        Debug.WriteLine("******************************* New object detection *************************************");

                        //Reset last object detection after X time
                        Task.Factory.StartNew(() =>
                        {
                            System.Threading.Thread.Sleep(2000);
                            ResetLastObjectDetection();
                        });

                        Debug.WriteLine("RX Data: " + b_BufferRX[0]);
                        bLastDataRX[0] = b_BufferRX[0];

                        //Check what item was detected
                        if (b_BufferRX[0] == 1)
                        {

                            Debug.WriteLine("Rubik cube detection");
                        }
                        else if (b_BufferRX[0] == 2)
                        {
                            Debug.WriteLine("Dice detection");

                            //Go to pick position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to pick position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Dice[1, 0], d_PickAndPlacePositions_Dice[1, 1], d_PickAndPlacePositions_Dice[1, 2], d_PickAndPlace_Speed[0, 1], d_PickAndPlace_Accel[0, 1]);

                            bl_GripperStatus = true;

                            //Go to place position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to middle position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Dice[2, 0], d_PickAndPlacePositions_Dice[2, 1], d_PickAndPlacePositions_Dice[2, 2], d_PickAndPlace_Speed[0, 2], d_PickAndPlace_Accel[0, 2]);

                            //Go to middle position         
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to place position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Dice[3, 0], d_PickAndPlacePositions_Dice[3, 1], d_PickAndPlacePositions_Dice[3, 2], d_PickAndPlace_Speed[0, 3], d_PickAndPlace_Accel[0, 3]);
                                                        
                            bl_GripperStatus = false;

                            //Go to ready position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to ready position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Dice[0, 0], d_PickAndPlacePositions_Dice[0, 1], d_PickAndPlacePositions_Dice[0, 2], d_PickAndPlace_Speed[0, 0], d_PickAndPlace_Accel[0, 0]);                            
                        }
                        else if (b_BufferRX[0] == 4)
                        {
                            Debug.WriteLine("Car detection");

                            //Go to pick position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to pick position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Car[1, 0], d_PickAndPlacePositions_Car[1, 1], d_PickAndPlacePositions_Car[1, 2], d_PickAndPlace_Speed[1, 1], d_PickAndPlace_Accel[1, 1]);

                            bl_GripperStatus = true;
                                                        
                            //Go to middle position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to middle position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Car[2, 0], d_PickAndPlacePositions_Car[2, 1], d_PickAndPlacePositions_Car[2, 2], d_PickAndPlace_Speed[1, 2], d_PickAndPlace_Accel[1, 2]);
                                                        
                            //Go to place position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to place position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Car[3, 0], d_PickAndPlacePositions_Car[3, 1], d_PickAndPlacePositions_Car[3, 2], d_PickAndPlace_Speed[1, 3], d_PickAndPlace_Accel[1, 3]);
                                                        
                            bl_GripperStatus = false;

                            //Go to ready position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to ready position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Car[0, 0], d_PickAndPlacePositions_Car[0, 1], d_PickAndPlacePositions_Car[0, 2], d_PickAndPlace_Speed[1, 0], d_PickAndPlace_Accel[1, 0]);                                                                                    
                        }
                        else if (b_BufferRX[0] == 8)
                        {
                            Debug.WriteLine("Cup detection");

                            //Go to pick position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to pick position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Cup[1, 0], d_PickAndPlacePositions_Cup[1, 1], d_PickAndPlacePositions_Cup[1, 2], d_PickAndPlace_Speed[2, 1], d_PickAndPlace_Accel[2, 1]);

                            bl_GripperStatus = true;

                            //Go to middle position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to middle position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Cup[2, 0], d_PickAndPlacePositions_Cup[2, 1], d_PickAndPlacePositions_Cup[2, 2], d_PickAndPlace_Speed[2, 2], d_PickAndPlace_Accel[2, 2]);

                            //Go to place position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to place position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Cup[3, 0], d_PickAndPlacePositions_Cup[3, 1], d_PickAndPlacePositions_Cup[3, 2], d_PickAndPlace_Speed[2, 3], d_PickAndPlace_Accel[2, 3]);
                                                        
                            bl_GripperStatus = false;

                            //Go to ready position            
                            //Pos X, Pos Y, Pos Z, Speed, Accel
                            Debug.WriteLine("MCD - Pick and place: Go to ready position");
                            MCD_PickAndPlace_Movement(d_PickAndPlacePositions_Cup[0, 0], d_PickAndPlacePositions_Cup[0, 1], d_PickAndPlacePositions_Cup[0, 2], d_PickAndPlace_Speed[2, 0], d_PickAndPlace_Accel[2, 0]);                            
                        }
                    }


                    /*
                    // Incoming message may be larger than the buffer size.
                    do
                    {
                        numberOfBytesRead = streamRX.Read(myReadBuffer, 0, myReadBuffer.Length);

                        //myCompleteMessage.AppendFormat("{0}", Encoding.ASCII.GetString(myReadBuffer, 0, numberOfBytesRead));
                    }
                    while (streamRX.DataAvailable);
                    */

                    // Print out the received message to the console.
                    //Debug.WriteLine("You received the following message : " + myReadBuffer[0] + " cant datos: " + numberOfBytesRead);
                }
                else
                {
                    Debug.WriteLine("Sorry. You cannot read from this NetworkStream (streamRX)");
                }
            }                     
        }

        private static byte[] ConvertDoubleToByteArray(double d)
        {
            return BitConverter.GetBytes(d);
        }

        private static void MCD_PickAndPlace_Movement(double d_DesiredPosition_X, double d_DesiredPosition_Y, double d_DesiredPosition_Z, double d_Speed, double d_Accel)
        {
            //Go to ready position                        
            Debug.WriteLine("Desired position: X = " + d_DesiredPosition_X + " Y = " + d_DesiredPosition_Y + " Z = " + d_DesiredPosition_Z);
            Buffered_Motion.d_EndEffector_TargetPosX_mm = d_DesiredPosition_X;
            Buffered_Motion.d_EndEffector_TargetPosY_mm = d_DesiredPosition_Y;
            Buffered_Motion.d_EndEffector_TargetPosZ_mm = d_DesiredPosition_Z;
            GUI.d_SetPoint_Speed_mm_s = d_Speed;
            GUI.d_SetPoint_Acceleration_mm_s_2 = d_Accel;
            Buffered_Motion.bl_Robot_Busy = true;
            Buffered_Motion.BufferedMovement(false);
            while (Buffered_Motion.bl_Robot_Busy || HiCONDLL.vsiStatusIsMoving(-1))   //Esperar a que finalice el movimiento (Todos los ejes esten detenidos)
            { }
            Debug.WriteLine("Movement finished");
        }

        //If X time lapsed, reset last object detection (For detect 2 consecutive same object detection)
        private static void ResetLastObjectDetection()
        {
            Debug.WriteLine("ResetLastObjectDetection()");
            bLastDataRX[0] = 0; 
        }
    }
}
