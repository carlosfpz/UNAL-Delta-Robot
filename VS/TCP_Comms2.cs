using System;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Timers;

namespace DeltaRobot_WPF_NetCore
{
    public static class TCP_Comms2
    {
        //*********************** PUBLIC VARIABLES ******************************

        //*********************** PRIVATE VARIABLES ******************************
        private static TcpListener server;
        private static TcpClient client;
        private static NetworkStream stream;
        private static System.Timers.Timer timer_TCPSendData;
        private static Thread tcp_Comms_Thread;    //Hilo de ejecucion de comunicacion TCP

        private static void TCP_Comms_Thread()
        {
            //TcpListener server = new TcpListener(IPAddress.Parse("127.0.0.1"), 6000);
            server = new TcpListener(IPAddress.Parse("127.0.0.1"), 6000);

            server.Start();
            Debug.WriteLine("El server se ha iniciado en 127.0.0.1:6000. Esperando una conexión...", Environment.NewLine);

            //TcpClient client = server.AcceptTcpClient();
            client = server.AcceptTcpClient();

            Debug.WriteLine("Un cliente conectado.");

            // TCP send data
            stream = client.GetStream();

            SetTimer_TCPSendData();  //Activate periodic controller TCP data sending
        }

        public static void TCP_Connect()
        {
            tcp_Comms_Thread = new Thread(new ThreadStart(TCP_Comms_Thread));   //Crear hilo adicional para realizar comunicaciones por TCP
            tcp_Comms_Thread.Start();   //Ejecutar comunicacion TCP en otro hilo           
        }

        public static void TCP_Send()
        {
            stream = client.GetStream();

            SetTimer_TCPSendData();  //Activate periodic controller TCP data sending
        }

        //TIMER CONTROLLER TCP SENDING DATA (Timer)
        private static void SetTimer_TCPSendData()
        {
            timer_TCPSendData = new System.Timers.Timer(10);   //Peridic timer in ms     
            timer_TCPSendData.Elapsed += Timer_TCPSendData_Event;
            timer_TCPSendData.AutoReset = true;
            timer_TCPSendData.Enabled = true;
        }

        private static void Timer_TCPSendData_Event(Object source, ElapsedEventArgs e)
        {
            //Debug.WriteLine("Timer_TCPSendData_Event");

            byte[] bDato1 = new byte[8];
            byte[] bDato2 = new byte[8];
            byte[] bDato3 = new byte[8];

            bDato1 = ConvertDoubleToByteArray(GUI.d_Arms_Q_Positions_Deg[0]);
            bDato2 = ConvertDoubleToByteArray(GUI.d_Arms_Q_Positions_Deg[1]);
            bDato3 = ConvertDoubleToByteArray(GUI.d_Arms_Q_Positions_Deg[2]);

            byte[] bSendBytes = new byte[24];

            // Concatenate 3 double data to byte array[24]
            System.Buffer.BlockCopy(bDato1, 0, bSendBytes, 0, bDato1.Length);
            System.Buffer.BlockCopy(bDato2, 0, bSendBytes, bDato1.Length, bDato2.Length);
            System.Buffer.BlockCopy(bDato3, 0, bSendBytes, bDato1.Length + bDato2.Length, bDato3.Length);

            //bSendBytes = System.Text.Encoding.ASCII.GetBytes(bSendBytes);
            //Debug.WriteLine("bSendBytes = " + bSendBytes);

            stream.Write(bSendBytes, 0, 24);
        }

        private static byte[] ConvertDoubleToByteArray(double d)
        {
            return BitConverter.GetBytes(d);
        }
    }
}
