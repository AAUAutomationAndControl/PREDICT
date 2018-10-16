using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Threading;
using System.Net;
using System.Net.Sockets;

namespace XPC
{
    class Program
    {

        static SerialPort serial_comm_1;


        static void Main(string[] args)
        {
            IPEndPoint XPip = new IPEndPoint(IPAddress.Any, 49001);
            UdpClient XPserver_receive = new UdpClient(XPip);
            UdpClient XPserver_send = new UdpClient("127.0.0.1", 49000);

            serial_comm_1 = new SerialPort();
            serial_comm_1.PortName = "COM5";
            serial_comm_1.BaudRate = 115200;
            serial_comm_1.Open();

            Console.WriteLine("Hello World!");
            Console.ReadKey();

            byte[] XP_receive_data = new byte[1024];
            byte[] XP_send_data = { 68, 65, 84, 65, 0, 11, 0, 0, 0, 205, 204, 76, 62, 205, 204, 76, 62, 205, 204, 76, 62, 0, 192, 121, 196, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 205, 204, 76, 62, 0, 192, 121, 196, 0, 192, 121, 196, 0, 192, 121, 196, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            int counter = 0;

            int receiving = 0;
            int bit_1 = 0;
            int bit_2 = 0;
            int bit_3 = 0;
            int i = 0;
            int all_received = 0;
            float[] raw_bytes_from_sim = { 0, 0, 0, 0, 0 };

            float elev = 0.0f, ail = 0.0f, rudder = 0.0f, throttle = 0.0f;

            float pitch = 0.0f;
            float p_pitch = 0.0f;

            float roll = 0.0f;
            float p_roll = 0.0f;

            float alt = 0.0f;
            float p_alt = 0.0f;

            float ias = 0.0f;
            float p_ias = 0.0f;


            float g_x = 0.0f;
            float g_y = 0.0f;
            float g_z = 0.0f;

            float dt = 0.05f;

            float P = 0.0f;
            float Q = 0.0f;
            float acc_x = 0.0f;
            float acc_y = 0.0f;
            float acc_z = 0.0f;


            Thread thread1 = new Thread(() =>
            {
                for (; ; )
                {
                    XP_receive_data = XPserver_receive.Receive(ref XPip);

                    pitch = BitConverter.ToSingle(XP_receive_data, 45);
                    roll = BitConverter.ToSingle(XP_receive_data, 49);
                    alt = BitConverter.ToSingle(XP_receive_data, 89);
                    ias = BitConverter.ToSingle(XP_receive_data, 33);

                    g_z = (float)(Math.Cos(pitch / 57.324) * Math.Cos(roll / 57.324) * 9.8);
                    g_x = (float)(Math.Sin(pitch / 57.324) * 9.8);
                    g_y = (float)(Math.Sin(roll / 57.3) * g_z);

                    P = (float)((pitch - p_pitch) / (dt));
                    Q = (float)((roll - p_roll) / (dt));

                    acc_x = (float)((ias - p_ias) / (dt)) + g_x;
                    acc_y = g_y;
                    acc_z = g_z;

                    p_pitch = pitch;
                    p_roll = roll;
                    p_ias = ias;

                    Random rnd = new Random();

                    float noise_gyro = (float)((rnd.Next(1, 10)) * 0.00001);
                    float noise_acc = (float)((rnd.Next(1, 10)) * 0.001);

                    P += noise_gyro;
                    Q += noise_gyro;
                    acc_x += noise_acc;
                    acc_y += noise_acc;
                    acc_z += noise_acc;

                    byte[] acc_x_bytes_array = new byte[4];
                    byte[] acc_y_bytes_array = new byte[4];
                    byte[] acc_z_bytes_array = new byte[4];

                    byte[] P_bytes_array = new byte[4];
                    byte[] Q_bytes_array = new byte[4];

                    acc_x_bytes_array = BitConverter.GetBytes(acc_x);
                    acc_y_bytes_array = BitConverter.GetBytes(acc_y);
                    acc_z_bytes_array = BitConverter.GetBytes(acc_z);

                    P_bytes_array = BitConverter.GetBytes(P);
                    Q_bytes_array = BitConverter.GetBytes(Q);

                    //Console.WriteLine(acc_z_bytes_array[3] + "," + acc_z_bytes_array[2] + "," + acc_z_bytes_array[1] + "," + acc_z_bytes_array[0] + ","+ acc_x + "," + acc_y + "," +acc_z);

                    byte[] data_to_serial = {   255,254,253,252,
                                                XP_receive_data[45],
                                                XP_receive_data[46],
                                                XP_receive_data[47],
                                                XP_receive_data[48],
                                                XP_receive_data[49],
                                                XP_receive_data[50],
                                                XP_receive_data[51],
                                                XP_receive_data[52],
                                                XP_receive_data[53],
                                                XP_receive_data[54],
                                                XP_receive_data[55],
                                                XP_receive_data[56],
                                                XP_receive_data[33],
                                                XP_receive_data[34],
                                                XP_receive_data[35],
                                                XP_receive_data[36],
                                                XP_receive_data[89],
                                                XP_receive_data[90],
                                                XP_receive_data[91],
                                                XP_receive_data[92],
                                                XP_receive_data[93],
                                                XP_receive_data[94],
                                                XP_receive_data[95],
                                                XP_receive_data[96],
                                                acc_x_bytes_array[0],
                                                acc_x_bytes_array[1],
                                                acc_x_bytes_array[2],
                                                acc_x_bytes_array[3],
                                                acc_y_bytes_array[0],
                                                acc_y_bytes_array[1],
                                                acc_y_bytes_array[2],
                                                acc_y_bytes_array[3],
                                                acc_z_bytes_array[0],
                                                acc_z_bytes_array[1],
                                                acc_z_bytes_array[2],
                                                acc_z_bytes_array[3],
                                                P_bytes_array[0],
                                                P_bytes_array[1],
                                                P_bytes_array[2],
                                                P_bytes_array[3],
                                                Q_bytes_array[0],
                                                Q_bytes_array[1],
                                                Q_bytes_array[2],
                                                Q_bytes_array[3],
                                                XP_receive_data[81],
                                                XP_receive_data[82],
                                                XP_receive_data[83],
                                                XP_receive_data[84],
                                                XP_receive_data[85],
                                                XP_receive_data[86],
                                                XP_receive_data[87],
                                                XP_receive_data[88],
                    };

                    counter += 1;
                    if (counter == 2) {
                        serial_comm_1.Write(data_to_serial, 0, 56);
                        counter = 0;
                    }

                                   
                                                          

                    for (int index = 0; index < data_to_serial.Length; index++)
                    {
                        //Console.WriteLine( data_to_serial[index]);
                    }

                    Thread.Sleep(10);
                }
            }); // Thread 1 ends here!


            Thread thread2 = new Thread(() =>
            {
                for (; ; )
                {
                    byte[] received = new byte[1];
                    
                    
                    serial_comm_1.Read(received, 0, 1);

                    if (receiving == 1)
                    {
                        i += 1;

                    }
                    if (received[0] == 255 && receiving == 0)
                    {
                        bit_1 = 1;
                    }
                    else if (received[0] == 254 && receiving == 0 && bit_1 == 1)
                    {
                        bit_2 = 1;
                    }
                    else if (received[0] == 253 && receiving == 0 && bit_2 == 1)
                    {
                        bit_3 = 1;
                    }
                    else if (received[0] == 252 && receiving == 0 && bit_3 == 1)
                    {
                        receiving = 1;
                        bit_1 = 0;
                        bit_2 = 0;
                        bit_3 = 0;
                    }
                    else
                    {
                        bit_1 = 0;
                        bit_2 = 0;
                        bit_3 = 0;
                    }

                    raw_bytes_from_sim[i] = received[0];
                    

                    if (i == 4)
                    {
                        receiving = 0;
                        i = 0;
                        bit_1 = 0;
                        bit_2 = 0;
                        bit_3 = 0;
                        all_received = 1;
                    }


                    if (all_received == 1)
                    {
                        elev = raw_bytes_from_sim[1];
                        ail = ((raw_bytes_from_sim[2]));
                        rudder = ((raw_bytes_from_sim[3]));
                        throttle =((raw_bytes_from_sim[4]));
                        
                        Console.WriteLine(elev + "," + ail + "," + rudder + "," + throttle);

                        elev = (elev - 100) / 500;
                        ail = (ail - 100) / 1000;
                        rudder = (rudder - 100) / 1000;
                        throttle = (throttle - 100) / 100;

                        


                        byte[] bytes_array = new byte[4];

                        bytes_array = BitConverter.GetBytes(elev);
                        XP_send_data[9] = bytes_array[0];
                        XP_send_data[10] = bytes_array[1];
                        XP_send_data[11] = bytes_array[2];
                        XP_send_data[12] = bytes_array[3];

                        bytes_array = BitConverter.GetBytes(ail);
                        XP_send_data[13] = bytes_array[0];
                        XP_send_data[14] = bytes_array[1];
                        XP_send_data[15] = bytes_array[2];
                        XP_send_data[16] = bytes_array[3];

                        bytes_array = BitConverter.GetBytes(rudder);
                        XP_send_data[17] = bytes_array[0];
                        XP_send_data[18] = bytes_array[1];
                        XP_send_data[19] = bytes_array[2];
                        XP_send_data[20] = bytes_array[3];

                        bytes_array = BitConverter.GetBytes(throttle);
                        XP_send_data[45] = bytes_array[0];
                        XP_send_data[46] = bytes_array[1];
                        XP_send_data[47] = bytes_array[2];
                        XP_send_data[48] = bytes_array[3];
                    }

                    all_received = 0;





                    XPserver_send.Send(XP_send_data, XP_send_data.Length);
                }
            }); // Thread 2 ends here!

            thread1.Start();
            thread2.Start();

        }
    }
}
