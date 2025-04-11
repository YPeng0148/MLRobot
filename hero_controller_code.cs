using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;
using Microsoft.SPOT;
using System;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace HERO_Arcade_Drive_Example3
{
    public class Program
    {
        /* create a talon */
        static TalonSRX rightSlave = new TalonSRX(4);
        static TalonSRX right = new TalonSRX(3);
        static TalonSRX leftSlave = new TalonSRX(2);
        static TalonSRX left = new TalonSRX(1);

        static StringBuilder stringBuilder = new StringBuilder();

        static CTRE.Phoenix.Controller.GameController _gamepad = null;

        static SerialPort _uart = null;

        public static void Main()
        {
            byte[] data = new byte[1024];
            string buffer = null;
            
            float v = 0;
            float w = 0;

            bool is_still_reading = false;
            int bytesReceived = 0;

            while (true) 
            {
                if (_gamepad == null)
                    _gamepad = new GameController(UsbHostDevice.GetInstance());

                if (!_gamepad.GetButton(6)) 
                {
                    if (_uart != null) 
                    { 
                        _uart.Flush();
                        _uart.Close();
                        _uart = null;
                    }
                    
                    v = -1 * _gamepad.GetAxis(1);
                    w = _gamepad.GetAxis(2);
                    Drive(v, w);
                } 
                else 
                {   
                    if (_uart == null)
                    {
                        _uart = new SerialPort(CTRE.HERO.IO.Port1.UART, 9600);
                        _uart.Open();
                        is_still_reading = false;
                        buffer = null;
                    }

                    bytesReceived = _uart.Read(data, 0, data.Length);

                    if (bytesReceived > 0)
                    {
                        stringBuilder.Append("received ");
                        stringBuilder.Append(bytesReceived);
                        stringBuilder.Append(" characters");
                        Debug.Print(stringBuilder.ToString());
                        stringBuilder.Clear();

                        for (int i = 0; i < bytesReceived; i++)
                        {
                            stringBuilder.Append("got character: ");
                            stringBuilder.Append(Convert.ToChar(data[i]));
                            Debug.Print(stringBuilder.ToString());
                            stringBuilder.Clear();
                            
                            if (data[i] == '!')
                                is_still_reading = true;
                            
                            else if (data[i] == '@')
                            {
                                try
                                {
                                    v = (float)Convert.ToDouble(buffer);
                                    buffer = null;

                                    stringBuilder.Append("v received: ");
                                    stringBuilder.Append(v);
                                    Debug.Print(stringBuilder.ToString());
                                    stringBuilder.Clear();
                                }
                                catch (System.Exception e)
                                {
                                    stringBuilder.Append("an exception occured");
                                    Debug.Print(stringBuilder.ToString());
                                    stringBuilder.Clear();
                                }
                            }
                            else if (data[i] == '#')
                            {
                                is_still_reading = false;
                                try
                                {
                                    w = (float)Convert.ToDouble(buffer);
                                    buffer = null;

                                    stringBuilder.Append("w received: ");
                                    stringBuilder.Append(w);
                                    Debug.Print(stringBuilder.ToString());
                                    stringBuilder.Clear();

                                    Drive(v, w);
                                }
                                catch (System.Exception e)
                                {
                                    stringBuilder.Append("an exception occured");
                                    Debug.Print(stringBuilder.ToString());
                                    stringBuilder.Clear();
                                }
                            }
                            else if (is_still_reading)
                                buffer += Convert.ToChar(data[i]);
                            
                        }
                    }
                }
                // feed watchdog to keep Talon's enabled 
                CTRE.Phoenix.Watchdog.Feed();
                // run this task every 20ms
                Thread.Sleep(20);
            }
        }
        
        static void Drive(float v, float w)
        {
            float leftThrot = v + w;
            float rightThrot = v - w;

            left.Set(ControlMode.PercentOutput, leftThrot);
            leftSlave.Set(ControlMode.PercentOutput, leftThrot);
            right.Set(ControlMode.PercentOutput, -rightThrot);
            rightSlave.Set(ControlMode.PercentOutput, -rightThrot);
        }
    }
}
