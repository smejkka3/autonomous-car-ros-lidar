#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sixwd_msgs/SixWheelCommand.h"
#include "sixwd_msgs/SixWheelInfo.h"
// C library headers
#include <stdio.h>
#include <string.h>
// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
//TODO: Backward direction error

class Translator
{
public:
  Translator()
  {
    ROS_INFO("Initializing");
    n = ros::NodeHandle("~");
    motor_cmd_sub = n.subscribe("motor_commands", 1000, &Translator::motorcmdCallback, this);
    motor_info_pub = n.advertise<sixwd_msgs::SixWheelInfo>("motor_controller_info", 1000);
    //motor_info_timer = n.createTimer(ros::Duration(0.1),&Translator::motorinfoCallback,this); //every 100ms for now
    motor_cmd_timer = n.createTimer(ros::Duration(1.0 / 25.0), &Translator::motorcmdCallback, this); //every 100ms for now
    bytes_to_send = new unsigned char[10];
    serial_buffer = new unsigned char[1];

    //////////////////
    //Initilise USB//
    /////////////////
    serial_port = open("/dev/ttyUSB_thumper", O_RDWR);
    // Check for errors
    if (serial_port < 0)
    {
      ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
    }
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    // Read in existing settings, and handle any error
    if (tcgetattr(serial_port, &tty) != 0)
    {
      ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    // tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    // tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    //tty.c_cflag |= CS5; // 5 bits per byte
    //tty.c_cflag |= CS6; // 6 bits per byte
    //tty.c_cflag |= CS7; // 7 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    // tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    // To enable followings just change '&' with '|'
    tty.c_lflag &= ~ICANON;                                                      //(Dis)Enable Canonical(ie: processes when new line recieved)
    tty.c_lflag &= ~ECHO;                                                        // Disable echo
    tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
    tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
    tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST;                                                       // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;                                                       // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
    tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B57600); //B0,  B50,  B75,  B110,  B134,  B150,  B200, B300, B600, B1200, B1800, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
    cfsetospeed(&tty, B57600);
    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
      ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    ////////////////////////////
    //End of USB Initilisation//
    ///////////////////////////

    ROS_INFO("Initialised");

    //Create a Dummy Message To trigger Comminication Between MCU and Serial Port //
    bytes_to_send[0] = 1;
    bytes_to_send[1] = 255;
    bytes_to_send[2] = 8;
    bytes_to_send[3] = 51;
    bytes_to_send[4] = 0;
    bytes_to_send[5] = 0;
    bytes_to_send[6] = 0;
    bytes_to_send[7] = 4;
    bytes_to_send[8] = '\0';
  }

  void motorcmdCallback(const sixwd_msgs::SixWheelCommand::ConstPtr &msg)
  {
    bytes_to_send[0] = 1;
    bytes_to_send[1] = 255;
    // ROS_INFO("Motor Command Call Back");
    switch (msg->controltype)
    {
    case 0:
      bytes_to_send[2] = 7;
      if ((msg->left_speed >= 0) && (msg->right_speed >= 0))
      {
        bytes_to_send[3] = 52;
      }
      if ((msg->left_speed >= 0) && (msg->right_speed < 0))
      {
        bytes_to_send[3] = 53;
      }
      if ((msg->left_speed < 0) && (msg->right_speed >= 0))
      {
        bytes_to_send[3] = 54;
      }
      if ((msg->left_speed < 0) && (msg->right_speed < 0))
      {
        bytes_to_send[3] = 55;
      }
      bytes_to_send[4] = abs(msg->right_speed);
      bytes_to_send[5] = abs(msg->left_speed);
      bytes_to_send[6] = 4;
      bytes_to_send[7] = '\0';
      break;
    case 1:
      bytes_to_send[2] = 8;
      bytes_to_send[3] = 51;
      if ((msg->linearspeed) > 0)
      {
        if ((msg->angle) > 0)
        {
          bytes_to_send[4] = 2;
        }
        else
        {
          bytes_to_send[4] = 1;
        }
      }
      else if ((msg->linearspeed) == 0)
      {
        bytes_to_send[4] = 0;
      }
      else
      {
        if ((msg->angle) > 0)
        {
          bytes_to_send[4] = 3;
        }
        else
        {
          bytes_to_send[4] = 4;
        }
      }

      bytes_to_send[5] = abs(msg->linearspeed);
      bytes_to_send[6] = abs(msg->angle);
      bytes_to_send[7] = 4; //Stop Byte
      bytes_to_send[8] = '\0';
      break;
    case 2:
      bytes_to_send[2] = 7;
      bytes_to_send[3] = (msg->motor_number + 9);
      bytes_to_send[4] = abs(msg->individual_motors_speed);

      if ((msg->individual_motors_speed) > 0)
      {
        bytes_to_send[5] = 2;
      }
      else if ((msg->individual_motors_speed) < 0)
      {
        bytes_to_send[5] = 1;
      }
      else
      {
        bytes_to_send[5] = 0;
      }

      bytes_to_send[6] = 4;
      bytes_to_send[7] = '\0';
      break;
    }
  }

  void motorcmdCallback(const ros::TimerEvent &)
  {

    u_int8_t State = 0;                  // This variable determines the message checking stages
    u_int8_t Bytes_Number = 0;           // This varible contains how many bytes recieved
    u_int8_t BytesToReceive = 0;         // This varible contains the expected bytes to recive
    u_int8_t InformationSize = 0;        // This variable contains the information size
    u_int8_t RecievedControlByte = 0;    // This vabiable contains the ControlByte
    u_int8_t InformationByteCounter = 0; //This variable keep the current number of information byte

    // ROS_INFO("Reading From MCU");

    write(serial_port, (char *)bytes_to_send, sizeof(char) * bytes_to_send[2]); // Writes the message and Triggers MCU response

    int byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read One Byte
    Bytes_Number = Bytes_Number + byte_checker;                            // Received = 0 + 1 = 1

    if (byte_checker < 0)
    {
      ROS_ERROR("Error reading: %s", strerror(errno));
    } //Check if Serial port woeks fine

    if (State == 0) //STATE 0: Start byte check
    {
      // ROS_INFO("STATE0");

      if (serial_buffer[0] == 1) //Protocol Start Byte is 1
      {
        State++;                                                           //Pass the Stage 1
        byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); //Read next byte
        Bytes_Number = Bytes_Number + byte_checker;                        // Received = 1 + 1 = 2
      }
      else
      {
        ROS_ERROR("Start Byte is not true!");
        State = 0;
      }
    }

    if (State == 1) // State 1 : Check Adress
    {
      //ROS_INFO("STATE1");

      if (serial_buffer[0] == 255) //My Adress is 255
      {
        State++;                                                           // Go to the next state
        byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); //Read Next One
        Bytes_Number = Bytes_Number + byte_checker;                        // Received = 2 + 1 = 3
      }
      else
      {
        ROS_ERROR("Adress is not ture!!");
        State = 0;
      }
    }
    if (State == 2) // State 2: Check message size and calculate Information size
    {
      //ROS_INFO("STATE2");
      // The RecievedByte in this state contains the size of the message
      BytesToReceive = serial_buffer[0];
      // Calcule the size of information bytes : That is BytesToReceive minus startbyte, adressbyte, lenghtbyte, controlbyte and stopbyte = 5
      InformationSize = (BytesToReceive - 5);
      State++;                                                           // Go to the next state
      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); //Read Next One
      Bytes_Number = Bytes_Number + byte_checker;                        // Received = 3 + 1 = 4
    }
    if (State == 3) // State 3: Check control byte
    {
      //ROS_INFO("STATE3");
      // Put the ReceivedByte in RecievedControlByte
      RecievedControlByte = serial_buffer[0];
      State++; // Pass The Stage
    }

    // State 4:
    if (State == 4)
    {
      //ROS_INFO("STATE4");

      // Assign Information to Related Message

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.linearspeed = serial_buffer[0];                       //Linear Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 0 + 1 = 1

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor1_speed = serial_buffer[0];                      // First Motor's Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 1 + 1 = 2

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor2_speed = serial_buffer[0];                      // Second Motor's Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 2 + 1 = 3

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor3_speed = serial_buffer[0];                      // Third Motor's Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 3 + 1 = 4

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor4_speed = serial_buffer[0];                      // Fourth Motor's Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 4 + 1 = 5

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor5_speed = serial_buffer[0];                      // Fifth Motor's Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 5 + 1 = 6

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor6_speed = serial_buffer[0];                      // Sixth Motor's Speed
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 6 + 1 = 7

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor1_current = serial_buffer[0];                    // First Motor's Current
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 7 + 1 = 8

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor2_current = serial_buffer[0];                    // Second Motor's Current
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 8 + 1 = 9

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor3_current = serial_buffer[0];                    // Third Motor's Current
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 9 + 1 = 10

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor4_current = serial_buffer[0];                    // Fourth Motor's Current
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 10 + 1 = 11

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor5_current = serial_buffer[0];                    // Fifth Motor's Current
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 11 + 1 = 12

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.motor6_current = serial_buffer[0];                    // Sixth Motor's Current
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 12 + 1 = 13

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.voltage = serial_buffer[0];                           // Voltage
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 13 + 1 = 14

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read Another one
      info_message.temperature = serial_buffer[0];                        // Temperature
      InformationByteCounter = InformationByteCounter + byte_checker;    // Info = 14 + 1 = 15

      if (bytes_to_send[5] < 20)
      {
        info_message.motor1_speed = 0;
        info_message.motor2_speed = 0;
        info_message.motor3_speed = 0;
      }

      if(bytes_to_send[4] < 20)
      {
        info_message.motor4_speed = 0;
        info_message.motor5_speed = 0;
        info_message.motor6_speed = 0;
      }

      if (bytes_to_send[3] == 54 || bytes_to_send[3] == 55)
      {
        info_message.motor1_speed = -info_message.motor1_speed;
        info_message.motor2_speed = -info_message.motor2_speed;
        info_message.motor3_speed = -info_message.motor3_speed;
      }
      if (bytes_to_send[3] == 53 || bytes_to_send[3] == 55)
      {
        info_message.motor4_speed = -info_message.motor4_speed;
        info_message.motor5_speed = -info_message.motor5_speed;
        info_message.motor6_speed = -info_message.motor6_speed;
      }

      // if ((bytes_to_send[3] == 51) && ((bytes_to_send[4] == 1) || (bytes_to_send[4] == 2))) //We are Going Forward Speeds are True
      // {
      //   if ((bytes_to_send[4] == 1)) //Turning Left Command issued Motor 1,2,3 Getting Slower
      //   {
      //     if (bytes_to_send[6] > 126) //After 126 They turns to oposit direction
      //     {
      //       info_message.motor1_speed = -info_message.motor1_speed;
      //       info_message.motor2_speed = -info_message.motor2_speed;
      //       info_message.motor3_speed = -info_message.motor3_speed;
      //     }
      //     if (abs((bytes_to_send[6] / 126) - bytes_to_send[5]) <= 20) // if Setpoint is under 20 Wheels stops
      //     {
      //       info_message.motor1_speed = 0.0;
      //       info_message.motor2_speed = 0.0;
      //       info_message.motor3_speed = 0.0;
      //     }
      //   }
      //   else if ((bytes_to_send[4] == 2)) //Turning Right Command issued Motor 4,5,6 Getting Slower
      //   {
      //     if (bytes_to_send[6] > 126)
      //     {
      //       info_message.motor4_speed = -info_message.motor4_speed;
      //       info_message.motor5_speed = -info_message.motor5_speed;
      //       info_message.motor6_speed = -info_message.motor6_speed;
      //     }
      //     if (abs((bytes_to_send[6] / 126) - bytes_to_send[5]) < 20) // if Setpoint is under 20 Wheels stops
      //     {
      //       info_message.motor4_speed = 0.0;
      //       info_message.motor5_speed = 0.0;
      //       info_message.motor6_speed = 0.0;
      //     }
      //   }
      // }
      // else if ((bytes_to_send[3] == 51) && ((bytes_to_send[4] == 3) || (bytes_to_send[4] == 4))) //We are Going Backwards Speeds are inversed
      // {
      //   info_message.motor1_speed = -info_message.motor1_speed;
      //   info_message.motor2_speed = -info_message.motor2_speed;
      //   info_message.motor3_speed = -info_message.motor3_speed;
      //   info_message.motor4_speed = -info_message.motor4_speed;
      //   info_message.motor5_speed = -info_message.motor5_speed;
      //   info_message.motor6_speed = -info_message.motor6_speed;

      //   if (bytes_to_send[4] == 3) //Going Back ward and right side is getting slower w.r.t to car
      //   {
      //     if (bytes_to_send[6] > 126) //After 126 They turns to oposit direction
      //     {
      //       info_message.motor4_speed = -info_message.motor4_speed;
      //       info_message.motor5_speed = -info_message.motor5_speed;
      //       info_message.motor6_speed = -info_message.motor6_speed;
      //     }
      //     if (abs((bytes_to_send[6] / 126) - bytes_to_send[5]) < 20) // if Setpoint is under 20 Wheels stops
      //     {
      //       info_message.motor4_speed = 0.0;
      //       info_message.motor5_speed = 0.0;
      //       info_message.motor6_speed = 0.0;
      //     }
      //   }
      //   else if ((bytes_to_send[4] == 4)) //Turning Right Command issued Motor 4,5,6 Getting Slower
      //   {
      //     if (bytes_to_send[6] >= 126)
      //     {
      //       info_message.motor1_speed = -info_message.motor1_speed;
      //       info_message.motor2_speed = -info_message.motor2_speed;
      //       info_message.motor3_speed = -info_message.motor3_speed;
      //     }
      //     if (abs((bytes_to_send[6] / 126) - bytes_to_send[5]) < 20) // if Setpoint is under 20 Wheels stops
      //     {
      //       info_message.motor1_speed = 0.0;
      //       info_message.motor2_speed = 0.0;
      //       info_message.motor3_speed = 0.0;
      //     }
      //   }
      // }

      info_message.linearspeed = (info_message.motor2_speed + info_message.motor5_speed) / 2;

      Bytes_Number = Bytes_Number + InformationByteCounter; // 4 + 15 = 19

      byte_checker = read(serial_port, serial_buffer, sizeof(char) * 1); // Read the End Byte
      Bytes_Number = Bytes_Number + byte_checker;                        //19 + 1 = 20

      // Check Message Integrity, End Byte and Publish It
      if ((InformationByteCounter == InformationSize) && (serial_buffer[0] == 4) && (Bytes_Number == 20))
      {
        //ROS_INFO("Message is in integrity!!!");
        State++;
        motor_info_pub.publish(info_message);
      }
      else
      {
        ROS_ERROR("Message is NOT in integrity");
      }
    }
  }

private:
  ros::NodeHandle n;
  ros::Subscriber motor_cmd_sub;
  ros::Publisher motor_info_pub;
  ros::Timer motor_cmd_timer;
  int serial_port;
  u_int8_t *bytes_to_send;
  u_int8_t *serial_buffer;
  sixwd_msgs::SixWheelInfo info_message;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "serial_communicator");
  Translator serial;
  ros::spin();
  return 0;
}
