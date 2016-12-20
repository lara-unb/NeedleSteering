/*
 * main2.cpp
 *
 *  Created on: Apr 30, 2015
 *      Author: andre
 */

#include "UStepDevice.h"
#include "debug.h"
#include <pigpio.h>
#include <math.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <cstring>      // Needed for memset
#include <sys/socket.h> // Needed for the socket functions
#include <netdb.h>      // Needed for the socket functions

// Commands exchanged with the Matlab client
#define CMD_OPEN_FRONT_GRIPPER		  1
#define CMD_CLOSE_FRONT_GRIPPER		  2
#define CMD_OPEN_BACK_GRIPPER		    3
#define CMD_CLOSE_BACK_GRIPPER		  4
#define CMD_ROTATE                  5
#define CMD_TRANSLATE               6
#define CMD_MOVE_FORWARD            7
#define CMD_MOVE_BACKWARD           8
#define CMD_MOVE_DC_BIDIRECTIONAL   9
#define CMD_MOVE_DC_FLIPPING        10

#define CMD_MOVE_DC_FLIPPING_PART1  11
#define CMD_MOVE_DC_FLIPPING_PART2  12

#define CMD_DONE                    42
#define CMD_SHUT_DOWN               255

#define CMD_MOVE_MOTOR              101           // DISABLED
#define CMD_MOVE_MOTOR_STEPS        102           // DISABLED
#define CMD_SET_DIRECTION           103           // DISABLED
#define CMD_SET_ENABLE              104           // DISABLED

// Global parameters
const char kTCPPortNumber[] = "5555";
const int kInputBufferSize = 1000;
const char ack_byte = CMD_DONE;

// Global Variables
UStepDevice device;                       // Object to represent the complete device
int socket_fd ;                           // Socket for opening the TCP/IP communication
int connection_socket_fd;                 // Socket for connecting to the Matlab client
char input_data_buffer[kInputBufferSize]; // Input data buffer

int startTCPServer(int *socket_fd, const char *port_number)
{
  int status;
  struct addrinfo host_info;
  struct addrinfo *host_info_list;

  memset(&host_info, 0, sizeof host_info);

  // Setting up the structs
  host_info.ai_family = AF_UNSPEC;
  host_info.ai_socktype = SOCK_STREAM;
  host_info.ai_flags = AI_PASSIVE;
  status = getaddrinfo(NULL, port_number, &host_info, &host_info_list);
  if (status != 0)
  {
    Error("ERROR Main::startTCPServer - Error setting up the addrinfo struct \n");
    return ERR_ADDR_INFO_CREATE_FAIL;
  }

  // Creating socket
  *socket_fd = socket(host_info_list->ai_family, host_info_list->ai_socktype, host_info_list->ai_protocol);
  if (*socket_fd == -1)
  {
    Error("ERROR Main::startTCPServer - Error creating the socket \n");
    return ERR_SOCKET_CREATE_FAIL;
  }

  // Binding socket
  int yes = 1;
  status = setsockopt(*socket_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
  status = bind(*socket_fd, host_info_list->ai_addr, host_info_list->ai_addrlen);
  if (status == -1)
  {
    Error("ERROR Main::startTCPServer - Error binding the socket \n");
    return ERR_SOCKET_BIND_FAIL;
  }

  freeaddrinfo(host_info_list);

  return 0;
}

int acceptTCPConnection(int socket_fd, int *connection_socket_fd)
{
  // Listening for connections on the opened socket
  if (listen(socket_fd, 5) < 0)
  {
    Error("ERROR Main::acceptTCPConnection - Listen error \n");
    return ERR_SOCKET_LISTEN_FAIL;
  }

  // Accepting connection from the client
  struct sockaddr_storage their_addr;
  socklen_t addr_size = sizeof(their_addr);
  *connection_socket_fd = accept(socket_fd, (struct sockaddr *)&their_addr, &addr_size);
  if (*connection_socket_fd == -1)
  {
    Error("ERROR Main::acceptTCPConnection - Accept error \n");
    return ERR_SOCKET_ACCEPT_FAIL;
  }

  return 0;
}

void displayReceivedData(char *data_buffer, int bytes_received)
{
  printf("%d bytes received: ", bytes_received);
  for(int i = 0; i < bytes_received-1; i++)
  {
    printf("%u, ", data_buffer[i]);
  }
  printf("%u\n", data_buffer[bytes_received-1]);
}

void sendAckByte()
{
  ssize_t bytes_sent = -1;
  while(bytes_sent == -1)
  {
    bytes_sent = send(connection_socket_fd, &ack_byte, 1, 0);
    usleep(300000);
  }

}

int decodeReceivedMessage(ssize_t bytes_received)
{
  Debug("Received command %u with %u bytes\n", input_data_buffer[0], bytes_received);
  Debug("Received bytes: ");
  for(int i = 0; i < bytes_received; i++)
    Debug("%X, ", input_data_buffer[i]);
  Debug("\n");

  switch(input_data_buffer[0])
  {

    case CMD_OPEN_FRONT_GRIPPER:
      if(bytes_received == 1)
      {
        device.openFrontGripper();
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command OPEN_FRONT_GRIPPER \n");
      }

      break;

    case CMD_CLOSE_FRONT_GRIPPER:
      if(bytes_received == 1)
      {
        device.closeFrontGripper();
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command CLOSE_FRONT_GRIPPER \n");
      }

      break;

    case CMD_OPEN_BACK_GRIPPER:
      if(bytes_received == 1)
      {
        device.openBackGripper();
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command OPEN_BACK_GRIPPER \n");
      }

      break;

    case CMD_CLOSE_BACK_GRIPPER:
      if(bytes_received == 1)
      {
        device.closeBackGripper();
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command CLOSE_BACK_GRIPPER \n");
      }

      break;

    case CMD_ROTATE:
      if(bytes_received == 17)
      {
        double revolutions;
        double rotation_speed;
        memcpy(&revolutions   , input_data_buffer + 1 , 8);
        memcpy(&rotation_speed, input_data_buffer + 9 , 8);
        Debug("Received command ROTATE with parameters: revolutions = %f, rotation speed = %f\n", revolutions, rotation_speed);

        Debug("Performing a duty cycle motion... \n");
        device.rotateNeedle(revolutions, rotation_speed);
        sendAckByte();
        Debug("Done, waiting for next command \n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_SPIN \n");
      }
      break;

    case CMD_TRANSLATE:
      if(bytes_received == 17)
      {
        double displacement;
        double speed;
        memcpy(&displacement, input_data_buffer + 1 , 8);
        memcpy(&speed       , input_data_buffer + 9, 8);
        Debug("Received command TRANSLATE with parameters: displacement = %f, speed = %f\n", displacement, speed);

        Debug("Moving the motor... \n");
        device.translateFrontGripper(displacement, speed);
        sendAckByte();
        Debug("Done, waiting for next command \n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_MOTOR \n");
      }
      break;

    case CMD_MOVE_FORWARD:
      if(bytes_received == 17)
      {
        double insertion_depth;
        double insertion_speed;
        memcpy(&insertion_depth, input_data_buffer + 1 , 8);
        memcpy(&insertion_speed, input_data_buffer + 9 , 8);
        Debug("Received command MOVE_FORWARD with parameters: displacement = %f, insertion speed = %f\n", insertion_depth, insertion_speed);

        Debug("Performing a duty cycle motion... \n");
        device.performBidirectionalDutyCyleStep(insertion_depth, insertion_speed, 1.0, 0.0);
        sendAckByte();
        Debug("Done, waiting for next command \n\n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_DC \n");
      }
      break;

    case CMD_MOVE_BACKWARD:
      if(bytes_received == 17)
      {
        double insertion_depth;
        double insertion_speed;
        memcpy(&insertion_depth, input_data_buffer + 1 , 8);
        memcpy(&insertion_speed, input_data_buffer + 9 , 8);
        Debug("Received command MOVE_BACKWARD with parameters: displacement = %f, insertion speed = %f\n", insertion_depth, insertion_speed);

        Debug("Moving the needle backward... \n");
        device.performBackwardStep(insertion_depth, insertion_speed);
        sendAckByte();
        Debug("Done, waiting for next command \n\n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_BACK \n");
      }
      break;

    case CMD_MOVE_DC_BIDIRECTIONAL:
      if(bytes_received == 33)
      {
        double insertion_depth;
        double insertion_speed;
        double rotation_speed;
        double duty_cycle;
        memcpy(&insertion_depth, input_data_buffer + 1 , 8);
        memcpy(&insertion_speed, input_data_buffer + 9 , 8);
        memcpy(&rotation_speed , input_data_buffer + 17, 8);
        memcpy(&duty_cycle     , input_data_buffer + 25, 8);
        Debug("Received command MOVE_DC_BIDIRECTIONAL with parameters: displacement = %f, insertion speed = %f, rotation speed = %f, DC = %f\n", insertion_depth, insertion_speed, rotation_speed, duty_cycle);

        Debug("Performing a duty cycle motion... \n");
        device.performBidirectionalDutyCyleStep(insertion_depth, insertion_speed, rotation_speed, duty_cycle);
        sendAckByte();
        Debug("Done, waiting for next command \n\n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_DC_BIDIRECTIONAL \n");
      }
      break;

    case CMD_MOVE_DC_FLIPPING:
      if(bytes_received == 33)
      {
        double insertion_depth;
        double insertion_speed;
        double minimum_insertion;
        double duty_cycle;
        memcpy(&insertion_depth  , input_data_buffer + 1 , 8);
        memcpy(&insertion_speed  , input_data_buffer + 9 , 8);
        memcpy(&minimum_insertion, input_data_buffer + 17, 8);
        memcpy(&duty_cycle       , input_data_buffer + 25, 8);
        Debug("Received command MOVE_DC_FLIPPING with parameters: displacement = %f, insertion speed = %f, minimum insertion = %f, DC = %f\n", insertion_depth, insertion_speed, minimum_insertion, duty_cycle);

        Debug("Performing a duty cycle motion... \n");
        device.performFlippingDutyCyleStep(insertion_depth, insertion_speed, minimum_insertion, duty_cycle);
        sendAckByte();
        Debug("Done, waiting for next command \n\n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_DC_FLIPPING \n");
      }
      break;

    case CMD_MOVE_DC_FLIPPING_PART1:
      if(bytes_received == 33)
      {
        double insertion_depth;
        double insertion_speed;
        double minimum_insertion;
        double duty_cycle;
        memcpy(&insertion_depth  , input_data_buffer + 1 , 8);
        memcpy(&insertion_speed  , input_data_buffer + 9 , 8);
        memcpy(&minimum_insertion, input_data_buffer + 17, 8);
        memcpy(&duty_cycle       , input_data_buffer + 25, 8);
        Debug("Received command MOVE_DC_FLIPPING_PART1 with parameters: displacement = %f, insertion speed = %f, minimum insertion = %f, DC = %f\n", insertion_depth, insertion_speed, minimum_insertion, duty_cycle);

        Debug("Performing a duty cycle motion... \n");
        device.performFlippingDutyCyleStepPart1(insertion_depth, insertion_speed, minimum_insertion, duty_cycle);
        sendAckByte();
        Debug("Done, waiting for next command \n\n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_DC_FLIPPING_PART1 \n");
      }
      break;

    case CMD_MOVE_DC_FLIPPING_PART2:
      if(bytes_received == 33)
      {
        double insertion_depth;
        double insertion_speed;
        double minimum_insertion;
        double duty_cycle;
        memcpy(&insertion_depth  , input_data_buffer + 1 , 8);
        memcpy(&insertion_speed  , input_data_buffer + 9 , 8);
        memcpy(&minimum_insertion, input_data_buffer + 17, 8);
        memcpy(&duty_cycle       , input_data_buffer + 25, 8);
        Debug("Received command MOVE_DC_FLIPPING_PART2 with parameters: displacement = %f, insertion speed = %f, minimum insertion = %f, DC = %f\n", insertion_depth, insertion_speed, minimum_insertion, duty_cycle);

        Debug("Performing a duty cycle motion... \n");
        device.performFlippingDutyCyleStepPart2(insertion_depth, insertion_speed, minimum_insertion, duty_cycle);
        sendAckByte();
        Debug("Done, waiting for next command \n\n");
      }
      else
      {
        Warn("WARNING Main::decodeReceivedMessage - Bad parameters for command MOVE_DC_FLIPPING_PART2 \n");
      }
      break;

    // Shut down the UStep Device control software
    case CMD_SHUT_DOWN:
      return -1;

    //
    default:
      Warn("WARNING Main::decodeReceivedMessage - Unrecognized command \n");
      displayReceivedData(input_data_buffer, bytes_received);
  }
  return 0;
}

int communicateWithTheMatlabClient()
{
  printf("Starting TCP/IP communication with the Matlab client \n");

  // Initialize the TCP/IP Server to communicate with the Matlab cient
  if(startTCPServer(&socket_fd, kTCPPortNumber) < 0)
  {
    Error("ERROR Main - Unable to start the TCP/IP server \n");
    return ERR_START_TCPIP_FAIL;
  }

  bool stop_TCP_server = false;
  while(!stop_TCP_server)
  {
    // Wait for the client to connect
    if(acceptTCPConnection(socket_fd, &connection_socket_fd) < 0)
    {
      Error("ERROR Main - Unable to connect to the client \n");
      close(socket_fd);
      return ERR_CLIENT_CONNECT_FAIL;
    }

    while(!stop_TCP_server)
    {
      // Wait for a message from the Matlab client
      ssize_t bytes_received = recv(connection_socket_fd, input_data_buffer,kInputBufferSize, 0);
      if (bytes_received == -1)
      {
        Error("ERROR Main - Failed to receive message from the client \n");
        close(connection_socket_fd);
        close(socket_fd);
        return ERR_CLIENT_MSG_RECEIVE_FAIL;
      }

      // Host disconnected - wait for new client connection
      if (bytes_received == 0) break;

      // Decode the message and decide what to do
      if(decodeReceivedMessage(bytes_received) == -1)
      {
        stop_TCP_server = true;
        break;
      }
    }

    // Close the communication with the Matlab client
    close(connection_socket_fd);
  }

  // Close the TCP/IP communication
  close(socket_fd);
  return 0;
}

int main(int argc, char *argv[])
{
  // Initialize the UStep Device
  device.configureMotorParameters();
  if(device.initGPIO() < 0)
  {
    Error("ERROR Main::main - Unable to call gpioInitialise() \n");
    return ERR_GPIO_INIT_FAIL;
  }

  device.calibrateMotorsStartingPosition();

  // Connect to the Matlab client and answer to its commands
  communicateWithTheMatlabClient();

  // Stop the UStep Device
  device.terminateGPIO();
  printf("\nShutting down the system! \n");

  return 0 ;
}
