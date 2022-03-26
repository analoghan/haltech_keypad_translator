// Including Arduino_Due_SD_HSCMI library also creates SD object (MassStorage class)
#include <Arduino_Due_SD_HSMCI.h>                       // This creates the object SD
#include "variant.h"
#include <due_can.h>

#include <Arduino.h>
#include <pins_arduino.h>
#include <stdint.h>

#include <M2_12VIO.h>

M2_12VIO M2IO;                                          //Constructor for the M2_12Vio library

// We need to create FileStore object, we will be using it to open/create file and to close file.
FileStore FS;
byte initial_brake_position;

void setup() {

  M2IO.Init_12VIO();                                    // Initialise the M2I/O library;
  M2IO.Setpin_12VIO(1, OFF, SOURCE, PWM_PIN, 1000, 0);  // Set 12V Pin 1 as 12V source, 10000hz, duty cycle 10; Mapped to 
  M2IO.Setpin_12VIO(2, OFF, SOURCE, PWM_PIN, 1000, 0);  // Set 12V Pin 2 as 12V source, 10000hz, duty cycle 10; Mapped to 
  M2IO.Setpin_12VIO(3, OFF, SOURCE, PWM_PIN, 1000, 0);  // Set 12V Pin 3 as 12V source, 10000hz, duty cycle 10; Mapped to 
  M2IO.Setpin_12VIO(4, OFF, SINK);                      // SET 12V Pin 4 as Ground Sink; Mapped to IOA DPI1 - Parking Brake Light
  M2IO.Setpin_12VIO(5, OFF, SINK);                      // SET 12V Pin 5 as Ground Sink; Mapped to IOB DPI3
  M2IO.Setpin_12VIO(6, OFF, SINK);                      // SET 12V Pin 6 as Ground Sink; Mapped to IOA DPI2
  
  pinMode(LED_BUILTIN, OUTPUT);                         // Set onboard LED as output

  // SD Card Setup
  InitalizeSD();
  ReadSD();

  // Setup CANbus
  CANAutoBaud();
}

void InitalizeSD () {
  SD.Init();                                            // Initialization of HSCMI protocol and SD socket switch GPIO (to adjust pin number go to library source file - check Getting Started Guide)
  FS.Init();                                            // Initialization of FileStore class for file manipulation
}

// Read data from SD card
void ReadSD () {
  FS.Open("0:","can_status.txt",false);                 // Open file named "can_status.txt" in directory "0:" in read only mode. 
  SerialUSB.print("Reading from file can_status.txt");  // Print Status
  SerialUSB.print('\n');                                // New line
  FS.Seek(0);                                           // Go to start of file
  char can_initial_status;                              // Define var for can_initial_status
  FS.Read(can_initial_status);                          // Read one byte from file and store it in "can_initial_status"
  initial_brake_position = can_initial_status;
  String initial_brake_status; 
  if ( can_initial_status == 0 ) {
    initial_brake_status = "released";
    M2IO.Setpin_12VIO(4, OFF);    
  } else if ( can_initial_status == 1 ) {
    initial_brake_status = "set";
    M2IO.Setpin_12VIO(4, ON);    
  } else {
    initial_brake_status = "errored";
  }
  SerialUSB.print("Initial brake status is ");          // Format output
  SerialUSB.print(initial_brake_status);                // Print can_initial_status to serial
  SerialUSB.print('\n');                                // New line
  FS.Close();                                           // File is saved, when it is closed. When closed, you can rename it or delete it.  
}

// Write data to SD card
void WriteSD(byte can_status) {
  String brake_status; 
  if ( can_status == 0 ) {
    brake_status = "released";
  } else if ( can_status == 1 ) {
    brake_status = "set";
  } else {
    brake_status = "errored";
  }
  FS.Open("0:","can_status.txt",true);                  // Open file in directory "0:"; third atribute is read (false) / write (true) 
  SerialUSB.print("Writing to file can_status.txt");
  SerialUSB.print('\n');
  FS.Seek(0);                                           // Go to start of file
  SerialUSB.print("Current brake status is ");
  SerialUSB.print(brake_status);
  SerialUSB.print('\n');
  FS.Write(can_status);
  FS.Close();                                           // File is saved, when it is closed. When closed, you can rename it or delete it. 
  initial_brake_position = can_status;
}

// Automatically setup CAN
void CANAutoBaud() {
  SerialUSB.println("Doing Auto Baud scan on CAN0");
  Can0.beginAutoSpeed();
//  SerialUSB.println("Doing Auto Baud scan on CAN1");
//  Can1.beginAutoSpeed();
  
  //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended 
  //or standard frames
  int filter;
  //extended
  for (filter = 0; filter < 3; filter++) {
  Can0.setRXFilter(filter, 0, 0, true);
//  Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (int filter = 3; filter < 7; filter++) {
  Can0.setRXFilter(filter, 0, 0, false);
//  Can1.setRXFilter(filter, 0, 0, false);
  }
}

// Output CAN frame to ECUMaster systems
void CANSendFrame(int call, int state) {
  CAN_FRAME outgoing;
  if ( call == 1 ) {
    outgoing.id = 0x195;
    outgoing.extended = false;
    outgoing.length = 3;
    if ( state == 1 ) {
      outgoing.data.byte[0] = 0x01;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
    } else if( state == 0 ) {
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
    }
    SerialUSB.print("Sending Hazards Frame ");
    SerialUSB.print("\r\n");  
  }
  if ( call == 2 ) {
    outgoing.id = 0x195;
    outgoing.extended = false;
    outgoing.length = 3;
    if ( state == 1 ) {
      outgoing.data.byte[0] = 0x32;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
    } else if( state == 0 ) {
      outgoing.data.byte[0] = 0x00;
      outgoing.data.byte[1] = 0x00;
      outgoing.data.byte[2] = 0x00;
    }
    SerialUSB.print("Sending Horn Frame ");
    SerialUSB.print("\r\n");        
  }
  if ( call == 3 ) {
    outgoing.id = 0x773;
    outgoing.extended = false;
    outgoing.length = 8;
    if ( state == 1 ) {
      outgoing.data.byte[0] = 0x04;
      outgoing.data.byte[1] = 0xFF;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;
    } else if( state == 0 ) {
      outgoing.data.byte[0] = 0xFF;
      outgoing.data.byte[1] = 0x04;
      outgoing.data.byte[2] = 0x00;
      outgoing.data.byte[3] = 0x00;
      outgoing.data.byte[4] = 0x00;
      outgoing.data.byte[5] = 0x00;
      outgoing.data.byte[6] = 0x00;
      outgoing.data.byte[7] = 0x00;   
    }
    SerialUSB.print("Sending Brake Frame ");
    SerialUSB.print("\r\n");
  }
  Can0.sendFrame(outgoing);
}

// CAN Input from Haltech Keypad
void processFrame(CAN_FRAME &frame) {
int call;
int hazard_state;
int horn_state;
byte brake_position;

  if (frame.id == 1548) {
    // Hazard Control
    if ((bitRead(frame.data.bytes[3], 0) == 1) && (bitRead(frame.data.bytes[4], 0) == 1)) {   
        SerialUSB.print("Hazards Enabled\r\n");
        call=1;
        hazard_state=1;
        CANSendFrame(call, hazard_state);           // Send status to CAN output function 
    } else if ((bitRead(frame.data.bytes[3], 0) == 1) && (bitRead(frame.data.bytes[4], 0) == 0)) {   
        SerialUSB.print("Hazards Disabled\r\n");
        call=1;
        hazard_state=0;
        CANSendFrame(call, hazard_state);           // Send status to CAN output function 
    }
    // Horn Control
    if ((bitRead(frame.data.bytes[3], 1) == 1) && (bitRead(frame.data.bytes[4], 0) == 1)) {
        SerialUSB.print("Horn Enabled\r\n");
        call=2;
        horn_state=1;
        CANSendFrame(call, horn_state);              // Send status to CAN output function 
    } else if ((bitRead(frame.data.bytes[3], 1) == 1) && (bitRead(frame.data.bytes[4], 0) == 0)) {
        SerialUSB.print("Horn Disabled\r\n");
        call=2;
        horn_state=0;
        CANSendFrame(call, horn_state);              // Send status to CAN output function 
    }
  }
  if (frame.id == 396) {    
    if (bitRead(frame.data.bytes[1], 1) == 1) {      // Parking Brake Set 
        SerialUSB.print("Set Parking Brake\r\n");
        call=3;
        brake_position = 1;
        M2IO.Setpin_12VIO(4, ON);                     // Turn off brake indicator on dash
        CANSendFrame(call, brake_position);           // Send status to CAN output function           
        if (brake_position != initial_brake_position) {
          SerialUSB.print("Brake status changed to set.\r\n");
          WriteSD(brake_position);
        }
    }
    if (bitRead(frame.data.bytes[1], 6) == 1) {        // Parking Brake Release
        SerialUSB.print("Release Parking Brake\r\n");
        call=3;
        brake_position = 0;
        M2IO.Setpin_12VIO(4, OFF);                    // Turn off brake indicator on dash
        CANSendFrame(call, brake_position);           // Send status to CAN output function        
        if (brake_position != initial_brake_position) {
          SerialUSB.print("Brake status changed to released.\r\n");    
          WriteSD(brake_position);
      }
    }
  }
}

void loop() {
  static unsigned long lastTime = 0;

  CAN_FRAME incoming;
  if (Can0.available() > 0) {
    Can0.read(incoming);
    processFrame(incoming);
  }
}
