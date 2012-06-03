//
//  serialstream.h
//  
//
//  Created by Nathaniel Lewis on 3/11/12.
//  Copyright (c) UC Merced Robotics Society. All rights reserved.
//

#ifndef _serialstream_h_
#define _serialstream_h_

#include <string>
#include <termios.h>

// A base class to handle serial device exceptions
class SerialDeviceException {
public:
    SerialDeviceException(std::string message);
    std::string message;
};

class SerialDevice {
public:
    // Constructor/Deconstructor
    SerialDevice(std::string port, unsigned int baudrate) throw (SerialDeviceException);
    ~SerialDevice();
    
    // Port setup
    void setBaudrate(unsigned int baudrate);
    
    // Raw port settings control
    void set_termios(struct termios *settings);
    void get_termios(struct termios *settings);
    
    // Port status 
    unsigned int available();  // returns bytes currently in buffer
    void         flush();      // flush the buffers
    
    // IO Operations
    size_t read (char *s, size_t n);
    size_t write(char *s, size_t n);
    
private:
    int  fd;    // The serial port device identifier
    
};

namespace System {
    size_t Read (int fd, void* buf, size_t count);
    size_t Write(int fd, void* buf, size_t count);
    void   Close(int fd);
}

#endif