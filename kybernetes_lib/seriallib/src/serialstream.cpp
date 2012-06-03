//
//  serialstream.cpp
//  
//
//  Created by Nathaniel Lewis on 3/11/12.
//  Copyright (c) UC Merced Robotics Society. All rights reserved.
//

#include <seriallib/serialstream.h>

#include <sstream> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <time.h>   // time calls

#include <sys/ioctl.h>

SerialDevice::SerialDevice(std::string port, unsigned int baudrate) throw (SerialDeviceException)
{
    // Open the port
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0)
    {
        // The port failed to open
        std::ostringstream error;
        error << "Failure connecting to port \"" << port << "\"" << std::ends;
        throw SerialDeviceException(error.str());
    } else {
        fcntl(fd, F_SETFL, 0);
    }
    
    // Get the current port settings
    struct termios settings;
    get_termios(&settings);
    
    // Set the baudrate
    cfsetispeed(&settings, baudrate);
    cfsetospeed(&settings, baudrate);
    
    // Set the port flow options
    settings.c_cflag &= ~PARENB;    // set no parity, 1 stop bit, 8 data bits
	settings.c_cflag &= ~CSTOPB;
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8;
    //settings.c_lflag |= (ICANON | ECHO | ECHOE);
    
    // Set the port settings
    set_termios(&settings);
}

SerialDevice::~SerialDevice()
{
    // Close the port if its open
    if(fd > -1) System::Close(fd);
}

// Port settings control
void SerialDevice::setBaudrate(unsigned int baudrate)
{
    // Get the current settings
    struct termios settings;
    get_termios(&settings);
    
    // Set the baudrate
    cfsetispeed(&settings, baudrate);
    cfsetospeed(&settings, baudrate);
    
    // Set the new settings
    set_termios(&settings);
}

// Raw control of port
void SerialDevice::set_termios(struct termios *settings)
{
    tcsetattr(fd, TCSANOW, settings);
}

void SerialDevice::get_termios(struct termios *settings)
{
    tcgetattr(fd, settings);
}

// Port status reading
unsigned int SerialDevice::available()
{
    unsigned int bytes = 0;
    ioctl(fd, FIONREAD, &bytes);
    return bytes;
}

void SerialDevice::flush()
{
    tcflush(fd, TCIOFLUSH);
}

// IO Operations
size_t SerialDevice::read(char *s, size_t n)
{
    return System::Read(fd, s, n);
}

size_t SerialDevice::write(char *s, size_t n)
{
    return System::Write(fd, s, n);
}

// Exceptions classes
SerialDeviceException::SerialDeviceException(std::string message)
{
    this->message = message;
}

// Access to some system functions
namespace System {
    // System read function
    size_t Read(int fd, void* buf, size_t count)
    {
        return read(fd, buf, count);
    }
    
    // System write function
    size_t Write(int fd, void* buf, size_t count)
    {
        return write(fd, buf, count);
    }
    
    // System close function
    void Close(int fd)
    {
        close(fd);
    }
}
