# Boost-Serial-Port

This header file provides asynchronous, non-blocking access to utilize device's hardware serial port in Arduino-like manner<br />
Every write function returns instantly.<br />

## Build
#### Linux
+ Requires boost_system and pthread linked.

## Usage

### Typedefs and enums
```cpp
typedef boost::asio::serial_port_base::flow_control::type flowControlType;
 ```
 **possible values:**
+ hardware / software / none
 
 ***
 ```cpp
typedef boost::asio::serial_port_base::parity::type parityType;
 ```
 **possible values:**
+ odd / even / none
 
 ***
 ```cpp
typedef boost::asio::serial_port_base::stop_bits::type stopBitsType;
 ```
 **possible values:**
+ one / onepointfive / two
 
***
 ```cpp
typedef boost::system::errc::errc_t errorCode;
 ```
 **possible values:**
+ see: https://www.boost.org/doc/libs/1_68_0/boost/system/error_code.hpp
 
***
 ```cpp
enum format;
```
 **possible values:**
 + BIN / OCT / DEC / HEX<br /><br />
***
### Open serial port
```cpp
void open(std::string name,
unsigned int baudRate = 115200,
flowControlType flowControl = flowControlType::none,
unsigned int characterSize = 8,
parityType parity = parityType::none,
stopBitsType stopBits = stopBitsType::one);
```
+ **name:** Name of the serial port (i.e COM1)
+ **Everything else:** should be self explanatory <br />
***
### Check whether serial port is open
```cpp
bool isOpen() const;
```
+ **returns:** True if serial port is open, false otherwise<br />
***
### Close serial port
```cpp
void close();
```

***
### Check whether error happened during async operation
```cpp
bool good() const;
```
+ **returns:** false if something happened, true on successful operation<br />
***
### Clear port's error flag
```cpp
void clear();
```
***
### Get error
```cpp
int getErr() const;
```
+ **returns:** error code of previous operation, see: https://www.boost.org/doc/libs/1_68_0/boost/system/error_code.hpp<br />
***

### Write raw bytes
```cpp
unsigned int write(uint8_t data);
```
+ **data:** A byte to write
<br />

+ **returns:** Number of bytes written<br /><br />

***
```cpp
unsigned int write(std::vector<uint8_t> const & data);
```
+ **data:** Vector of bytes to write
<br />

+ **returns:** Number of bytes written<br /><br />
***
### Print ascii formatted data
```cpp
template<typename T>
  unsigned int print(T const & data, unsigned int option = BoostSerial::DEC);
```
+ **data:** Some data to print. 
+ **option:** For integral variables its used to specify the format of given data (see BoostSerial::format)
+ **option:** For floating point variables its used to specify the decimal precision.
+ **option:** For anything else this argument is useless.
<br />

+ **returns:** Number of characters written<br /><br />
***
### Print ascii formatted data with newline
```cpp
template<typename T>
  unsigned int println(T const & data, unsigned int option = BoostSerial::DEC);
```
+ **data:** Some data to print. 
+ **option:** For integral variables its used to specify the format of given data (see BoostSerial::format)
+ **option:** For floating point variables its used to specify the decimal precision.
+ **option:** For anything else this argument is useless.
<br />

+ **returns:** Number of characters written<br /><br />
***
### Read raw byte or sequence of raw bytes from buffer
```cpp
int16_t read();
```
+ **returns:** First byte available in the buffer (removes it from the buffer) or -1 if the buffer is empty.<br /><br />

***
```cpp
std::vector<uint8_t> readBuffer();
```
+ **returns:** Current content of read buffer (clears the buffer)
***
```cpp
std::vector<uint8_t> readBytes(uint16_t len = 0xFFFF);
```
+ **len:** Number of bytes to read
<br/>

+ **returns:** vector of bytes of size **len** or smaller if given number of bytes hadn't been read before timeout (removes read characters from the buffer)<br /><br />

***
```cpp
std::vector<uint8_t> readBytesUntil(uint8_t terminator, uint16_t len = 0xFFFF);
```
+ **terminator:** Byte that will end the reading (this byte is not included in the return but is removed from the buffer)
+ **len:** Number of bytes to read
<br />

+ **returns:** Vector of bytes to given terminator or vector of size **len** if given terminator hadn't been found in **len** bytes or smaller vector if timeout happened and any of the previous conditions hadn't been met (read characters are removed from the buffer)<br />
***
### Read strings
```cpp
std::string readString();
```
+ **returns:** Everything from serial's read buffer as string. Stops reading on '\0' or on timeout<br /><br />

***
```cpp
std::string readStringUntil(char terminator = '\0');
```
+ **terminator:** Character that will end the reading (this character is not included in the return but is removed from the buffer)
<br />

+ **returns:** String to given terminator or smaller string if timeout happened and terminator hadn't been found (read characters are removed from the buffer).
***
### Check next character in read buffer
```
int16_t peek() const;
```
+ **returns:** First byte in the buffer (doesn't remove it) or -1 if the buffer is empty<br />
***
### Check number of bytes available in the read buffer
```cpp
unsigned int available() const;
```
+ **returns:** Number of bytes waiting in the read buffer<br />
***
### Check if serial port is in idle state
```cpp
bool idle() const;
```
+ **returns:** True if there is no write operation on serial port<br />
***
### Clear read buffer
```cpp
void flush();
```

***
### Set parameters of the serial port
```cpp
void setBaud(unsigned int baud = 115200);
void setFlowControl(flowControlType flowControl = flowControlType::none);
void setCharacterSize(unsigned int charSize = 8);
void setPraity(parityType parity = parityType::none);
void setStopBits(stopBitsType stopBits = stopBitsType::one);
void setBufferSize(unsigned int bufSize = 256);
void setTimeout(unsigned int timeinms = 1000);
```
setBufferSize affects the size of the internal read buffer. If overflow happens, the data that appeared first will be lost<br />
***
### Get parameters of the serial port
```cpp
unsigned int getBaud() const;
flowControlType getFlowControl() const;
unsigned int getCharacterSize() const;
parityType getParity() const;
stopBitsType getStopBits() const;
unsigned int getBufferSize() const;
unsigned int getTimeout() const;
```
