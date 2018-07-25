# Boost-Serial-Port

This header file provides synchronous, non-blocking* access to utilize device's hardware serial port in Arduino-like manner<br /><br />

*write is blocking for now

## Build
#### Linux
+ Requires boost_system and pthread linked.

## Usage

#### Open serial port
```cpp
void open(std::string name,
unsigned int baudRate = 115200,
flowControlType flowControl = flowControlType::none,
unsigned int characterSize = 8,
parityType parity = parityType::none,
stopBitsType stopBits = stopBitsType::one);
```
+ **name:** Name of the serial port (i.e COM1)
+ **Everything else:** should be self explanatory <br /><br />

#### check whether serial port is open
```cpp
bool isOpen() const;
```
+ **returns:** True if serial port is opened, false otherwise<br /><br />

#### Close serial port
```cpp
void close();
```
<br />

#### Write raw bytes
```cpp
void write(uint8_t data);
```
+ **data:** A byte to write
+ **returns:** Number of bytes written<br /><br />

```cpp
void write(std::vector<uint8_t> && data);
unsigned int write(std::vector<uint8_t> const & data);
```
+ **data:** Vector of bytes to write
+ **returns:** Number of bytes written<br /><br />
 
#### Print ascii formatted data
```cpp
template<typename T>
  unsigned int print(T data, unsigned int option = BoostSerial::DEC);
```
+ **data:** Some data to print. 
+ **option:** For integral variables its used to specify the format of given data (see BoostSerial::format)
+ **option:** For floating point variables its used to specify the decimal precision.
+ **option:** For anything else this argument is useless.
+ **returns:** Number of characters written<br /><br />

```cpp
unsigned int print(std::string const & data);
 ```
 + **data:** String to write
 + **returns:** Number of characters written<br /><br />
 
#### Print ascii formatted characters or strings with newline
```cpp
template<typename T>
  unsigned int println(T data, unsigned int option = BoostSerial::DEC);
```
+ **data:** Some data to print. 
+ **option:** For integral variables its used to specify the format of given data (see BoostSerial::format)
+ **option:** For floating point variables its used to specify the decimal precision.
+ **option:** For anything else this argument is useless.
+ **returns:** Number of characters written<br /><br />

```cpp
unsigned int println(std::string const & data);
```
 + **data:** String to write
 + **returns:** Number of characters written<br /><br />
 
#### Read character or sequence of raw bytes from buffer
```cpp
int16_t read();
```
+ **returns:** First byte available in the buffer (removes it from the buffer) or -1 if the buffer is empty.<br /><br />

```cpp
std::vector<uint8_t> readBytes();
```
+ **returns:** Everything from the serial read buffer (clears the buffer)<br /><br />

```cpp
std::vector<uint8_t> readBytesUntil(uint8_t terminator);
```
+ **terminator:** Byte that will end the reading (this byte is not included in the return but is removed from the buffer)
+ **returns:** Everything to given terminator or whole buffer if terminator wasn't found<br /><br />

#### Read strings
```cpp
std::string readString();
```
+ **returns:** Everything from serial's read buffer as string<br /><br />

```cpp
std::string readStringUntil(char terminator);
```
+ **terminator:** Character that will end the reading(this character is not included in the return but is removed from the buffer)
+ **returns:** Everything to given terminator or everything given to terminator character '\0' or whole buffer whichever was first<br /><br />

#### Check next character in read buffer
```
int16_t peek() const;
```
+ **returns:** First byte in the buffer (doesn't remove it) or -1 if the buffer is empty<br /><br />

#### Check number of bytes available in the read buffer
```cpp
unsigned int available() const;
```
+ **returns:** Number of bytes waiting in the read buffer<br /><br />
#### Clear read buffer
```cpp
void flush();
```
<br />

#### Set parameters of the serial port
```cpp
void etBaud(unsigned int = 115200);
void setFlowControl(flowControlType = flowControlType::none);
void setCharacterSize(unsigned int = 8);
void setPraity(parityType = parityType::none);
void setStopBits(stopBitsType = stopBitsType::one);
void setBufferSize(unsigned int = 256);
```
setBufferSize affects size of the internal read buffer. If overflow happens, the data that appeared first will be lost<br /><br />

#### Get parameters of the serial port
```cpp
unsigned int getBaud();
flowControlType getFlowControl();
unsigned int getCharacterSize();
parityType getParity();
stopBitsType getStopBits();
unsigned int getBufferSize();
```
