# Boost-Serial-Port

This header file provides synchronous, non-blocking* access to utilize device's hardware serial port in Arduino-like manner<br /><br />

*write is blocking for now

## Usage

#### Write raw bytes
```cpp
void write(uint8_t);
void write(std::vector<uint8_t> &&);
void write(std::vector<uint8_t> &);
```

#### Print ascii formatted characters or strings with newline
```cpp
template<typename T>
  void print(T, unsigned int = BoostSerial::DEC);
void print(std::string const &);
 ```
 
#### Print ascii formatted characters or strings with newline
```cpp
template<typename T>
  void println(T, unsigned int = BoostSerial::DEC);
void println(std::string const &);
```

#### Read character or sequence of raw bytes from buffer
```cpp
int16_tread();
std::vector<uint8_t> readBytes();
std::vector<uint8_t> readBytesUntil(uint8_t);
```

#### Read strings
```cpp
std::string readString();
std::string readStringUntil(char);
```

#### Check next character in read buffer
```
int16_t peek() const;
```

#### Open serial port
```cpp
void open(std::string, unsigned int = 115200,
flowControlType = flowControlType::none,
unsigned int = 8,
parityType = parityType::none,
stopBitsType = stopBitsType::one);
```

#### check whether serial port is open
```cpp
bool isOpen() const;
```

#### Close serial port
```cpp
void close();
```

#### Check number of bytes available in the read buffer
```cpp
unsigned int available() const;
```

#### Clear read buffer
```cpp
void flush();
```

#### Set parameters of the serial port
```cpp
void etBaud(unsigned int = 115200);
void setFlowControl(flowControlType = flowControlType::none);
void setCharacterSize(unsigned int = 8);
void setPraity(parityType = parityType::none);
void setStopBits(stopBitsType = stopBitsType::one);
void setBufferSize(unsigned int = 256);
```

#### Get parameters of the serial port
```cpp
unsigned int getBaud();
flowControlType getFlowControl();
unsigned int getCharacterSize();
parityType getParity();
stopBitsType getStopBits();
unsigned int getBufferSize();
```
