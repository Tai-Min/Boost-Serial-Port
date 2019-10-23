#ifndef BOOST_SERIAL_H
#define BOOST_SERIAL_H

#include <array>
#include <vector>
#include <string>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <sstream>
#include <iomanip>
#include <bitset>

#include <chrono>
#include <boost/asio.hpp>

class BoostSerial
{
  public:
    typedef boost::asio::serial_port_base::flow_control::type flowControlType;
    typedef boost::asio::serial_port_base::parity::type parityType;
    typedef boost::asio::serial_port_base::stop_bits::type stopBitsType;
    typedef boost::system::errc::errc_t errorCode;
    enum format
    {
        BIN = 0,
        OCT = 1,
        DEC = 2,
        HEX = 3
    };

  private:
    //serial device stuff
    boost::asio::io_service serial_service;
    boost::asio::io_service::work serial_work;
    boost::asio::serial_port serial;

    //async stuff
    std::unique_ptr<std::thread> asyncReadThread;
    std::array<uint8_t, 256> buf;
    mutable std::mutex readBufferMtx; //for usableReadBuffer and readBufferSize
    std::vector<uint8_t> usableReadBuffer;
    void asyncReadHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

    mutable std::mutex writeMtx;     //for writeLocked
    std::condition_variable writeCv; // wait for notify from asyncWriteHandler
    bool writeLocked = false;        //block thread if previous async write hasn't finished
    void asyncWriteHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

    mutable std::mutex errMtx;
    int err;

    //serial config stuff
    int baud = 115200;
    //software / hardware / none
    flowControlType flowControl = flowControlType::none;
    unsigned int characterSize = 8;
    //odd / even / none
    parityType parity = parityType::none;
    //one / onepointfive / two
    stopBitsType stopBits = stopBitsType::one;
    unsigned int readBufferSize = 256;
    unsigned int timeoutVal = 1000;

    void printString(std::string const &);

  public:
    BoostSerial();
    ~BoostSerial();

    //open serial port and start async read thread
    void open(std::string,
              unsigned int = 115200,
              flowControlType = flowControlType::none,
              unsigned int = 8,
              parityType = parityType::none,
              stopBitsType = stopBitsType::one);
    bool isOpen() const;
    void close();
    bool good() const;

    void clear();
    int getErr() const;

    //write one byte
    unsigned int write(uint8_t);
    //write vector of bytes
    unsigned int write(std::vector<uint8_t> const &);

    //print to stream any data using stringstream
    //second parameter depends on type
    //for int its bin/dec/oct/hex
    //for double its decimal point precision
    //for others its useless
    template <class T>
    unsigned int print(T const &, unsigned int = DEC);

    //same as above with newline added
    template <class T>
    unsigned int println(T const &, unsigned int = DEC);

    //read one character/byte -1 if buffer is empty
    int16_t read();

    //read everything as a vector of bytes
    std::vector<uint8_t> readBuffer();

    //read bytes until given number of bytes has been read or timeout happens
    std::vector<uint8_t> readBytes(uint16_t = 0xFFFF);

    //read bytes until given value (removes it from buffer but doesn't includes it in the result)
    //or given number of bytes has been read
    //or timeout
    //whichever was first
    std::vector<uint8_t> readBytesUntil(uint8_t, uint16_t = 0xFFFF);

    //read string until \0
    //or timeout
    std::string readString();

    //read to given character (doesn't includes it in the result but removes from buffer)
    //or to end the of buffer if character couldn't be found
    //or to \0
    //whichever is first
    std::string readStringUntil(char = '\0');

    //check next character in the buffer without removing it -1 if buffer is empty
    int16_t peek() const;

    //returns whether there is data awaiting in the buffer
    unsigned int available() const;
    bool idle() const;

    //clear buffer
    void flush();

    void setBaud(unsigned int = 115200);
    void setFlowControl(flowControlType = flowControlType::none);
    void setCharacterSize(unsigned int = 8);
    void setParity(parityType = parityType::none);
    void setStopBits(stopBitsType = stopBitsType::one);
    void setBufferSize(unsigned int = 256);
    void setTimeout(unsigned int = 1000);

    unsigned int getBaud() const;
    flowControlType getFlowControl() const;
    unsigned int getCharacterSize() const;
    parityType getParity() const;
    stopBitsType getStopBits() const;
    unsigned int getBufferSize() const;
    unsigned int getTimeout() const;
};

#endif
