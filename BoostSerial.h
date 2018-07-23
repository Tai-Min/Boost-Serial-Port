#ifndef BOOST_SERIAL_H
#define BOOST_SERIAL_H

#include <array>
#include <vector>
#include <string>
#include <mutex>
#include <boost/asio.hpp>
#include <thread>

#include "MMaster.h"

class BoostSerial : public AbstractSerial
{
  public:
    typedef boost::asio::serial_port_base::flow_control::type flowControlType;
    typedef boost::asio::serial_port_base::parity::type parityType;
    typedef boost::asio::serial_port_base::stop_bits::type stopBitsType;

  private:
    //serial device stuff
    boost::asio::io_service serial_service;
    boost::asio::io_service::work serial_work;
    boost::asio::serial_port serial;

    //async stuff
    std::unique_ptr<std::thread> asyncReadThread;
    std::array<uint8_t, 256> buf;
    mutable std::mutex usableBufferMtx;
    std::vector<uint8_t> usableBuffer;
    void asyncReadHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

    //serial config stuff
    int baud = 115200;
    //software / hardware / none
    flowControlType flowControl = flowControlType::none;
    unsigned int characterSize = 8;
    //odd / even / none
    parityType parity = parityType::none;
    //one / onepointfive / two
    stopBitsType stopBits = stopBitsType::one;

  public:
    BoostSerial();
    ~BoostSerial();

    //write one byte
    void write(uint8_t);
    //write vector of bytes
    void writeBytes(std::vector<uint8_t>);
    //write string
    void writeString(std::string);

    //read one character/byte -1 if buffer is empty
    int16_t read();
    //read everything as a vector of bytes
    void readBytes(std::vector<uint8_t> &);
    //read string until \0
    void readString(std::string &);
    //read to given character (doesn't includes it in the result but removes from buffer)
    //or to end the of buffer if character couldn't be found
    //or to \0
    //whichever is first
    void readStringUntil(std::string &, char);

    //check next character in the buffer without removing it -1 if buffer is empty
    int16_t peek();

    //open serial port and stard async read thread
    void open(std::string,
              unsigned int = 115200,
              flowControlType = flowControlType::none,
              unsigned int = 8,
              parityType = parityType::none,
              stopBitsType = stopBitsType::one);
    bool isOpen() const;
    void close();

    //returns whether there is data awaiting in the buffer
    bool available() const;

    //clear buffer
    void flush();

    void setBaud(unsigned int = 115200);
    void setFlowControl(flowControlType = flowControlType::none);
    void setCharacterSize(unsigned int = 8);
    void setParity(parityType);
    void setStopBits(stopBitsType);
};

void BoostSerial::asyncReadHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    //place the content of buffer array into usableBuffer vector
    //usableBuffer is necessary because using handler buffer directly in read() or available() functions
    //fuck ups entire world for some reason
    //probably due to manipulating buffer in async_read_some
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    for (auto i = 0; i < bytes_transferred; i++)
    {
        usableBuffer.push_back(buf[i]);
    }
    lk.unlock();

    serial.async_read_some(
        boost::asio::buffer(buf, buf.size()),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            asyncReadHandler(error, bytes_transferred);
        });
}

BoostSerial::BoostSerial() : serial_service(), serial(serial_service), serial_work(serial_service), asyncReadThread(nullptr) {}

BoostSerial::~BoostSerial()
{
    if (serial.is_open())
        close();
}

void BoostSerial::write(uint8_t c)
{
    std::vector<uint8_t> v({c});
    writeBytes(v);
}

void BoostSerial::writeBytes(std::vector<uint8_t> v)
{
    serial.write_some(boost::asio::buffer(v, v.size()));
}

void BoostSerial::writeString(std::string s)
{
    serial.write_some(boost::asio::buffer(s, s.size()));
}

int16_t BoostSerial::read()
{
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    //return -1 if there is no data available
    if (!usableBuffer.size())
        return -1;

    //return first element and remove it from the buffer
    int res = usableBuffer[0];
    usableBuffer.erase(usableBuffer.begin());
    return res;
}

void BoostSerial::readBytes(std::vector<uint8_t> &v)
{
    //move content of buffer to user's variable
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    v = std::move(usableBuffer);
    usableBuffer.clear();
}

void BoostSerial::readString(std::string &s)
{
    /*s = "";
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    for (auto i : usableBuffer)
        s += i;
    usableBuffer.clear();*/
    readStringUntil(s, '\0');
}

void BoostSerial::readStringUntil(std::string &s, char c)
{
    s = "";
    char x;
    bool done = false;
    do
    {
        x = read();
        if (x != -1 || x != c || x != '\0')
        {
            s += x;
        }
        else
            done = true;
    } while (!done);
}

int16_t BoostSerial::peek()
{
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    //return -1 if there is no data available
    if (!usableBuffer.size())
        return -1;

    return usableBuffer[0];
    ;
}

void BoostSerial::open(std::string dname,
                       unsigned int baud_,
                       flowControlType flowControl_,
                       unsigned int characterSize_,
                       parityType parity_,
                       stopBitsType stopBits_)
{
    serial.open(dname);
    if (!serial.is_open())
        return;

    baud = baud_;
    flowControl = flowControl_;
    characterSize = characterSize_;
    parity = parity_;
    stopBits = stopBits_;

    serial.set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial.set_option(boost::asio::serial_port_base::flow_control(flowControl));
    serial.set_option(boost::asio::serial_port_base::character_size(characterSize));
    serial.set_option(boost::asio::serial_port_base::parity(parity));
    serial.set_option(boost::asio::serial_port_base::stop_bits(stopBits));

    //create thread that will read incoming data asynchronously
    //asyncReadThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &serial_service)));
    asyncReadThread.reset(new std::thread([this] { serial_service.run(); }));

    //push the first read request
    serial.async_read_some(
        boost::asio::buffer(buf, buf.size()),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            asyncReadHandler(error, bytes_transferred);
        });
}

bool BoostSerial::isOpen() const
{
    return serial.is_open();
}

void BoostSerial::close()
{
    if (!serial.is_open())
        return;

    //cancel pending async processes
    serial.cancel();

    //finish async read thread and delete it
    serial_service.stop();
    asyncReadThread->join();

    serial.close();
}

bool BoostSerial::available() const
{
    //returns true if there are some bytes in the buffer
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    return (bool)usableBuffer.size();
}

void BoostSerial::flush()
{
    std::vector<uint8_t> dull;
    if (available())
        readBytes(dull);
}

void BoostSerial::setBaud(unsigned int b)
{
    baud = b;
    if (serial.is_open())
        serial.set_option(boost::asio::serial_port_base::baud_rate(b));
}

void BoostSerial::setFlowControl(flowControlType t)
{
    flowControl = t;
    if (serial.is_open())
        serial.set_option(boost::asio::serial_port_base::flow_control(t));
}

void BoostSerial::setCharacterSize(unsigned int s)
{
    characterSize = s;
    if (serial.is_open())
        serial.set_option(boost::asio::serial_port_base::character_size(s));
}

void BoostSerial::setParity(parityType t)
{
    parity = t;
    if (serial.is_open())
        serial.set_option(boost::asio::serial_port_base::parity(t));
}

void BoostSerial::setStopBits(stopBitsType t)
{
    stopBits = t;
    if (serial.is_open())
        serial.set_option(boost::asio::serial_port_base::stop_bits(t));
}

#endif