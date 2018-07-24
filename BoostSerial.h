#ifndef BOOST_SERIAL_H
#define BOOST_SERIAL_H

#include <array>
#include <vector>
#include <string>

#include <thread>
#include <mutex>

#include <sstream>
#include <iomanip>
#include <bitset>

#include <boost/asio.hpp>

class BoostSerial
{
  public:
    typedef boost::asio::serial_port_base::flow_control::type flowControlType;
    typedef boost::asio::serial_port_base::parity::type parityType;
    typedef boost::asio::serial_port_base::stop_bits::type stopBitsType;
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
    unsigned int bufferSize = 256;
    void printString(std::string const &);

  public:
    BoostSerial();
    ~BoostSerial();

    //write one byte
    void write(uint8_t);
    //write vector of bytes
    void writeBytes(std::vector<uint8_t> &&);
    void writeBytes(const std::vector<uint8_t> &);

    //print to stream any data using stringstream
    //second parameter depends on type
    //for int its bin/dec/oct/hex
    //for double its decimal point precision
    template <class T>
    void print(T, unsigned int = DEC);
    //for string use reference to avoid unnecessary copies
    void print(std::string const &);

    //same as above with newline added
    template <class T>
    void println(T, unsigned int = DEC);
    void println(std::string const &);

    //read one character/byte -1 if buffer is empty
    int16_t read();

    //read everything as a vector of bytes
    std::vector<uint8_t> readBytes();

    //read bytes until given value (removes it from buffer but doesn't includes it in the result)
    //or end of buffer
    //whichever was first
    std::vector<uint8_t> readBytesUntil(uint8_t);

    //read string until \0 or empty buffer
    std::string readString();

    //read to given character (doesn't includes it in the result but removes from buffer)
    //or to end the of buffer if character couldn't be found
    //or to \0
    //whichever is first
    std::string readStringUntil(char);

    //check next character in the buffer without removing it -1 if buffer is empty
    int16_t peek() const;

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
    unsigned int available() const;

    //clear buffer
    void flush();

    void setBaud(unsigned int = 115200);
    void setFlowControl(flowControlType = flowControlType::none);
    void setCharacterSize(unsigned int = 8);
    void setParity(parityType = parityType::none);
    void setStopBits(stopBitsType = stopBitsType::one);
    void setBufferSize(unsigned int = 256);

    unsigned int getBaud();
    flowControlType getFlowControl();
    unsigned int getCharacterSize();
    parityType getParity();
    stopBitsType getStopBits();
    unsigned int getBufferSize();
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

    //overflow
    if (usableBuffer.size() > buf.size())
    {
        //remove overflow
        //and lose data
        int overflow = usableBuffer.size() - buf.size();
        usableBuffer.erase(usableBuffer.begin(), usableBuffer.begin() + overflow);
    }
    lk.unlock();

    //read async again
    serial.async_read_some(
        boost::asio::buffer(buf, buf.size()),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            asyncReadHandler(error, bytes_transferred);
        });
}

void BoostSerial::printString(const std::string &s)
{
    serial.write_some(boost::asio::buffer(s, s.size()));
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

void BoostSerial::writeBytes(std::vector<uint8_t> &&v)
{
    serial.write_some(boost::asio::buffer(v, v.size()));
}

void BoostSerial::writeBytes(const std::vector<uint8_t> &v)
{
    serial.write_some(boost::asio::buffer(v, v.size()));
}

template <class T>
void BoostSerial::print(T a, unsigned int option)
{
    //write unknown data to stringstream
    std::stringstream ss;
    //for float treat option as precision
    if (std::is_floating_point<T>::value)
    {
        ss << std::setprecision(option) << std::fixed << a;
    }
    //for int treat it as print format
    else if (std::is_integral<T>::value)
    {
        //only for BIN
        bool trim = true;
        std::string str;

        switch (option)
        {
        case BIN:
            ss << std::bitset<sizeof(T) * 8>(a);
            str = ss.str();

            //remove leading zeroes
            while (trim)
            {
                if (!str.length())
                    trim = false;
                else if (str[0] == '1')
                    trim = false;
                else
                    str.erase(str.begin());
            }
            ss.str(std::string());
            ss.clear();
            ss << str;
            break;
        case HEX:
            ss << std::hex << a;
            break;
        case OCT:
            ss << std::oct << a;
            break;
        case DEC:
        default:
            ss << a;
            break;
        }
    }
    //do not use option value for non int nor float
    else
    {
        ss << a;
    }
    printString(ss.str());
}

void BoostSerial::print(const std::string &s)
{
    printString(s);
}

template <class T>
void BoostSerial::println(T t, unsigned int option)
{
    print(t, option);
    write('\n');
}

void BoostSerial::println(const std::string &s)
{
    printString(s);
    write('\n');
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

std::vector<uint8_t> BoostSerial::readBytes()
{
    std::vector<uint8_t> v;
    //move content of buffer to user's variable
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    v = std::move(usableBuffer);
    usableBuffer.clear();

    return v;
}

std::vector<uint8_t> BoostSerial::readBytesUntil(uint8_t t)
{
    std::vector<uint8_t> v;
    bool done = false;
    do
    {
        int16_t c = read();
        //c != end of stream and c != given character
        if (c != -1 && c != t)
            v.push_back((uint8_t)c);
        else
            done = true;
    } while (!done);
    return v;
}

std::string BoostSerial::readString()
{
    return readStringUntil('\0');
}

std::string BoostSerial::readStringUntil(char c)
{
    std::string s = "";
    bool done = false;
    do
    {
        int16_t x = read();
        //c != end of stream and c != given character and c != string terminator character
        if (x != -1 && x != c && x != '\0')
        {
            s += (char)x;
        }
        else
            done = true;
    } while (!done);
    return s;
}

int16_t BoostSerial::peek() const
{
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    //return -1 if there is no data available
    if (!usableBuffer.size())
        return -1;

    return usableBuffer[0];
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

    //set options for serial port
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud));
    serial.set_option(boost::asio::serial_port_base::flow_control(flowControl));
    serial.set_option(boost::asio::serial_port_base::character_size(characterSize));
    serial.set_option(boost::asio::serial_port_base::parity(parity));
    serial.set_option(boost::asio::serial_port_base::stop_bits(stopBits));

    //create thread that will read incoming data asynchronously
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

unsigned int BoostSerial::available() const
{
    //returns true if there are some bytes in the buffer
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    return usableBuffer.size();
}

void BoostSerial::flush()
{
    readBytes();
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

void BoostSerial::setBufferSize(unsigned int b)
{
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    bufferSize = b;
}

unsigned int BoostSerial::getBaud()
{
    return baud;
}

auto BoostSerial::getFlowControl() -> flowControlType
{
    return flowControl;
}

unsigned int BoostSerial::getCharacterSize()
{
    return characterSize;
}

auto BoostSerial::getParity() -> parityType
{
    return parity;
}

auto BoostSerial::getStopBits() -> stopBitsType
{
    return stopBits;
}

unsigned int BoostSerial::getBufferSize()
{
    std::unique_lock<std::mutex> lk(usableBufferMtx);
    return bufferSize;
}

#endif