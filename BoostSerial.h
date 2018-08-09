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

    std::mutex writeMtx;             //for writeLocked
    std::condition_variable writeCv; // wait for notify from asyncWriteHandler
    bool writeLocked = false;        //block thread if previous async write hasn't finished
    void asyncWriteHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

    std::mutex errMtx;
    bool errFlag;
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

    //write one byte
    unsigned int write(uint8_t);
    //write vector of bytes
    unsigned int write(std::vector<uint8_t> &&);
    unsigned int write(std::vector<uint8_t> const &);

    //print to stream any data using stringstream
    //second parameter depends on type
    //for int its bin/dec/oct/hex
    //for double its decimal point precision
    //for others its useless
    template <class T>
    unsigned int print(T const &, unsigned int = DEC);
    //for string use reference to avoid unnecessary copies
    //unsigned int print(std::string const &);

    //same as above with newline added
    template <class T>
    unsigned int println(T const &, unsigned int = DEC);
    //unsigned int println(std::string const &);

    //read one character/byte -1 if buffer is empty
    int16_t read();

    //read everything as a vector of bytes
    std::vector<uint8_t> readBuffer();

    //read bytes until given number of bytes has been read or timeout happens
    std::vector<uint8_t> readBytes(uint16_t = 0xFFFF); //TODO

    //read bytes until given value (removes it from buffer but doesn't includes it in the result)
    //or given number of bytes has been read
    //or timeout
    //whichever was first
    std::vector<uint8_t> readBytesUntil(uint8_t, uint16_t = 0xFFFF); //TODO

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

    //open serial port and stard async read thread
    void open(std::string,
              unsigned int = 115200,
              flowControlType = flowControlType::none,
              unsigned int = 8,
              parityType = parityType::none,
              stopBitsType = stopBitsType::one);
    bool isOpen() const;
    void close();
    bool good();
    
    void clear();
    void cancel();

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
    void setTimeout(unsigned int = 1000);

    int getErr();
    unsigned int getBaud();
    flowControlType getFlowControl();
    unsigned int getCharacterSize();
    parityType getParity();
    stopBitsType getStopBits();
    unsigned int getBufferSize();
    unsigned int getTimeout();
};

void BoostSerial::asyncReadHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    std::unique_lock<std::mutex> elk(errMtx);
    if (error)
    {
        err = error.value();
        errFlag = 1;
    }
    elk.unlock();

    //place the content of buffer array into usableReadBuffer vector
    //usableReadBuffer is necessary because using handler buffer directly in read() or available() functions
    //fuck ups entire class for some reason
    //probably due to manipulating buffer in async_read_some
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize and usableReadBuffer
    for (auto i = 0; i < bytes_transferred; i++)
    {
        usableReadBuffer.push_back(buf[i]);
    }

    //overflow
    if (usableReadBuffer.size() > readBufferSize)
    {
        //remove overflow
        //and lose data
        unsigned int overflow = usableReadBuffer.size() - readBufferSize;
        usableReadBuffer.erase(usableReadBuffer.begin(), usableReadBuffer.begin() + overflow);
    }
    lk.unlock();

    //read async again
    serial.async_read_some(
        boost::asio::buffer(buf, buf.size()),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            asyncReadHandler(error, bytes_transferred);
        });
}

void BoostSerial::asyncWriteHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    std::unique_lock<std::mutex> elk(errMtx);
    if (error)
    {
        err = error.value();
        errFlag = 1;
    }
    elk.unlock();

    std::unique_lock<std::mutex> lk(writeMtx);
    writeLocked = false;
    lk.unlock();          //next write is possible
    writeCv.notify_one(); //if there is next write() waiting, notify it so it can continue
}

void BoostSerial::printString(const std::string &s)
{
    //TODO to change
    //so it doesn't repeat write() function
    std::unique_lock<std::mutex> lk(writeMtx); //lock variable
    if (writeLocked)                           //if previous async is still writting
        writeCv.wait(lk);                      //pause this thread and wait until write is finished then lock variable again

    writeLocked = true; //then set locked and begin writting
    lk.unlock();

    boost::asio::async_write(
        serial,
        boost::asio::buffer(s, s.size()),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            asyncWriteHandler(error, bytes_transferred);
        });
}

BoostSerial::BoostSerial() : serial_service(), serial(serial_service), serial_work(serial_service), asyncReadThread(nullptr) {}

BoostSerial::~BoostSerial()
{
    if (serial.is_open())
        close();
}

unsigned int BoostSerial::write(uint8_t c)
{
    std::vector<uint8_t> v({c});
    return write(v);
}

unsigned int BoostSerial::write(std::vector<uint8_t> &&v)
{
    return write(v);
}

unsigned int BoostSerial::write(const std::vector<uint8_t> &v)
{
    std::unique_lock<std::mutex> lk(writeMtx); //lock variable
    if (writeLocked)                           //if previous async is still writting
        writeCv.wait(lk);                      //pause this thread and wait until write is finished then lock variable again

    writeLocked = true; //then set locked and begin writting
    lk.unlock();        //asyncWriteHandler can access writeLocked now

    boost::asio::async_write(
        serial,
        boost::asio::buffer(v, v.size()),
        [this](const boost::system::error_code &error, std::size_t bytes_transferred) {
            asyncWriteHandler(error, bytes_transferred);
        });

    return v.size();
}

template <class T>
unsigned int BoostSerial::print(T const &a, unsigned int option)
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
    std::string res = ss.str();
    printString(res);
    return res.length();
}

/*unsigned int BoostSerial::print(const std::string &s)
{
    printString(s);
    return s.length();
}*/

template <class T>
unsigned int BoostSerial::println(T const & t, unsigned int option)
{
    unsigned int r = print(t, option);
    write('\n');
    return r + 1;
}

/*unsigned int BoostSerial::println(const std::string &s)
{
    printString(s);
    write('\n');
    return s.length() + 1;
}*/

int16_t BoostSerial::read()
{
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    //return -1 if there is no data available
    if (!usableReadBuffer.size())
        return -1;

    //return first element and remove it from the buffer
    int res = usableReadBuffer[0];
    usableReadBuffer.erase(usableReadBuffer.begin());
    return res;
}

std::vector<uint8_t> BoostSerial::readBuffer()
{
    std::vector<uint8_t> v;
    //move content of buffer to user's variable
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    v = std::move(usableReadBuffer);
    usableReadBuffer.clear();

    return v;
}

std::vector<uint8_t> BoostSerial::readBytes(uint16_t len)
{
    auto start = std::chrono::system_clock::now();
    bool timeout = false;
    std::vector<uint8_t> res;
    //check whether given number of bytes has been read
    //or whether timeout happened
    while (res.size() < len && !timeout)
    {
        //check whether timeout happened
        //and if so, set timeout flag
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= timeoutVal)
        {
            timeout = true;
        }
        //else check if there is data in the buffer
        //and if so, push it to res
        else
        {
            int16_t b = read();
            if (b > -1)
            {
                res.push_back(b);
            }
        }
    }
    return res;
}

std::vector<uint8_t> BoostSerial::readBytesUntil(uint8_t givenByte, uint16_t len)
{
    auto start = std::chrono::system_clock::now();
    bool timeout = false;
    bool endFlag = false;
    std::vector<uint8_t> res;
    //check whether given number of bytes has been read
    //or whether timeout happened
    while (res.size() < len && !endFlag)
    {
        //check whether timeout happened
        //and if so, set timeout flag
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= timeoutVal)
        {
            endFlag = true;
        }
        //else check if there is data in the buffer
        //and if so, push it to res
        else
        {
            int16_t readByte = read();
            if (readByte == givenByte)
            {
                endFlag = true;
            }
            else if (readByte > -1)
            {
                res.push_back(readByte);
            }
        }
    }
    return res;
}

std::string BoostSerial::readString()
{
    return readStringUntil();
}

std::string BoostSerial::readStringUntil(char givenChar)
{
    auto start = std::chrono::system_clock::now();
    bool endFlag = false;
    std::string res;
    //check whether given number of characters has been read
    //or whether timeout happened
    while (!endFlag)
    {
        //check whether timeout happened
        //and if so, set endFlag
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= timeoutVal)
        {
            endFlag = true;
        }

        else
        {
            int16_t readChar = read();
            //check if read character is given character
            //or string terminator
            //and if so, set endFlag flag
            if (readChar == givenChar || readChar == '\0')
            {
                endFlag = true;
            }
            //else check if there is data in the buffer
            //and if so, push it to res
            else if (readChar > -1)
            {
                res += readChar;
            }
        }
    }
    return res;
}

int16_t BoostSerial::peek() const
{
    //std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    std::unique_lock<std::mutex> lk(readBufferMtx);
    //return -1 if there is no data available
    if (!usableReadBuffer.size())
        return -1;

    return usableReadBuffer[0];
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

bool BoostSerial::good()
{
    std::unique_lock<std::mutex> lk(errMtx);
    return errFlag;
}

int BoostSerial::getErr()
{
    std::unique_lock<std::mutex> lk(errMtx);
    return err;
}

void BoostSerial::clear()
{
    std::unique_lock<std::mutex> lk(errMtx);
    errFlag = 0;
    err = errorCode::success;
}

void BoostSerial::cancel()
{
    serial.cancel();
}

unsigned int BoostSerial::available() const
{
    //returns true if there are some bytes in the buffer
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    return usableReadBuffer.size();
}

void BoostSerial::flush()
{
    readBuffer();
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
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    readBufferSize = b;
}

void BoostSerial::setTimeout(unsigned int t)
{
    timeoutVal = t;
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
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    return readBufferSize;
}

unsigned int BoostSerial::getTimeout()
{
    return timeoutVal;
}

#endif