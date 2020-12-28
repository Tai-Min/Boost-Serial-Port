#include "BoostSerial.h"

#include <iostream>
using namespace std;
void BoostSerial::asyncReadHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    std::unique_lock<std::mutex> elk(errMtx);
    if (error) {
        err = error.value();
    }
    elk.unlock();

    if (error.value() == 995)
        return;//operation aborted

    //place the content of buffer array into usableReadBuffer vector
    //usableReadBuffer is necessary because using handler buffer directly in read() or available() functions
    //fuck ups entire class for some reason
    //probably due to manipulating buffer in async_read_some
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize and usableReadBuffer
    for (auto i = 0; i < bytes_transferred; i++) {
        usableReadBuffer.push_back(buf[i]);
    }

    //overflow
    if (usableReadBuffer.size() > readBufferSize) {
        //remove overflow
        //and lose data
        unsigned int overflow = usableReadBuffer.size() - readBufferSize;
        usableReadBuffer.erase(usableReadBuffer.begin(), usableReadBuffer.begin() + overflow);
    }
    lk.unlock();

    //read async again
    serial.async_read_some(
        boost::asio::buffer(buf, buf.size()),
    [this](const boost::system::error_code & error, std::size_t bytes_transferred) {
        asyncReadHandler(error, bytes_transferred);
    });
}

void BoostSerial::asyncWriteHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    std::unique_lock<std::mutex> elk(errMtx);
    if (error) {
        err = error.value();
    }
    elk.unlock();

    std::unique_lock<std::mutex> lk(writeMtx);
    writeLocked = false;
    lk.unlock();          //next write is possible
    writeCv.notify_one(); //if there is next write() waiting, notify it so it can continue
}

void BoostSerial::printString(const std::string &s)
{
    std::unique_lock<std::mutex> lk(writeMtx); //lock variable
    if (writeLocked)                           //if previous async is still writting
        writeCv.wait(lk);                      //pause this thread and wait until write is finished then lock variable again

    writeLocked = true; //then set locked and begin writting
    lk.unlock();

    boost::asio::async_write(
        serial,
        boost::asio::buffer(s, s.size()),
    [this](const boost::system::error_code & error, std::size_t bytes_transferred) {
        asyncWriteHandler(error, bytes_transferred);
    });
}

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
    [this](const boost::system::error_code & error, std::size_t bytes_transferred) {
        asyncWriteHandler(error, bytes_transferred);
    });

    return v.size();
}

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
    while (res.size() < len && !timeout) {
        //check whether timeout happened
        //and if so, set timeout flag
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= timeoutVal) {
            timeout = true;
        }
        //else check if there is data in the buffer
        //and if so, push it to res
        else {
            int16_t b = read();
            if (b > -1) {
                res.push_back(b);
                start = std::chrono::system_clock::now();
            }
        }
    }
    return res;
}

std::vector<uint8_t> BoostSerial::readBytesUntil(uint8_t givenByte, uint16_t len)
{
    auto start = std::chrono::system_clock::now();
    bool endFlag = false;
    std::vector<uint8_t> res;
    //check whether given number of bytes has been read
    //or whether timeout happened
    while (res.size() < len && !endFlag) {
        //check whether timeout happened
        //and if so, set timeout flag
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= timeoutVal) {
            endFlag = true;
        }
        //else check if there is data in the buffer
        //and if so, push it to res
        else {
            int16_t readByte = read();
            if (readByte == givenByte) {
                endFlag = true;
            } else if (readByte > -1) {
                res.push_back(readByte);
                start = std::chrono::system_clock::now();
            }
        }
    }
    return res;
}

std::string BoostSerial::readStringUntil(char givenChar)
{
    auto start = std::chrono::system_clock::now();
    bool endFlag = false;
    std::string res;
    //check whether given number of characters has been read
    //or whether timeout happened
    while (!endFlag) {
        //check whether timeout happened
        //and if so, set endFlag
        auto now = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (elapsed.count() >= timeoutVal) {
            endFlag = true;
        }

        else {
            int16_t readChar = read();
            //check if read character is given character
            //or string terminator
            //and if so, set endFlag flag
            if (readChar == givenChar || readChar == '\0') {
                endFlag = true;
            }
            //else check if there is data in the buffer
            //and if so, push it to res
            else if (readChar > -1) {
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
    //cleanup if port was already opened
    if (serial.is_open())
        close();

    try {
        serial.open(dname);
    } catch (...) {}

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

    serial_work.reset(new boost::asio::io_service::work(serial_service));
    //create thread that will read incoming data asynchronously
    asyncReadThread.reset(new std::thread([this] { serial_service.run(); }));

    //push the first read request
    serial.async_read_some(
        boost::asio::buffer(buf, buf.size()),
    [this](const boost::system::error_code & error, std::size_t bytes_transferred) {

        asyncReadHandler(error, bytes_transferred);
    });
}

void BoostSerial::close()
{
    if (!serial.is_open())
        return;

    clear();//clear read buffer

    //cancel pending async processes
    serial.cancel();

    //finish async read thread and delete it
    serial_service.stop();
    if (asyncReadThread.get() != nullptr) {
        asyncReadThread->join();
        asyncReadThread.reset(nullptr);
    }

    //reset io service for reopening
    serial_service.reset();

    serial.close();
}

bool BoostSerial::good() const
{
    std::unique_lock<std::mutex> lk(errMtx);
    return !err;
}

int BoostSerial::getErr()  const
{
    std::unique_lock<std::mutex> lk(errMtx);
    return err;
}

void BoostSerial::clear()
{
    std::unique_lock<std::mutex> lk(errMtx);
    err = errorCode::success;
}

unsigned int BoostSerial::available() const
{
    //returns true if there are some bytes in the buffer
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    return usableReadBuffer.size();
}

bool BoostSerial::idle() const
{
    std::unique_lock<std::mutex> lx(writeMtx);
    return !writeLocked;
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

unsigned int BoostSerial::getBufferSize() const
{
    std::unique_lock<std::mutex> lk(readBufferMtx); //for readBufferSize;
    return readBufferSize;
}
