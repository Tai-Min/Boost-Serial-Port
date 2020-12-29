#pragma once

#include <array>
#include <vector>
#include <string>

#include <thread>
#include <mutex>

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
    std::unique_ptr<boost::asio::io_service::work> serial_work;
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
    /**
     * @brief Class constructor.
     */
    BoostSerial();

    /**
     * @brief Class destructor.
     */
    virtual ~BoostSerial();

    /**
     * @brief Open serial port and start async read thread.
     *
     * @param dname Name of serial port to connect to.
     * @param baud_ Baud rate.
     * @param flowControl_ Flow control either none / software / hardware.
     * @param characterSize_ Character size.
     * @param parity_ Parity either none / odd / even.
     * @param stopBits Number of stop bits either one / onepointfive / two.
     */
    void open(std::string dname,
              unsigned int baud_ = 115200,
              flowControlType flowControl_ = flowControlType::none,
              unsigned int characterSize_ = 8,
              parityType parity_ = parityType::none,
              stopBitsType stopBits_ = stopBitsType::one);

    /**
     * @brief Check whether serial port is open.
     *
     * @return True if open, false otherwise.
     */
    bool isOpen() const;

    /**
     * @brief Close serial port.
     */
    void close();

    /**
     * @brief Check whether something happened to serial port.
     *
     * @return False if something happened, true on successful operation.
     */
    bool good() const;

    /**
     * @brief Clear port's error flag.
     */
    void clear();

    /**
     * @brief Get error code of previous operation
     *
     * @return Error code, see: https://www.boost.org/doc/libs/1_68_0/boost/system/error_code.hpp
     */
    int getErr() const;

    /**
     * @brief Write single byte.
     *
     * @param c Byte to be written.
     * @return Number of bytes written (should be 1).
     */
    unsigned int write(uint8_t c);

    /**
     * @brief Write vector of bytes.
     *
     * @param v Vector to be written.
     * @return Number of bytes written (should be size of v).
     */
    unsigned int write(std::vector<uint8_t> const &v);

    /**
     * @brief Write any data to serial port using stringstream.
     *
     * @param a Data to be written.
     * @param option For integers it's display option (either BIN, DEC, OCT, HEX),
     * for floating point it's decimal point precision
     * for others it's useless.
     * @return Number of bytes written.
     */
    template <class T>
    unsigned int print(T const &a, unsigned int option = DEC)
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

    /**
     * @brief Write any data to serial port using stringstream, appends a newline at the end.
     *
     * @param a Data to be written.
     * @param option For integers it's display option (either BIN, DEC, OCT, HEX),
     * for floating point it's decimal point precision
     * for others it's useless.
     * @return Number of bytes written.
     */
    template <class T>
    unsigned int println(T const &a, unsigned int option = DEC)
    {
        unsigned int r = print(a, option);
        write('\n');
        return r + 1;
    }

    /**
     * @brief Read one character from serial port.
     *
     * @return Byte or -1 if buffer is empty.
     */
    int16_t read();

    /**
     * @brief Read all available bytes to be read from serial port.
     *
     * @return Content of serial port's buffer. The buffer is cleared after this operation.
     */
    std::vector<uint8_t> readBuffer();

    /**
     * @brief Read bytes until number of bytes has been read or timeout.
     *
     * @param len Number of bytes to read.
     * @return Vector of bytes.
     */
    std::vector<uint8_t> readBytes(uint16_t len = 0xFFFF);

    /**
     * @brief Read bytes until given value or given number of bytes has been read or timeout, whichever criteria has been met first.
     *
     * @param givenByte The "stop" byte. This byte is not included in result vector.
     * @param len Maximum number of bytes to read.
     * @return Vector of bytes from serial port.
     */
    std::vector<uint8_t> readBytesUntil(uint8_t givenByte, uint16_t len = 0xFFFF);

    /**
     * @brief Read string until \0 or timeout.
     *
     * @return String received from serial port.
     */
    std::string readString();

    /**
     * @brief Read string until given character, \0 or timeout, whichever criteria has been met first.
     *
     * @param givenChar The "stop" character. This character is not included in result string.
     * @return String received from serial port.
     */
    std::string readStringUntil(char givenChar = '\0');

    /**
     * @brief Check next character in serial port's receive buffer without removing it.
     *
     * @return Next character in serial port's receive buffer or -1 if buffer is empty
     */
    int16_t peek() const;

    /**
     * @brief Check whether there is data awaiting in receive buffer.
     *
     * @return Number of bytes awaiting in receive buffer.
     */
    unsigned int available() const;

    /**
     * @brief Check whether serial port is idle.
     *
     * @return True if there is no write operation on serial port.
     */
    bool idle() const;

    /**
     * @brief Clear receive buffer.
     */
    void flush();

    /**
     * @brief Set baud rate.
     *
     * @param b Baud rate to be set.
     */
    void setBaud(unsigned int b = 115200);

    /**
     * @brief Set flow control.
     *
     * @param t Flow control to be set, either none / software / hardware.
     */
    void setFlowControl(flowControlType t = flowControlType::none);

    /**
     * @brief Set character size.
     *
     * @param s Character size to be set.
     */
    void setCharacterSize(unsigned int s = 8);

    /**
     * @brief Set paryty.
     *
     * @param t Parity to be set, either none / odd / even.
     */
    void setParity(parityType t = parityType::none);

    /**
     * @brief Set stop bits.
     *
     * @param t Stop bits to be set, either one / onepointfive / two.
     */
    void setStopBits(stopBitsType t = stopBitsType::one);

    /**
     * @brief Set buffer size.
     *
     * @param b Buffer size to be set.
     */
    void setBufferSize(unsigned int b = 256);

    /**
     * @brief Set timeout.
     *
     * @param t Timeout to be set in milliseconds.
     */
    void setTimeout(unsigned int t = 1000);

    /**
     * @brief Get baud rate.
     *
     * @return Baud rate of serial port.
     */
    unsigned int getBaud() const;

    /**
     * @brief Get flow control.
     *
     * @return Flow control of serial port.
     */
    flowControlType getFlowControl() const;

    /**
     * @brief Get character size.
     *
     * @return Character size of serial port.
     */
    unsigned int getCharacterSize() const;

    /**
     * @brief Get parity.
     *
     * @return Parity of serial port.
     */
    parityType getParity() const;

    /**
     * @brief Get stop bits.
     *
     * @return Stop bits of serial port.
     */
    stopBitsType getStopBits() const;

    /**
     * @brief Get buffer size.
     *
     * @return Buffer size of serial port.
     */
    unsigned int getBufferSize() const;

    /**
     * @brief Get timeout.
     *
     * @return Timeout of serial port.
     */
    unsigned int getTimeout() const;

    /**
     * @brief read bytes between read pattern and end pattern
     *
     * @return matched bytes in the buffer including start pattern and end pattern
     */
    std::vector<uint8_t> readBytePattern(std::vector<uint8_t> start_pattern, std::vector<uint8_t> end_pattern);
};
