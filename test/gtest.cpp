#include <thread>
#include <gtest/gtest.h>
#include "BoostSerial.h"

#define DEFAULT_TIMEOUT 15

/*
Arduino code
void setup()
{
    Serial.begin(115200);
}

void loop()
{

    if (Serial.available() > 0)
    {
        Serial.write(Serial.read());
    }
}
*/

BoostSerial s;

//check write() for one byte
TEST(Serial, write)
{
    //write one byte and check whether arduino responded with the same byte
    ASSERT_EQ(1, s.write(10));
    while (!s.available())
        ;
    ASSERT_EQ(10, s.read());
}

//check whether write() works with rvalues
TEST(Serial, writeBytestRval)
{
    //write some bytes and check whether arduino responded with the same bytes
    ASSERT_EQ(5, s.write({1, 2, 5, 7, 19}));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 5, 7, 19}), s.readBytes());
}

//check whether write() works with lvals
TEST(Serial, writeBytesLval)
{
    std::vector<uint8_t> v({1, 6, 10, 245});
    ASSERT_EQ(4, s.write(v));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 10, 245}), s.readBytes());
}

//check print() for various data types
TEST(Serial, printTemplate)
{
    ASSERT_EQ(2, s.print((int)11));
    while (!s.available())
        ;
    ASSERT_EQ("11", s.readString());

    ASSERT_EQ(2, s.print("24"));
    while (!s.available())
        ;
    ASSERT_EQ("24", s.readString());

    ASSERT_EQ(2, s.print("24\0 somechars"));
    while (!s.available())
        ;
    ASSERT_EQ("24", s.readString());

    ASSERT_EQ(1, s.print('c'));
    while (!s.available())
        ;
    ASSERT_EQ('c', s.read());

    ASSERT_EQ(5, s.print((float)21.2));
    while (!s.available())
        ;
    ASSERT_EQ("21.20", s.readString());

    ASSERT_EQ(5, s.print((double)213.2, 1));
    while (!s.available())
        ;
    ASSERT_EQ("213.2", s.readString());

    ASSERT_EQ(1, s.print(10, BoostSerial::HEX));
    while (!s.available())
        ;
    ASSERT_EQ("a", s.readString());

    ASSERT_EQ(2, s.print(12, BoostSerial::OCT));
    while (!s.available())
        ;
    ASSERT_EQ("14", s.readString());

    ASSERT_EQ(11, s.print(1333, BoostSerial::BIN));
    while (!s.available())
        ;
    ASSERT_EQ("10100110101", s.readString());

    std::string t = "string1";
    ASSERT_EQ(7, s.print(t));
    while (!s.available())
        ;
    ASSERT_EQ("string1", s.readString());

    t = "string1\0 20";
    ASSERT_EQ(7, s.print(t));
    while (!s.available())
        ;
    ASSERT_EQ("string1", s.readString());
}

//its print() with newline added, so nothing to test tbh
TEST(Serial, printlnTemplate)
{
    ASSERT_EQ(5, s.println("Test"));
    while (!s.available())
        ;
    ASSERT_EQ("Test\n", s.readString());

    std::string t = "Min";
    ASSERT_EQ(4, s.println(t));
    while (!s.available())
        ;
    ASSERT_EQ("Min\n", s.readString());
}

TEST(Serial, read)
{
    //should return -1 on empty buffer
    ASSERT_EQ(-1, s.read());

    ASSERT_EQ(1, s.write(20));
    while (!s.available())
        ;
    ASSERT_EQ(20, s.read());
}

TEST(Serial, readBytes)
{
    ASSERT_EQ(4, s.write(std::vector<uint8_t>({10, 20, 30, 44})));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({10, 20, 30, 44}), s.readBytes());

    //this should be not equal
    //because readBytes can't process it in given timeout
    s.setTimeout(2);
    ASSERT_EQ(36, s.write(std::vector<uint8_t>({10, 20, 30, 44, 2, 2, 4, 5, 6, 2, 3, 5, 6, 76, 2, 234, 5, 6, 7, 23, 3, 2, 3, 3, 3, 3, 1, 2, 2, 3, 3, 34, 3, 1, 1, 1})));
    while (!s.available())
        ;
    ASSERT_NE(std::vector<uint8_t>({10, 20, 30, 44, 2, 2, 4, 5, 6, 2, 3, 5, 6, 76, 2, 234, 5, 6, 7, 23, 3, 2, 3, 3, 3, 3, 1, 2, 2, 3, 3, 34, 3, 1, 1, 1}), s.readBytes());
    s.setTimeout(DEFAULT_TIMEOUT);

    //wait for the rest of the data
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIMEOUT * 20));
    s.flush();
}

TEST(Serial, readBytesUntil)
{
    ASSERT_EQ(5, s.write(std::vector<uint8_t>({1, 2, 4, 7, 22})));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 4}), s.readBytesUntil(7));
    s.flush();

    //this should be not equal
    //because readBytes can't process it in given timeout
    s.setTimeout(2);
    ASSERT_EQ(37, s.write(std::vector<uint8_t>({10, 20, 30, 44, 2, 2, 4, 5, 6, 2, 3, 5, 6, 76, 2, 234, 5, 6, 7, 23, 3, 2, 3, 3, 3, 3, 1, 2, 2, 3, 3, 34, 3, 1, 1, 1, 255})));
    while (!s.available())
        ;
    ASSERT_NE(std::vector<uint8_t>({10, 20, 30, 44, 2, 2, 4, 5, 6, 2, 3, 5, 6, 76, 2, 234, 5, 6, 7, 23, 3, 2, 3, 3, 3, 3, 1, 2, 2, 3, 3, 34, 3, 1, 1, 1}), s.readBytesUntil(255));
    s.setTimeout(DEFAULT_TIMEOUT);

    //wait for the rest of the data
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIMEOUT * 20));
    s.flush();
}

TEST(Serial, readString)
{
    ASSERT_EQ(5, s.print("Hello"));
    while (!s.available())
        ;
    ASSERT_EQ("Hello", s.readString());

    //this should be not equal
    //because readBytes can't process it in given timeout
    s.setTimeout(2);
    ASSERT_EQ(79, s.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"));
    while (!s.available())
        ;
    ASSERT_NE("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", s.readString());
    s.setTimeout(DEFAULT_TIMEOUT);

    //wait for the rest of the data
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIMEOUT * 20));
    s.flush();
}

TEST(Serial, readStringUntil)
{
    ASSERT_EQ(5, s.print("Mmmad"));
    while (!s.available())
        ;
    ASSERT_EQ("Mmm", s.readStringUntil('a'));
    while (!s.available())
        ;
    s.flush();

    //this should be not equal
    //because readBytes can't process it in given timeout
    s.setTimeout(2);
    ASSERT_EQ(92, s.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaaaaaaaaaaaaAAAAAAAAAAAABAAAAAAAAAA"));
    while (!s.available())
        ;
    ASSERT_NE("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaaaaaaaaaaaaAAAAAAAAAAAA", s.readStringUntil('B'));
    s.setTimeout(DEFAULT_TIMEOUT);

    //wait for the rest of the data
    std::this_thread::sleep_for(std::chrono::milliseconds(DEFAULT_TIMEOUT * 20));
    s.flush();
}

TEST(Serial, peek)
{
    ASSERT_EQ(-1, s.peek());
    ASSERT_EQ(3, s.print("abd"));
    while (!s.available())
        ;
    ASSERT_EQ('a', s.peek());
    ASSERT_EQ('a', s.peek());
    s.read();
    while (!s.available())
        ;
    ASSERT_EQ('b', s.peek());
    s.flush();
}

//check how this class behaves on big data
TEST(Serial, writeALot)
{
    s.setTimeout(1000);
    std::vector<uint8_t> data(10000);
    s.write(data);
    while (!s.available())
        ;
    ASSERT_EQ(data.size(), s.readBytes().size());
}

int main(int argc, char **argv)
{
    s.open("/dev/ttyUSB0");
    if (!s.isOpen())
        return 1;
    s.setBaud(115200);
    s.flush();
    s.setTimeout(DEFAULT_TIMEOUT);

    //wait for arduino
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    ::testing::InitGoogleTest(&argc, argv);
    int res = RUN_ALL_TESTS();

    s.close();

    return res;
}
