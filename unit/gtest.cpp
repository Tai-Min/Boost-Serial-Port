#include <thread>
#include <gtest/gtest.h>
#include "../BoostSerial.h"

/*
Arduino code
void setup()
{
    Serial.begin(9600);
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

TEST(Serial, write)
{
    ASSERT_EQ(1, s.write(10));
    while (!s.available())
        ;
    ASSERT_EQ(10, s.read());
}

TEST(Serial, writeBytestRval)
{
    ASSERT_EQ(5, s.write({1, 2, 5, 7, 19}));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 5, 7, 19}), s.readBytes());
}

TEST(Serial, writeBytesLval)
{
    std::vector<uint8_t> v({1, 6, 10, 245});
    ASSERT_EQ(4, s.write(v));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 10, 245}), s.readBytes());
}

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
}

TEST(Serial, printString)
{
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

TEST(Serial, printlnTemplate)
{
    ASSERT_EQ(5, s.println("Test"));
    while (!s.available())
        ;
    ASSERT_EQ("Test\n", s.readString());
}

TEST(Serial, printlnString)
{
    std::string t = "Min";
    ASSERT_EQ(4, s.println(t));
    while (!s.available())
        ;
    ASSERT_EQ("Min\n", s.readString());
}

TEST(Serial, read)
{
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
}

TEST(Serial, readBytesUntil)
{
    ASSERT_EQ(5, s.write(std::vector<uint8_t>({1, 2, 4, 7, 22})));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 4}), s.readBytesUntil(7));
    s.flush();
}

TEST(Serial, readString)
{
    ASSERT_EQ(5, s.print("Hello"));
    while (!s.available())
        ;
    ASSERT_EQ("Hello", s.readString());
}

TEST(Serial, readStringUntil)
{
    ASSERT_EQ(5, s.print("Mmmad"));
    while (!s.available())
        ;
    ASSERT_EQ("Mmm", s.readStringUntil('a'));
    ASSERT_EQ(1, s.available());
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
    ASSERT_EQ('b', s.peek());
    s.flush();
}

TEST(Serial, available)
{
    ASSERT_EQ(5, s.print("Hello"));
    while (!s.available())
        ;
    ASSERT_EQ(5, s.available());
    s.flush();
}

TEST(Serial, flush)
{
    ASSERT_EQ(3, s.print("Sup"));
    while (!s.available())
        ;
    ASSERT_EQ(3, s.available());
    s.flush();
    ASSERT_EQ(0, s.available());
}

int main(int argc, char **argv)
{
    s.open("/dev/ttyUSB0");
    if (!s.isOpen())
        return 1;
    s.setBaud(9600);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    ::testing::InitGoogleTest(&argc, argv);
    int res = RUN_ALL_TESTS();

    s.close();

    return res;
}