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
    s.write(10);
    while (!s.available())
        ;
    ASSERT_EQ(10, s.read());
    s.flush();
}

TEST(Serial, writeBytestRval)
{
    s.writeBytes({1, 2, 5, 7, 19});
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 5, 7, 19}), s.readBytes());
    s.flush();
}

TEST(Serial, writeBytesLval)
{
    std::vector<uint8_t> v({1, 6, 10, 245});
    s.writeBytes(v);
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 6, 10, 245}), s.readBytes());
    s.flush();
}

TEST(Serial, printTemplate)
{
    s.print((int)11);
    while (!s.available())
        ;
    ASSERT_EQ("11", s.readString());
    s.flush();

    s.print("24");
    while (!s.available())
        ;
    ASSERT_EQ("24", s.readString());
    s.flush();

    s.print('c');
    while (!s.available())
        ;
    ASSERT_EQ('c', s.read());
    s.flush();

    s.print((float)21.2);
    while (!s.available())
        ;
    ASSERT_EQ("21.20", s.readString());
    s.flush();

    s.print((double)213.2, 1);
    while (!s.available())
        ;
    ASSERT_EQ("213.2", s.readString());
    s.flush();

    s.print(10, BoostSerial::HEX);
    while (!s.available())
        ;
    ASSERT_EQ("a", s.readString());
    s.flush();

    s.print(12, BoostSerial::OCT);
    while (!s.available())
        ;
    ASSERT_EQ("14", s.readString());
    s.flush();

    s.print(1333, BoostSerial::BIN);
    while (!s.available())
        ;
    ASSERT_EQ("10100110101", s.readString());
    s.flush();
}

TEST(Serial, printString)
{
    std::string t = "string1";
    s.print(t);
    while (!s.available())
        ;
    ASSERT_EQ("string1", s.readString());

    t = "string1\0 20";
    s.print(t);
    while (!s.available())
        ;
    ASSERT_EQ("string1", s.readString());
    s.flush();
}

TEST(Serial, printlnTemplate)
{
    s.println("Test");
    while (!s.available())
        ;
    ASSERT_EQ("Test\n", s.readString());
    s.flush();
}

TEST(Serial, printlnString)
{
    std::string t = "Min";
    s.println(t);
    while (!s.available())
        ;
    ASSERT_EQ("Min\n", s.readString());
    s.flush();
}

TEST(Serial, read)
{
    ASSERT_EQ(-1, s.read());

    s.write(20);
    while (!s.available())
        ;
    ASSERT_EQ(20, s.read());
}

TEST(Serial, readBytes)
{
    s.writeBytes(std::vector<uint8_t>({10, 20, 30, 44}));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({10, 20, 30, 44}), s.readBytes());
    s.flush();
}

TEST(Serial, readBytesUntil)
{
    s.writeBytes(std::vector<uint8_t>({1, 2, 4, 7, 22}));
    while (!s.available())
        ;
    ASSERT_EQ(std::vector<uint8_t>({1, 2, 4}), s.readBytesUntil(7));
    s.flush();
}

TEST(Serial, readString)
{
    s.print("Hello");
    while (!s.available())
        ;
    ASSERT_EQ("Hello", s.readString());
    s.flush();
}

TEST(Serial, readStringUntil)
{
    s.print("Mmmad");
    while (!s.available())
        ;
    ASSERT_EQ("Mmm", s.readStringUntil('a'));
    s.flush();
}

TEST(Serial, peek)
{
    ASSERT_EQ(-1, s.peek());
    s.print("abd");
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
    s.print("Hello");
    while (!s.available())
        ;
    ASSERT_EQ(5, s.available());
    s.flush();
}

TEST(Serial, flush)
{
    s.print("Sup");
    while (!s.available())
        ;
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