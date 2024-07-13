#include "../include/PCA9685/PCA9685.h"

#include <cstdlib>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdexcept>
#include <iostream>

PCA9685::PCA9685(int address, uint8_t bus) {
    this->address = address;
    this->bus = bus;
}

void PCA9685::init() {
    std::string filename = "/dev/i2c-";
    filename += std::to_string(this->bus);
    this->i2cHandle = open(filename.c_str(), O_RDWR);
    if (this->i2cHandle < 0) {
        //throw std::runtime_error("AAAAAAAHHHH the i2c bus couldn't open PANIC PANIC PANIC!!");
        std::cerr << "the i2c bus couldn't open" << std::endl;
    }
    if (ioctl(this->i2cHandle, I2C_SLAVE, this->address) < 0) {
        //throw std::runtime_error("OH GOD! the darn thing isn't plugged in you silly head");
        std::cerr << "the darn thing isn't plugged in you silly head" << std::endl;
    }

    this->setRegister(0x00, 0b00010000);
    this->setRegister(0xFE, PRESCALE);
    this->setRegister(0x00, 0);
}

void PCA9685::setRegister(uint8_t reg, uint8_t val) {
    uint8_t packet[2] = {
            reg,
            val
    };
    if (write(this->i2cHandle, packet, 2) != 2) {
        //throw std::runtime_error("fail write");
        std::cerr << "fail write" << std::endl;
    }
}

int PCA9685::getRegister(uint8_t reg) {
    uint8_t buf[1] = {
            reg
    };

    if (write(this->i2cHandle, buf, 1) != 1) {
        //throw std::runtime_error("fail read 1");
        std::cerr << "fail read 1" << std::endl;
    }
    if (read(this->i2cHandle, buf, 1) != 1) {
        //throw std::runtime_error("fail read 2");
        std::cerr << "fail read 2" << std::endl;
    }
    return buf[0];
}

void PCA9685::setPWM(int channel, int on, int off) {
    if (channel > 15) {
        //throw std::domain_error("Channel is too large");
        std::cerr << "Channel is too large" << std::endl;
    }
    auto lowON = (uint8_t) (on & 0xFF);
    auto highON = (uint8_t) ((on >> 8) & 0xFF);

    auto lowOFF = (uint8_t) (off & 0xFF);
    auto highOFF = (uint8_t) ((off >> 8) & 0xFF);

    this->setRegister((uint8_t) (0x06 + (channel * 4)), lowON);
    this->setRegister((uint8_t) (0x06 + (channel * 4) + 1), highON);
    this->setRegister((uint8_t) (0x06 + (channel * 4) + 2), lowOFF);
    this->setRegister((uint8_t) (0x06 + (channel * 4) + 3), highOFF);
}
