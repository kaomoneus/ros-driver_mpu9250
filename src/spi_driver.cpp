#include "spi_driver.h"

#include <pigpiod_if2.h>
#include <stdexcept>
#include <ros/ros.h>
#include <sstream>
#include <time.h>
#include <signal.h>

#include "SPIdev.h"

template <typename AddrT>
unsigned int WriteReg(AddrT WriteAddr, uint8_t WriteData)
{
    unsigned char tx[2] = {static_cast<unsigned char>(WriteAddr), WriteData};
    unsigned char rx[2] = {0};

    SPIdev::transfer("/dev/spidev0.1", tx, rx, 2);

    return rx[1];
}

template <typename AddrT>
unsigned int ReadReg(AddrT ReadAddr)
{
    return WriteReg((uint8_t)ReadAddr | (uint8_t)driver::misc::READ_FLAG, 0x00);
}

template <typename AddrT>
void ReadRegs(AddrT ReadAddr, uint8_t *ReadBuf, unsigned int Bytes)
{
    unsigned int  i = 0;

    unsigned char tx[255] = {0};
    unsigned char rx[255] = {0};

    tx[0] = (uint8_t)ReadAddr | (uint8_t)driver::misc::READ_FLAG;

    SPIdev::transfer("/dev/spidev0.1", tx, rx, Bytes + 1);

    for(i=0; i<Bytes; i++)
        ReadBuf[i] = rx[i + 1];

    usleep(50);
}

spi_driver::spi_driver()
{
}
spi_driver::~spi_driver()
{
}

void time_callback(union sigval timer_data){
    spi_driver *driver = static_cast<spi_driver*>(timer_data.sival_ptr);
    // Instruct the driver to read the available data from the registers.
    driver->read_data();
}

void spi_driver::initialize_backend() {

    // I2C Master mode
    WriteReg(driver::register_i2c_control::USER_CTRL, 0x20);
    // I2C configuration multi-master  IIC 400KHz
    WriteReg(driver::register_i2c_control::I2C_MST_CTRL, 0x0D);

    // Timer

    //  sigevent specifies behaviour on expiration
    struct sigevent sev = { 0 };

    itimerspec its;
    memset(&its, 0, sizeof(itimerspec));
    // Start delay 100ms
    its.it_value.tv_nsec = 100000;
    // Period 5ms
    its.it_interval.tv_nsec = 5000;

    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_notify_function = &time_callback;
    sev.sigev_value.sival_ptr = this;

    auto res = timer_create(CLOCK_REALTIME, &sev, &this->timerId);

    if (res != 0){
        std::stringstream message;
        message << "time_create error: " << strerror(errno);
        throw std::runtime_error(message.str());
    }

    /* start timer */
    res = timer_settime(timerId, 0, &its, NULL);

    if (res != 0){
        std::stringstream message;
        message << "time_settime error: " << strerror(errno);
        throw std::runtime_error(message.str());
    }
}

void spi_driver::deinitialize_backend() {
    timer_delete(this->timerId);
}

void spi_driver::write_mpu9250_register(register_mpu9250_type address, unsigned char value)
{
    WriteReg(address, value);
}
unsigned char spi_driver::read_mpu9250_register(register_mpu9250_type address)
{
    return ReadReg(address);
}
void spi_driver::read_mpu9250_registers(register_mpu9250_type address, unsigned int n_bytes, char *buffer)
{
    ReadRegs(address, (uint8_t*)buffer, n_bytes);
}

void spi_driver::write_ak8963_register(register_ak8963_type address, unsigned char value)
{
    WriteReg(driver::register_i2c_control::SLV0_ADDR, (uint8_t)driver::misc::AK8963_ADDR);
    WriteReg(driver::register_i2c_control::SLV0_REG, (uint8_t)driver::register_ak8963_type::CONTROL_1);
    WriteReg(driver::register_i2c_control::SLV0_DO, 0x12);
    WriteReg(driver::register_i2c_control::SLV0_CTRL, 0x81);
}
unsigned char spi_driver::read_ak8963_register(register_ak8963_type address)
{
    //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(
        driver::register_i2c_control::SLV0_ADDR,
        (uint8_t)driver::misc::AK8963_ADDR | (uint8_t)driver::misc::READ_FLAG
    );
    //I2C slave 0 register address from where to begin data transfer
    WriteReg(driver::register_i2c_control::SLV0_REG, (uint8_t)address);
    //Read 1 byte from the magnetometer
    WriteReg(driver::register_i2c_control::SLV0_CTRL, 0x81);
    return ReadReg(driver::register_i2c_control::EXT_SENS_DATA_00);
}
void spi_driver::read_ak8963_registers(register_ak8963_type address, unsigned int n_bytes, char *buffer)
{
    if (n_bytes > 15) {
        std::stringstream message;
        message << "Oh, gush, too much to be taken from ak stuff: n_bytes: " << n_bytes;
        throw std::runtime_error(message.str());
    }

    //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(
        driver::register_i2c_control::SLV0_ADDR,
        (uint8_t)driver::misc::AK8963_ADDR | (uint8_t)driver::misc::READ_FLAG
    );
    //I2C slave 0 register address from where to begin data transfer
    WriteReg(driver::register_i2c_control::SLV0_REG, (uint8_t)address);
    //Read 1 byte from the magnetometer
    WriteReg(driver::register_i2c_control::SLV0_CTRL, 0x80 + n_bytes);
    ReadRegs(driver::register_i2c_control::EXT_SENS_DATA_00, (uint8_t*)buffer, n_bytes);
}
