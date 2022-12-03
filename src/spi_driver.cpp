#include "spi_driver.h"

#include <pigpiod_if2.h>
#include <stdexcept>
#include <ros/ros.h>
#include <sstream>
#include <time.h>
#include <signal.h>

#include "SPIdev.h"

unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData)
{
    unsigned char tx[2] = {WriteAddr, WriteData};
    unsigned char rx[2] = {0};

    SPIdev::transfer("/dev/spidev0.1", tx, rx, 2);

    return rx[1];
}

unsigned int ReadReg(uint8_t ReadAddr)
{
    return WriteReg(ReadAddr | READ_FLAG, 0x00);
}

void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes)
{
    unsigned int  i = 0;

    unsigned char tx[255] = {0};
    unsigned char rx[255] = {0};

    tx[0] = ReadAddr | READ_FLAG;

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
    spi_driver *driver = timer_data.sival_ptr;
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

    res = timer_create(CLOCK_REALTIME, &sev, &this->timerId);

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
    ReadRegs(address, buffr, n_bytes);
}

void spi_driver::write_ak8963_register(register_ak8963_type address, unsigned char value)
{
    WriteReg(driver::register_i2c_control::SLV0_ADDR, driver::register_i2c_control::AK8963_ADDR);
    WriteReg(driver::register_i2c_control::SLV0_REG, AK8963_CNTL1);
    WriteReg(driver::register_i2c_control::SLV0_DO, 0x12);
    WriteReg(driver::register_i2c_control::SLV0_CTRL, 0x81);
}
unsigned char spi_driver::read_ak8963_register(register_ak8963_type address)
{
    WriteReg(driver::register_i2c_control::SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(driver::register_i2c_control::SLV0_REG, address); //I2C slave 0 register address from where to begin data transfer
    WriteReg(driver::register_i2c_control::SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    return ReadReg(driver::register_i2c_control::EXT_SENS_DATA_00);
}
void spi_driver::read_ak8963_registers(register_ak8963_type address, unsigned int n_bytes, char *buffer)
{
    if (n_bytes > 15) {
        std::stringstream message;
        message << "Oh, gush, too much to be taken from ak stuff: n_bytes: " << n_bytes;
        throw std::runtime_error(message.str());
    }

    WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, address); //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x80 + n_bytes); //Read 1 byte from the magnetometer
    ReadRegs(MPUREG_EXT_SENS_DATA_00, buffer, n_bytes)
}
