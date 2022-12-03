/// \file spi_driver.h
/// \brief Defines the spi_driver class.
#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "driver.h"

///
/// \brief An MPU9250 driver for the Raspberry Pi (SPI version).
///
class spi_driver : public driver
{
public:
    // CONSTRUCTORS
    ///
    /// \brief spi_driver Initializes a new Raspberry Pi driver.
    ///
    spi_driver();
    ~spi_driver() override;

    // METHODS
    void initialize_backend() override;

private:
    // VARIABLES
    int timerId = 0;
    // METHODS
    void write_mpu9250_register(register_mpu9250_type address, unsigned char value) override;
    unsigned char read_mpu9250_register(register_mpu9250_type address) override;
    void read_mpu9250_registers(register_mpu9250_type address, unsigned int n_bytes, char* buffer) override;

    void write_ak8963_register(register_ak8963_type address, unsigned char value) override;
    unsigned char read_ak8963_register(register_ak8963_type address) override;
    void read_ak8963_registers(register_ak8963_type address, unsigned int n_bytes, char* buffer) override;
};

#endif // SPI_DRIVER_H
