/// \file rpi_driver.h
/// \brief Defines the rpi_driver class.
#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

///
/// \brief An MPU9250 driver for the Raspberry Pi.
///
class spi_driver : public driver
{
public:
    // CONSTRUCTORS
    ///
    /// \brief rpi_driver Initializes a new Raspberry Pi driver.
    ///
    rpi_driver();
    ~rpi_driver() override;

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

#endif // RPI_DRIVER_H
