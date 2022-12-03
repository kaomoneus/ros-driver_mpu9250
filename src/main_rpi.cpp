#include "ros_node.h"
#include "rpi_driver.h"
#include "spi_driver.h"

std::shared_ptr<driver> create_driver() {
    auto param_interface_type = private_node.param<std::string>("interface_type", "spi");

    if (param_interface_type == "spi") {
        return std::make_shared<spi_driver>()
    } else if (param_interface_type == "i2c") {
        return std::make_shared<rpi_driver>();
    }

    std::stringstream message;
    message << "Unknown interface type: " << param_interface_type;
    throw std::runtime_error(message.str());
}

int main(int argc, char **argv)
{
    // Create the driver.
    auto driver = create_driver();

    // Create the node.
    ros_node node(driver, argc, argv);

    // Run the node.
    node.spin();
}
