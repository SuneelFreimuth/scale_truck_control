# Center Resiliency Coordinator (CRC) Module

The Center Resiliency Coordinator (CRC) module is designed to manage and coordinate the communication between trucks, specifically the Lead Vehicle (LV), First Follower Vehicle (FV1), and Second Follower Vehicle (FV2). The primary responsibility of this module is to predict and send velocities for each truck, taking into account the current state and sensor reliability.

## Table of Contents

- [Installation](#installation)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [API](#api)
- [License](#license)

## Installation

To install the CRC module, clone the repository, and include `crc.hpp` in your project:


## Dependencies

The CRC module depends on the `UDPsock` library for sending and receiving UDP data packets. Ensure that the `UDPsock` library is installed and configured correctly in your project.

## API

### CenterRC()

Constructor for the `CenterRC` class, initializing UDP sockets for sending and receiving data.

### ~CenterRC()

Destructor for the `CenterRC` class.

### init()

Initializes the necessary variables and settings for the UDP communication.

### UDPsendData(float pred_vel, int to)

Sends the predicted velocity (`pred_vel`) to the specified truck (`to`).

### UDPrecvTruckData()

Receives truck data from the UDP sockets and updates the variables accordingly.

### UDPrecvControlCenterData()

Receives control center data from the UDP sockets.

### PredictVelocity(int index)

Predicts the velocity for the specified truck (`index`) and returns the predicted velocity.

### ModeCheck(uint8_t lv_mode, uint8_t fv1_mode, uint8_t fv2_mode)

Checks and updates the CRC mode based on the modes of the trucks.

### Communicate()

Main function for communication between the CRC module and the trucks. It receives truck data, updates the CRC mode, predicts and sends velocities for each truck, and displays information on the console.

## Usage

To use the CRC module, create an instance of the `CenterResiliencyCoordinator::CenterRC` class in your code:

```cpp
#include "crc.hpp"

int main() {
    CenterResiliencyCoordinator::CenterRC crc;

    while (true) {
        crc.Communicate();
    }

    return 0;
}
