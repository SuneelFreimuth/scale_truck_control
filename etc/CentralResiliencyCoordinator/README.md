# Center Resiliency Coordinator (CRC) Module

The Center Resiliency Coordinator (CRC) module is designed to manage and coordinate the communication between trucks, specifically the Lead Vehicle (LV), First Follower Vehicle (FV1), and Second Follower Vehicle (FV2). The primary responsibility of this module is to predict and send velocities for each truck, taking into account the current state and sensor reliability.

## Table of Contents

- [How-it-Works](#How-it-Works)
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Usage](#usage)
- [API](#api)
- [License](#license)

## How-it-Works

1. The CRC module initializes the necessary UDP sockets for sending and receiving data from the trucks and the control center.
2. The module continuously receives data packets from each truck, containing the current velocities, distances, and reliability of the velocity sensors.
3. The CRC module processes the received data, checking the platooning mode of each truck, and updating its internal state accordingly.
4. The module then predicts the velocity for each truck based on their current state, platooning mode, and sensor reliability.
5. The CRC module sends the predicted velocities back to the trucks over UDP.
6. The trucks use the received velocities to adjust their speeds and maintain their positions within the platoon.
7. The CRC module prints out the current state, platooning mode, and predicted velocities for each truck to the console for monitoring and debugging purposes.

This continuous process ensures that the platooning system remains efficient, safe, and robust in the face of any sensor failures or unexpected changes in the truck's behavior.

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
