//
// Created by subbu on 2020-05-01.
//

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel
#define DEVICE_NAME                      "/dev/ttyUSB0"
#define BAUDRATE                        1000000
#define ADDR_MX_TORQUE_ENABLE           24
#define TORQUE_ENABLE                   1
#define ADDR_LOW_MX_GOAL_POSITION           30
#define ADDR_HIGH_MX_GOAL_POSITION           31

void showSuccess(const std::string &message) {
    std::cout << "[SUCCESS] "  << message << std::endl;
}

int showError(const std::string &message) {
    std::cerr << "[ERROR] "  << message << std::endl;
    return 1;
}

bool writeWord(uint8_t ID, uint8_t ADDR, uint16_t VALUE, dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, ID, ADDR, VALUE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        showError(packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    } else if (dxl_error != 0) {
        showError(packetHandler->getRxPacketError(dxl_error));
        return false;
    } else {
        return true;
    }
}

bool writeGoalPosition(uint8_t ID, uint64_t value, dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler) {
    int dxl_comm_result = COMM_TX_FAIL;                                                         // Communication result
    bool dxl_addparam_result = false;                                                           // addParam result
    uint8_t param_goal_position[2];
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_LOW_MX_GOAL_POSITION, 2);

    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE((int)value);
    param_goal_position[1] = DXL_HIBYTE((int)value);

    // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(ID, param_goal_position);
    if (!dxl_addparam_result) {
        showError("groupSyncWrite addparam failed");
        return false;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        showError(packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();
    return true;
}

int angleToPosition(double angle, double scale, double resolution, int zeroPosition) {
    return (int) (scale * (angle / resolution) + zeroPosition);
}

bool writeGoalAngle(uint8_t ID, double value, dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler) {
    int zeroPosition = ID % 2 == 0 ? 3072 : 1024;
    int position = angleToPosition( ID % 2 == 0 ? -value : value,
                                    1.0,
                                    0.088,
                                    zeroPosition);
    std::cout << "ID: " << (int)ID << " Position: " << position << std::endl;
    writeGoalPosition(ID, position, portHandler, packetHandler);
}

int main() {
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler->openPort()) {
        showSuccess("Successfully opened the device");
    } else {
        return showError("Failed to open device!");
    }

    if (portHandler->setBaudRate(BAUDRATE)) {
        showSuccess("Successfully set baudrate");
    } else {
        return showError("Failed to set baudrate!");
    }

    for(uint8_t i=1; i<=6; i++) {
        writeWord(i, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, portHandler, packetHandler);
    }

    writeGoalAngle(1, 0, portHandler, packetHandler);
    writeGoalAngle(2, 0, portHandler, packetHandler);
    writeGoalAngle(3, 0, portHandler, packetHandler);
    writeGoalAngle(4, 0, portHandler, packetHandler);
    writeGoalAngle(5, 0, portHandler, packetHandler);
    writeGoalAngle(6, 0, portHandler, packetHandler);

    return 0;
}