#include "KincoDrive.h"

#include <iostream>

KincoDrive::KincoDrive(int NodeID) : Drive::Drive(NodeID) {
    //Remap torque reading and writting registers
    OD_Addresses[ACTUAL_TOR] = {0x6078, 0x00};
    OD_Addresses[TARGET_TOR] = {0x60F6, 0x08};
    //Weird Kinco error and DIOs addresses
    OD_Addresses[ERROR_WORD] = {0x2601, 0x00};
    //OD_Addresses[DIGITAL_IN] = {0x2010, 0x0B};
    //OD_Addresses[DIGITAL_IN] = {0x60FD, 0x00}; //Use default. 0x2010, 0x0B not working (SDO setup error)
    //OD_Addresses[DIGITAL_OUT] = {0x2010, 0x0E};
    //OD_Addresses[DIGITAL_OUT] = {0x60FE, 0x01}; //Use default. 0x2010, 0x0E not working (SDO setup error)
}
KincoDrive::~KincoDrive() {
    spdlog::debug("KincoDrive Deleted");
}

bool KincoDrive::init() {
    spdlog::debug("NodeID {} KincoDrive::init()", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    resetError();
    if(initPDOs()) {
        resetError();
        return true;
    }
    return false;
}
bool KincoDrive::init(motorProfile profile) {
    spdlog::debug("NodeID {} KincoDrive::init(motorProfile profile)", NodeID);
    preop();//Set preop first to disable PDO during initialisation
    resetError();
    if(setMotorProfile(profile)) {
        if(initPDOs()) {
            return true;
        }
    }
    return false;
}

bool KincoDrive::posControlConfirmSP() {
    // for kinco driver, there is no need to set postion control confirm
//    DEBUG_OUT("NodeID " << NodeID << " Kinco::posControlConfirmSP")
//    Drive::posControlConfirmSP();
    return true;
}

bool KincoDrive::initPosControl(motorProfile posControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Position Control", NodeID);

    sendSDOMessages(generatePosControlConfigSDO(posControlMotorProfile));
    /**
     * \todo Move jointMinMap and jointMaxMap to set additional parameters (bit 5 in 0x6041 makes updates happen immediately)
     *
     */
    return true;
}
bool KincoDrive::initPosControl() {
    spdlog::debug("NodeID {} Initialising Position Control", NodeID);

    sendSDOMessages(Drive::generatePosControlConfigSDO());
    return true;
}
bool KincoDrive::initVelControl(motorProfile velControlMotorProfile) {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    resetError();
    /**
     * \todo create velControlMOTORPROFILE and test on exo
     * \todo Tune velocity loop gain index 0x2381 to optimize V control
     *
    */
    sendSDOMessages(generateVelControlConfigSDO(velControlMotorProfile));
    return true;
}

bool KincoDrive::initVelControl() {
    spdlog::debug("NodeID {} Initialising Velocity Control", NodeID);
    resetError();
    /**
     * \todo create velControlMOTORPROFILE and test on exo
     * \todo Tune velocity loop gain index 0x2381 to optimize V control
     *
    */
    sendSDOMessages(Drive::generateVelControlConfigSDO());
    return true;
}

bool KincoDrive::initTorqueControl() {
    spdlog::debug("NodeID {} Initialising Torque Control", NodeID);
    sendSDOMessages(generateTorqueControlConfigSDO());

    return true;
}

bool KincoDrive::resetError(){
    spdlog::debug("NodeID {} reset error", NodeID);
    sendSDOMessages(generateResetErrorSDO());
    return true;
}

bool KincoDrive::initPDOs() {
    spdlog::debug("KincoDrive::initPDOs");

    spdlog::debug("Set up STATUS_WORD TPDO on Node {}", NodeID);
    int TPDO_Num = 1;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0xFF)) < 0) {
        spdlog::error("Set up STATUS_WORD TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up ACTUAL_POS and ACTUAL_VEL TPDO on Node {}", NodeID);
    TPDO_Num = 2;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_POS and ACTUAL_VEL TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up ACTUAL_TOR TPDO on Node {}", NodeID);
    TPDO_Num = 3;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up ACTUAL_TOR TPDO FAILED on node {}", NodeID);
        return false;
    }

    /*
    spdlog::debug("Set up DIGITAL_IN RPDO on Node {}", NodeID);
    TPDO_Num = 4;
    if (sendSDOMessages(generateTPDOConfigSDO(TPDO_MappedObjects[TPDO_Num], TPDO_Num, TPDO_COBID[TPDO_Num] + NodeID, 0x01)) < 0) {
        spdlog::error("Set up DIGITAL_IN TPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up CONTROL_WORD and DIGITAL_OUT RPDO on Node {}", NodeID);
    int RPDO_Num = 1;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up CONTROL_WORD and DIGITAL_OUT RPDO FAILED on node {}", NodeID);
        return false;
    }
    */

    spdlog::debug("Set up CONTROL_WORD RPDO on Node {}", NodeID);
    int RPDO_Num = 1;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up CONTROL_WORD RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_POS RPDO on Node {}", NodeID);
    RPDO_Num = 2;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up TARGET_POS RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_VEL RPDO on Node {}", NodeID);
    RPDO_Num = 3;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up ARGET_VEL RPDO FAILED on node {}", NodeID);
        return false;
    }

    spdlog::debug("Set up TARGET_TOR RPDO on Node {}", NodeID);
    RPDO_Num = 4;
    if (sendSDOMessages(generateRPDOConfigSDO(RPDO_MappedObjects[RPDO_Num], RPDO_Num, RPDO_COBID[RPDO_Num] + NodeID, 0xff)) < 0) {
        spdlog::error("Set up TARGET_TOR RPDO FAILED on node {}", NodeID);
        return false;
    }

    return true;
}

std::vector<std::string> KincoDrive::generatePosControlConfigSDO(motorProfile positionProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set control word to power up (enable)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable profile position mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 1";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set velocity profile
    sstream << "[1] " << NodeID << " write 0x6081 0 i32 " << std::dec << positionProfile.profileVelocity;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x6083 0 i32 " << std::dec << positionProfile.profileAcceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x6084 0 i32 " << std::dec << positionProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set instant position mode; important for kinco
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x103f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}
std::vector<std::string> KincoDrive::generateVelControlConfigSDO(motorProfile velocityProfile) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set control word to power up (enable)
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x0f";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //enable profile Velocity mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 3";
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 -3";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set velocity loop gain
    sstream << "[1] " << NodeID << " write 0x60F9 1 u16 " << std::dec << 100;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set acceleration profile
    sstream << "[1] " << NodeID << " write 0x6083 0 i32 " << std::dec << velocityProfile.profileAcceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    //Set deceleration profile
    sstream << "[1] " << NodeID << " write 0x6084 0 i32 " << std::dec << velocityProfile.profileDeceleration;
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}
std::vector<std::string> KincoDrive::generateTorqueControlConfigSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // start drive
    sstream << "[1] " << NodeID << " start";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());
    //enable Torque Control mode
    sstream << "[1] " << NodeID << " write 0x6060 0 i8 4";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> KincoDrive::generateResetErrorSDO() {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;

    // shutdown
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x06";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // reset fault
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x80";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    // reset fault
    sstream << "[1] " << NodeID << " write 0x6040 0 u16 0x06";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}

std::vector<std::string> KincoDrive::readSDOMessage(int address, int datetype) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // read message from drive
    sstream.str(std::string());

    switch (datetype) {
        case 1:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u8";
            break;
        case 2:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u16";
            break;
        case 3:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 i8";
            break;
        case 4:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 i32";
            break;
        default:
            sstream << "[1] " << NodeID << " read 0x" << std::hex << address << " 0 u8";
            break;
    }

    std::cout << sstream.str() << "\n";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    sendSDOMessages(CANCommands);
    return CANCommands;
}

std::vector<std::string> KincoDrive::writeSDOMessage(int address, int value) {
    // Define Vector to be returned as part of this method
    std::vector<std::string> CANCommands;
    // Define stringstream for ease of constructing hex strings
    std::stringstream sstream;
    // read message from drive
    sstream.str(std::string());
    sstream << "[1] " << NodeID << " write 0x" << std::hex << address << " 0 i32 0x" << std::hex << value;
    std::cout << sstream.str() << "\n";
    CANCommands.push_back(sstream.str());
    sstream.str(std::string());

    return CANCommands;
}
