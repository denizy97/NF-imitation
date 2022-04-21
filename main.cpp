#include <stdio.h>
#include <cstring>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>
#include <sstream>
#include <string>

#include "falcon/core/FalconDevice.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/core/FalconLogger.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/gmtl/gmtl.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"

using namespace libnifalcon;
using namespace StamperKinematicImpl;

std::mutex mtx_goal; // mutex for updating goal position. Also used for updating the spring constant.

/**
 * @brief Inverse Kinematics: calculates the correct angles given the current world position of the Novint Falcon pointer
 * @copyright Function taken from libnifalcon: https://github.com/libnifalcon/libnifalcon
 * @param[in] angles            The appropriate angle given the world position of the Novint Falcon pointer
 * @param[out] worldPosition    Current world position of the Novint Falcon pointer
 */
void ComputeInverseKinematics(Angle& angles, const gmtl::Vec3d& worldPosition) {
    // First we need the offset vector from the origin of the XYZ coordinate frame to the
    // UVW coordinate frame:
    gmtl::Vec3d offset(-libnifalcon::r, -libnifalcon::s, 0);

    // Next lets convert the current end effector position into the UVW coordinates
    // of each leg:
    gmtl::Matrix33d R;
    R(0, 0) = cos(libnifalcon::phy[0]);
    R(0, 1) = sin(libnifalcon::phy[0]);
    R(0, 2) = 0;
    R(1, 0) = -sin(libnifalcon::phy[0]);
    R(1, 1) = cos(libnifalcon::phy[0]);
    R(1, 2) = 0;
    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = 1;
    gmtl::Vec3d P1 = R * worldPosition + offset;

    R(0, 0) = cos(libnifalcon::phy[1]);
    R(0, 1) = sin(libnifalcon::phy[1]);
    R(0, 2) = 0;
    R(1, 0) = -sin(libnifalcon::phy[1]);
    R(1, 1) = cos(libnifalcon::phy[1]);
    R(1, 2) = 0;
    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = 1;
    gmtl::Vec3d P2 = R * worldPosition + offset;

    R(0, 0) = cos(libnifalcon::phy[2]);
    R(0, 1) = sin(libnifalcon::phy[2]);
    R(0, 2) = 0;
    R(1, 0) = -sin(libnifalcon::phy[2]);
    R(1, 1) = cos(libnifalcon::phy[2]);
    R(1, 2) = 0;
    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = 1;
    gmtl::Vec3d P3 = R * worldPosition + offset;

    // Do the theta3's first. This is +/- but fortunately in the Falcon's case
    // only the + result is correct
    angles.theta3[0] = acos((P1[1] + libnifalcon::f) / libnifalcon::b);
    angles.theta3[1] = acos((P2[1] + libnifalcon::f) / libnifalcon::b);
    angles.theta3[2] = acos((P3[1] + libnifalcon::f) / libnifalcon::b);

    // Next find the theta1's
    // In certain cases could query the theta1 values directly and save a bit of processing
    // Again we have a +/- situation but only + is relevent
    double l01 = P1[2] * P1[2] + P1[0] * P1[0] + 2 * libnifalcon::c * P1[0] - 2 * libnifalcon::a * P1[0] + libnifalcon::a * libnifalcon::a +
                 libnifalcon::c * libnifalcon::c - libnifalcon::d * libnifalcon::d - libnifalcon::e * libnifalcon::e -
                 libnifalcon::b * libnifalcon::b * sin(angles.theta3[0]) * sin(angles.theta3[0]) -
                 2 * libnifalcon::b * libnifalcon::e * sin(angles.theta3[0]) - 2 * libnifalcon::b * libnifalcon::d * sin(angles.theta3[0]) -
                 2 * libnifalcon::d * libnifalcon::e - 2 * libnifalcon::a * libnifalcon::c;
    double l11 = -4 * libnifalcon::a * P1[2];
    double l21 = P1[2] * P1[2] + P1[0] * P1[0] + 2 * libnifalcon::c * P1[0] + 2 * libnifalcon::a * P1[0] + libnifalcon::a * libnifalcon::a +
                 libnifalcon::c * libnifalcon::c - libnifalcon::d * libnifalcon::d - libnifalcon::e * libnifalcon::e -
                 libnifalcon::b * libnifalcon::b * sin(angles.theta3[0]) * sin(angles.theta3[0]) -
                 2 * libnifalcon::b * libnifalcon::e * sin(angles.theta3[0]) - 2 * libnifalcon::b * libnifalcon::d * sin(angles.theta3[0]) -
                 2 * libnifalcon::d * libnifalcon::e + 2 * libnifalcon::a * libnifalcon::c;

    double l02 = P2[2] * P2[2] + P2[0] * P2[0] + 2 * libnifalcon::c * P2[0] - 2 * libnifalcon::a * P2[0] + libnifalcon::a * libnifalcon::a +
                 libnifalcon::c * libnifalcon::c - libnifalcon::d * libnifalcon::d - libnifalcon::e * libnifalcon::e -
                 libnifalcon::b * libnifalcon::b * sin(angles.theta3[1]) * sin(angles.theta3[1]) -
                 2 * libnifalcon::b * libnifalcon::e * sin(angles.theta3[1]) - 2 * libnifalcon::b * libnifalcon::d * sin(angles.theta3[1]) -
                 2 * libnifalcon::d * libnifalcon::e - 2 * libnifalcon::a * libnifalcon::c;
    double l12 = -4 * libnifalcon::a * P2[2];
    double l22 = P2[2] * P2[2] + P2[0] * P2[0] + 2 * libnifalcon::c * P2[0] + 2 * libnifalcon::a * P2[0] + libnifalcon::a * libnifalcon::a +
                 libnifalcon::c * libnifalcon::c - libnifalcon::d * libnifalcon::d - libnifalcon::e * libnifalcon::e -
                 libnifalcon::b * libnifalcon::b * sin(angles.theta3[1]) * sin(angles.theta3[1]) -
                 2 * libnifalcon::b * libnifalcon::e * sin(angles.theta3[1]) - 2 * libnifalcon::b * libnifalcon::d * sin(angles.theta3[1]) -
                 2 * libnifalcon::d * libnifalcon::e + 2 * libnifalcon::a * libnifalcon::c;

    double l03 = P3[2] * P3[2] + P3[0] * P3[0] + 2 * libnifalcon::c * P3[0] - 2 * libnifalcon::a * P3[0] + libnifalcon::a * libnifalcon::a +
                 libnifalcon::c * libnifalcon::c - libnifalcon::d * libnifalcon::d - libnifalcon::e * libnifalcon::e -
                 libnifalcon::b * libnifalcon::b * sin(angles.theta3[2]) * sin(angles.theta3[2]) -
                 2 * libnifalcon::b * libnifalcon::e * sin(angles.theta3[2]) - 2 * libnifalcon::b * libnifalcon::d * sin(angles.theta3[2]) -
                 2 * libnifalcon::d * libnifalcon::e - 2 * libnifalcon::a * libnifalcon::c;
    double l13 = -4 * libnifalcon::a * P3[2];
    double l23 = P3[2] * P3[2] + P3[0] * P3[0] + 2 * libnifalcon::c * P3[0] + 2 * libnifalcon::a * P3[0] + libnifalcon::a * libnifalcon::a +
                 libnifalcon::c * libnifalcon::c - libnifalcon::d * libnifalcon::d - libnifalcon::e * libnifalcon::e -
                 libnifalcon::b * libnifalcon::b * sin(angles.theta3[2]) * sin(angles.theta3[2]) -
                 2 * libnifalcon::b * libnifalcon::e * sin(angles.theta3[2]) - 2 * libnifalcon::b * libnifalcon::d * sin(angles.theta3[2]) -
                 2 * libnifalcon::d * libnifalcon::e + 2 * libnifalcon::a * libnifalcon::c;

    double T1a = (-l11 + sqrt(l11 * l11 - 4 * l01 * l21)) / (2 * l21);
    double T2a = (-l12 + sqrt(l12 * l12 - 4 * l02 * l22)) / (2 * l22);
    double T3a = (-l13 + sqrt(l13 * l13 - 4 * l03 * l23)) / (2 * l23);

    double T1b = (-l11 - sqrt(l11 * l11 - 4 * l01 * l21)) / (2 * l21);
    double T2b = (-l12 - sqrt(l12 * l12 - 4 * l02 * l22)) / (2 * l22);
    double T3b = (-l13 - sqrt(l13 * l13 - 4 * l03 * l23)) / (2 * l23);

    angles.theta1[0] = atan(T1b) * 2;
    angles.theta1[1] = atan(T2b) * 2;
    angles.theta1[2] = atan(T3b) * 2;

    // And finally calculate the theta2 values:
    angles.theta2[0] = acos((-P1[0] + libnifalcon::a * cos(angles.theta1[0]) - libnifalcon::c) /
                            (-libnifalcon::d - libnifalcon::e - libnifalcon::b * sin(angles.theta3[0])));
    angles.theta2[1] = acos((-P2[0] + libnifalcon::a * cos(angles.theta1[1]) - libnifalcon::c) /
                            (-libnifalcon::d - libnifalcon::e - libnifalcon::b * sin(angles.theta3[1])));
    angles.theta2[2] = acos((-P3[0] + libnifalcon::a * cos(angles.theta1[2]) - libnifalcon::c) /
                            (-libnifalcon::d - libnifalcon::e - libnifalcon::b * sin(angles.theta3[2])));
}

/**
 * @brief The velocity Jacobian where Vel=J*theta and Torque=J'*Force. Derivation in a slightly different style to Stamper
 * and may result in a couple of sign changes due to the configuration of the Falcon
 * @copyright Function taken from libnifalcon: https://github.com/libnifalcon/libnifalcon
 * @param[in] angles        Angles calculated from the inverse kinematics
 * @return gmtl::Matrix33d  Returns the Jacobian matrix
 */
gmtl::Matrix33d jacobian(const Angle& angles) {
    // Naming scheme:
    // Jx1 = rotational velocity of joint 1 due to linear velocity in x

    gmtl::Matrix33d J;

    // Arm1:
    double den =
        -libnifalcon::a * sin(angles.theta3[0]) * (sin(angles.theta1[0]) * cos(angles.theta2[0]) - sin(angles.theta2[0]) * cos(angles.theta1[0]));

    double Jx0 = cos(phy[0]) * cos(angles.theta2[0]) * sin(angles.theta3[0]) / den - sin(phy[0]) * cos(angles.theta3[0]) / den;
    double Jy0 = sin(phy[0]) * cos(angles.theta2[0]) * sin(angles.theta3[0]) / den + cos(phy[0]) * cos(angles.theta3[0]) / den;
    double Jz0 = (sin(angles.theta2[0]) * sin(angles.theta2[0])) / (den);

    // Arm2:
    den = -libnifalcon::a * sin(angles.theta3[1]) * (sin(angles.theta1[1]) * cos(angles.theta2[1]) - sin(angles.theta2[1]) * cos(angles.theta1[1]));

    double Jx1 = cos(phy[1]) * cos(angles.theta2[1]) * sin(angles.theta3[1]) / den - sin(phy[1]) * cos(angles.theta3[1]) / den;
    double Jy1 = sin(phy[1]) * cos(angles.theta2[1]) * sin(angles.theta3[1]) / den + cos(phy[1]) * cos(angles.theta3[1]) / den;
    double Jz1 = (sin(angles.theta2[1]) * sin(angles.theta2[1])) / (den);

    // Arm3:
    den = -libnifalcon::a * sin(angles.theta3[2]) * (sin(angles.theta1[2]) * cos(angles.theta2[2]) - sin(angles.theta2[2]) * cos(angles.theta1[2]));

    double Jx2 = cos(phy[2]) * cos(angles.theta2[2]) * sin(angles.theta3[2]) / den - sin(phy[2]) * cos(angles.theta3[2]) / den;
    double Jy2 = sin(phy[2]) * cos(angles.theta2[2]) * sin(angles.theta3[2]) / den + cos(phy[2]) * cos(angles.theta3[2]) / den;
    double Jz2 = (sin(angles.theta2[2]) * sin(angles.theta2[2])) / (den);

    J(0, 0) = Jx0;
    J(0, 1) = Jy0;
    J(0, 2) = Jz0;
    J(1, 0) = Jx1;
    J(1, 1) = Jy1;
    J(1, 2) = Jz1;
    J(2, 0) = Jx2;
    J(2, 1) = Jy2;
    J(2, 2) = Jz2;

    J.setState(J.FULL);
    invert(J);

    // ToDo: Check to see if Jacobian inverted properly.
    // If not we need to take action.

    return J;
}

/**
 * @brief Forward kinematics. Standard Newton-Raphson for linear systems using Jacobian to estimate slope. A small amount
 * of adjustment in the step size is all that is requried to guarentee convergence
 * @copyright Function taken from libnifalcon: https://github.com/libnifalcon/libnifalcon
 * @param[in] theta0        Current encoder angles of the Novint Falcon
 * @param[in out] pos       Current position of the Novint Falcon as well as new position based on the theta angles
 */
void ComputeForwardKinematics(const gmtl::Vec3d& theta0, gmtl::Vec3d& pos) {
    Angle angles;
    gmtl::Vec3d previousPos(pos);
    gmtl::Vec3d currentPos(pos);
    gmtl::Matrix33d J;
    gmtl::Vec3d delta;

    double targetError = 0.01;
    double previousError = 10000.0;
    double gradientAdjustment = 0.5;
    int maxTries = 15;

    bool done = 0;
    for (int i = 0; i < maxTries; i++) {
        // All we have initially are the three values for Theta0 and a guess of position

        // We can use the position guess to generate the angles at this position:
        ComputeInverseKinematics(angles, previousPos);
        // And these angles to find the Jacobian at the current position:
        J = jacobian(angles);
        // Then we can use the Jacobian to tell us which direction we need to move
        // in to rotate each theta0 to towards our desired values

        // Then we can see the difference between the actual and guess theta0:
        delta[0] = theta0[0] - angles.theta1[0];
        delta[1] = theta0[1] - angles.theta1[1];
        delta[2] = theta0[2] - angles.theta1[2];

        // Now use the Jacobian to tell us the direction:
        delta = J * delta;

        // And now we move along the adjustment vector
        // Nb: A good gradient descent algorithm would use more
        // intelligent step size adjustment. Here it only seems
        // to take a couple of steps to converge normally so we
        // simply start with a sensible step size and reduce it
        // if necessary to avoid oscillation about the target error.

        // Take the step size into account:
        delta *= gradientAdjustment;
        // And move the position guess:
        currentPos = previousPos + delta;

        // Let's see if we have got close enough to the target:
        // double error = sqrt(gmtl::dot(delta,delta));
        delta[0] = theta0[0] - angles.theta1[0];
        delta[1] = theta0[1] - angles.theta1[1];
        delta[2] = theta0[2] - angles.theta1[2];
        double error = dot(delta, delta);
        error = sqrt(error);
        previousPos = currentPos;

        if (error < targetError) {
            // Error is low enough so return the current position estimate
            pos = previousPos;
            // cout << i << endl;
            return;
        }
        // Error isn't small enough yet, see if we have over shot
        if ((error > previousError)) {
            // Whoops, over shot, reduce the stepsize next time:
            gradientAdjustment /= 2.0;
        }

        previousError = error;
    }

    // Failed to converge, leave last position as it was
    std::cout << "Failed to find the tool position in the max tries" << std::endl;
}

bool SetUpNovintFalcon(FalconDevice* nf_device, std::shared_ptr<FalconFirmware> ptr_falcon, int NF_idx) {
    unsigned int num_falcons = 0;

    if (!nf_device->getDeviceCount(num_falcons) || num_falcons == 0) {
        std::cout << "Cannot get device count" << std::endl;
        return false;
    } else {
        std::cout << "Falcons found: " << (int)num_falcons << std::endl;
    }

    // For now only use 1 Falcon: index 0
    if (!nf_device->open(NF_idx)) {
        // Novint Falcon is currently already opened, try to close and reopen it
        try {
            nf_device->close();
            if (nf_device->open(NF_idx)) {
                std::cout << "Opened falcon, required closing connection first" << std::endl;
            } else {
                std::cout << "Cannot open falcon - Error: " << nf_device->getErrorCode() << std::endl;
                return false;
            }
        } catch (const std::exception& e) {
            std::cout << e.what();
        }
    } else {
        std::cout << "Opened falcon" << std::endl;
    }

    if (!nf_device->isFirmwareLoaded()) {
        int i;
        std::cout << "Loading firmware" << std::endl;
        for (i = 0; i < 100; i++) {
            std::cout << "Attempt " << i << std::endl;
            if (nf_device->getFalconFirmware()->loadFirmware(true, NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
                                                                                         const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE))) {
                std::cout << "Loaded firmware" << std::endl;
                break;
            }
        }

        if (i == 101) {
            std::cout << "Could not load firmware" << std::endl;
            return false;
        }
    }

    // Homing mode to correct the origin of NF pointer
    nf_device->getFalconFirmware()->setHomingMode(true);

    return true;
}

/**
 * @brief Receive goal coordinate from the other NF
 * 
 * @param ip_send 
 * @param port 
 * @param sockfd 
 * @param goal 
 */
void threadReceiveCoordinate(const char * ip_send, unsigned short port, int sockfd, std::array<double, 3> *goal) {
    struct sockaddr_in sendaddr;
    memset(&sendaddr, 0, sizeof(sendaddr));
    sendaddr.sin_family = AF_INET; // IPv4
    sendaddr.sin_addr.s_addr = inet_addr(ip_send); // ip of other device
    sendaddr.sin_port = htons(port);

    char buffer[32];
    int n;
    socklen_t len = sizeof(sendaddr);

    while(true) {
        n = recvfrom(sockfd, (char *)buffer, 32, 
                MSG_WAITALL, ( struct sockaddr *) &sendaddr,
                &len);

        if (n <= 0)
        {
            std::cout << "error receiving UDP packet." << std::endl;
            exit(1);
        }
        buffer[n] = '\0';
        std::stringstream ss(buffer);

        //std::cout << buffer << std::endl;

        mtx_goal.lock();
        ss >> (*goal)[0];
        ss >> (*goal)[1];
        ss >> (*goal)[2];
        mtx_goal.unlock();
    }
}

/**
 * @brief Get a new spring constant as input
 * 
 * @param springConstant 
 */
void threadGetNewSpringConstant(int * springConstant){
    std::string springConstantString;
    while (true)
    {
        std::cin >> springConstantString;
        mtx_goal.lock();
        *springConstant = std::stoi(springConstantString);
        mtx_goal.unlock();
        std::cout << "spring constant updated to " << springConstantString << "!" << std::endl;        
    }
}


int main(int argc, char *argv[]) {
    // Get and parse arguments

    if (argc != 7)
    {
        std::cout << "Wrong usage. Please provide the following arguments:" << std::endl
        << "- spring constant (100 is a good value)" << std::endl
        << "- dampening constant (1000 is a good value)" << std::endl
        << "- ip address of the other NF"  << std::endl
        << "- self port (port of this device)" << std::endl 
        << "- send port (port of the other device)" << std::endl
        << "- device index (starting at 0)" << std::endl << std::endl;
        std::cout << "Example: sudo ./imitation 100 1000 192.168.0.104 2323" << std::endl;
        return 1;
    }
    int spring_constant = std::stoi(argv[1]);
    int dampening_constant = std::stoi(argv[2]);
    const char * ip_send = argv[3];
    unsigned short port_self = std::stoi(argv[4]);
    unsigned short port_send = std::stoi(argv[5]);
    int NF_idx = std::stoi(argv[6]);


    // Set up NF

    FalconDevice* nf_device = new FalconDevice();
    nf_device->setFalconFirmware<FalconFirmwareNovintSDK>();
    std::shared_ptr<FalconFirmware> ptr_falcon = nf_device->getFalconFirmware();

    if (!SetUpNovintFalcon(nf_device, ptr_falcon, NF_idx))
    {
        return 1;
    }    


    // Declaring some variables

    unsigned long last_time; // when did last loop end?
    unsigned long execution_time; // how long did loop take?
    unsigned long current_coordinate_time; // when did we take the coordinate?
    unsigned long last_coordinate_time = // when did we take the coordinate last time?
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); // should be fine
    unsigned long delta_time; // delta t for calculating velocity
    std::array<int, 3> enc_vec;
    std::array<double, 3> goal = {0, 0, 0.11};
    gmtl::Vec3d position = {0, 0, 0.11};
    gmtl::Vec3d last_position = {0.0, 0.0, 0.11};
    gmtl::Vec3d velocity;
    struct timespec t {0, 9200000};


    // Set up socket

    int sockfd;
    struct sockaddr_in selfaddr, sendaddr;
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        std::cout << "socket creation failed" << std::endl;
        exit(1);
    }
    memset(&selfaddr, 0, sizeof(selfaddr));
    memset(&sendaddr, 0, sizeof(sendaddr));
    selfaddr.sin_family = AF_INET; // IPv4
    selfaddr.sin_addr.s_addr = INADDR_ANY; // localhost
    selfaddr.sin_port = htons(port_self);
    sendaddr.sin_family = AF_INET; // IPv4
    sendaddr.sin_addr.s_addr = inet_addr(ip_send); // ip of other device
    sendaddr.sin_port = htons(port_send);
    if (bind(sockfd, (const struct sockaddr *)&selfaddr, sizeof(selfaddr)) < 0)
    {
        std::cout << "bind failed" << std::endl;
        exit(1);
    }
    socklen_t addr_len = sizeof(sendaddr);

    std::cout << "Starting thread to receive coordinates from other device" << std::endl;
    // Thread to receive goal coordinates from other NF
    std::thread t1(threadReceiveCoordinate, ip_send, port_send, sockfd, &goal);
    std::thread t2(threadGetNewSpringConstant, &spring_constant);

    last_time = // initialize at start, should be fine
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    std::cout << "Starting main loop!" << std::endl;
    // Main loop
    while (true)
    {
        nf_device->runIOLoop();
        //std::cout << execution_time << std::endl;

        position = {0.0, 0.0, 0.11};  // 0.11 offset to the Z-axis (default)

        // find delta t
        current_coordinate_time =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        delta_time = current_coordinate_time - last_coordinate_time;
        last_coordinate_time = current_coordinate_time;

        // Get current position
        std::array<int, 3> encoderPos = ptr_falcon->getEncoderValues();
        gmtl::Vec3d encoderAngles;
        encoderAngles[0] = nf_device->getFalconKinematic()->getTheta(encoderPos[0]);
        encoderAngles[1] = nf_device->getFalconKinematic()->getTheta(encoderPos[1]);
        encoderAngles[2] = nf_device->getFalconKinematic()->getTheta(encoderPos[2]);
        encoderAngles *= 0.0174532925;  // Convert to radians (pi/180)
        ComputeForwardKinematics(encoderAngles, position);

        // Calculate velocity
        velocity = (position - last_position);
        //std::cout << "trajectory is: (" << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << ")" << std::endl;
        velocity[0] = (1000*velocity[0]) / (delta_time/1000);
        velocity[1] = (1000*velocity[1]) / (delta_time/1000);
        velocity[2] = (1000*velocity[2]) / (delta_time/1000);
        last_position = position;

        //std::cout << "coordinates are: (" << position[0] << ", " << position[1] << ", " << position[2] << ")" << std::endl;
        //std::cout << "velocity is: (" << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << ")" << std::endl;
        
        // Send coordinates to other NF.
        std::string coor_str = std::to_string(position[0]) + " " + std::to_string(position[1]) + " " + std::to_string(position[2]);
        char coor_chars[32];
        strncpy(coor_chars, coor_str.c_str(), 32);
        sendto(sockfd, (const char *)coor_chars, strlen(coor_chars), 
        0, (const struct sockaddr *) &sendaddr,
            addr_len);

        // Force feedback
        gmtl::Vec3d force(0.0f, 0.9f, 0.0f); // Positive force is upwards - default: (0.0, 0.5-0.8, 0.0)
        enc_vec = {0, 0, 0};
        Angle nf_angles;

        ComputeInverseKinematics(nf_angles, position);

        // REMOVE FORCE LOGIC BELOW TO REMOVE FORCE FEEDBACK
        mtx_goal.lock();
        force += gmtl::Vec3d(spring_constant*(goal[0] - position[0]), spring_constant*(goal[1] - position[1]), spring_constant*(goal[2] - position[2]));
        mtx_goal.unlock();
        //std::cout << "force is: (" << force[0] << ", " << force[1] << ", " << force[2] << ")" << std::endl;
        force -= gmtl::Vec3d(dampening_constant*velocity[0], dampening_constant*velocity[1], dampening_constant*velocity[2]); // damping
        //std::cout << "force is: (" << force[0] << ", " << force[1] << ", " << force[2] << ")" << std::endl;

        // Dynamics

        // Jacobian
        gmtl::Matrix33d J;
        J = jacobian(nf_angles);

        // Convert force to motor torque values:
        J.setTranspose(J.getData());
        gmtl::Vec3d torque = J * force;

        // Now, we must scale the torques to avoid saturation of a motor
        // changing the ratio of torques and thus the force direction

        // Find highest torque:
        double maxTorque = 30.0;  // Rather random choice here, could be higher
        double largestTorqueValue = 0.0;
        int largestTorqueAxis = -1;
        for (int i = 0; i < 3; i++) {
            if (abs(torque[i]) > largestTorqueValue) {
                largestTorqueValue = abs(torque[i]);
                largestTorqueAxis = i;
            }
        }
        // If axis with the largest torque is over the limit, scale them all to
        // bring it back to the limit:
        if (largestTorqueValue > maxTorque) {
            double scale = largestTorqueValue / maxTorque;
            torque /= scale;
        }

        // Convert torque to motor voltages:
        torque *= 10000.0;
        enc_vec[0] = -torque[0];
        enc_vec[1] = -torque[1];
        enc_vec[2] = -torque[2];

        nf_device->getFalconFirmware()->setForces(enc_vec);
        nf_device->runIOLoop();

        execution_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - last_time;
        t.tv_nsec = execution_time > 940000
                        ? 0
                        : 940000 - execution_time;  // nanosleep(0) to yield timeslot for other processes (scheduling)
        nanosleep(&t, NULL);
        last_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }
    
}