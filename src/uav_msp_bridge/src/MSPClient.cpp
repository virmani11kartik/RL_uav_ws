#include "uav_msp_bridge/MSPClient.hpp"
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iomanip>

namespace uav_msp_bridge {

MSPClient::MSPClient(const std::string& port, int baud)
    : port_(port), baud_(baud), fd_(-1) {}

MSPClient::~MSPClient() {
    close();
}

bool MSPClient::open() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "Error opening " << port_ << ": " << strerror(errno) << std::endl;
        return false;
    }
    
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    // Set baud rate
    speed_t speed;
    switch (baud_) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
            std::cerr << "Unsupported baud rate: " << baud_ << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1 mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    // Flush buffers
    tcflush(fd_, TCIOFLUSH);
    
    return true;
}

void MSPClient::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool MSPClient::isOpen() const {
    return fd_ >= 0;
}

bool MSPClient::request(uint8_t cmd, const std::vector<uint8_t>& payload,
                        std::vector<uint8_t>& response, double timeout_sec) {
    if (!isOpen()) {
        return false;
    }
    
    response.clear();
    
    // CRITICAL: Flush input buffer before sending request
    tcflush(fd_, TCIFLUSH);
    
    // Build and send MSP frame
    std::vector<uint8_t> frame = buildFrame(cmd, payload);
    ssize_t written = write(fd_, frame.data(), frame.size());
    if (written != (ssize_t)frame.size()) {
        std::cerr << "Write error: wrote " << written << " of " << frame.size() << " bytes" << std::endl;
        return false;
    }
    
    // Wait for response
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(timeout_sec);
    
    while (std::chrono::steady_clock::now() - start < timeout) {
        uint8_t direction;
        std::vector<uint8_t> recv_payload;
        uint8_t recv_cmd;
        
        if (readFrame(direction, recv_cmd, recv_payload)) {
            if (recv_cmd == cmd) {
                if (direction == '>') {
                    response = recv_payload;
                    return true;
                } else if (direction == '!') {
                    std::cerr << "MSP error response for command " << (int)cmd << std::endl;
                    return false;
                }
            }
        }
        
        usleep(1000); // 1ms
    }
    
    return false;
}

std::vector<uint8_t> MSPClient::buildFrame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(6 + payload.size());
    
    // Preamble
    frame.push_back('$');
    frame.push_back('M');
    frame.push_back('<');
    
    // Size and command
    uint8_t size = payload.size();
    frame.push_back(size);
    frame.push_back(cmd);
    
    // Payload
    frame.insert(frame.end(), payload.begin(), payload.end());
    
    // Checksum
    uint8_t checksum = size ^ cmd;
    for (uint8_t b : payload) {
        checksum ^= b;
    }
    frame.push_back(checksum);
    
    return frame;
}

bool MSPClient::readFrame(uint8_t& direction, uint8_t& cmd, std::vector<uint8_t>& payload) {
    payload.clear();
    
    uint8_t b;
    
    // Read '$'
    if (!readByte(b, 0.01) || b != '$') {
        return false;
    }
    
    // Read 'M'
    if (!readByte(b, 0.01) || b != 'M') {
        return false;
    }
    
    // Read direction
    if (!readByte(direction, 0.01)) {
        return false;
    }
    if (direction != '>' && direction != '!') {
        return false;
    }
    
    // Read size
    uint8_t size;
    if (!readByte(size, 0.01)) {
        return false;
    }
    
    // Read command
    if (!readByte(cmd, 0.01)) {
        return false;
    }
    
    // Read payload
    payload.resize(size);
    for (uint8_t i = 0; i < size; i++) {
        if (!readByte(payload[i], 0.01)) {
            return false;
        }
    }
    
    // Read checksum
    uint8_t checksum_recv;
    if (!readByte(checksum_recv, 0.01)) {
        return false;
    }
    
    // Verify checksum
    uint8_t checksum_calc = size ^ cmd;
    for (uint8_t b : payload) {
        checksum_calc ^= b;
    }
    
    if (checksum_recv != checksum_calc) {
        std::cerr << "Checksum mismatch: expected 0x" << std::hex << (int)checksum_calc
                  << " got 0x" << (int)checksum_recv << std::dec << std::endl;
        return false;
    }
    
    return true;
}

bool MSPClient::readByte(uint8_t& byte, double timeout_sec) {
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(timeout_sec);
    
    while (std::chrono::steady_clock::now() - start < timeout) {
        int bytes_available;
        if (ioctl(fd_, FIONREAD, &bytes_available) == 0 && bytes_available > 0) {
            ssize_t n = read(fd_, &byte, 1);
            if (n == 1) {
                return true;
            }
        }
        
        usleep(100); 
    }
    
    return false;
}

} 