#ifndef UAV_MSP_BRIDGE_MSP_CLIENT_HPP
#define UAV_MSP_BRIDGE_MSP_CLIENT_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <fcntl.h>
#include <termios.h>

namespace uav_msp_bridge {

class MSPClient {
public:
    /**
     * Constructor
     * @param port Serial port path (e.g., "/dev/ttyACM0")
     * @param baud Baud rate (e.g., 115200)
     */
    MSPClient(const std::string& port, int baud);
    
    /**
     * Destructor - closes serial port if open
     */
    ~MSPClient();
    
    /**
     * Open serial connection
     * @return true if successful
     */
    bool open();
    
    /**
     * Close serial connection
     */
    void close();
    
    /**
     * Check if serial port is open
     * @return true if open
     */
    bool isOpen() const;
    
    /**
     * Send MSP request and wait for response
     * @param cmd MSP command ID
     * @param payload Command payload (empty for most read commands)
     * @param response Output vector for response payload
     * @param timeout_sec Timeout in seconds
     * @return true if response received successfully
     */
    bool request(uint8_t cmd, const std::vector<uint8_t>& payload,
                 std::vector<uint8_t>& response, double timeout_sec = 0.5);

private:
    /**
     * Build MSP v1 frame
     */
    std::vector<uint8_t> buildFrame(uint8_t cmd, const std::vector<uint8_t>& payload);
    
    /**
     * Read one MSP frame from serial port
     * @param direction Output: '>' for response, '!' for error
     * @param cmd Output: command ID
     * @param payload Output: payload bytes
     * @return true if valid frame read successfully
     */
    bool readFrame(uint8_t& direction, uint8_t& cmd, std::vector<uint8_t>& payload);
    
    /**
     * Read single byte with timeout
     */
    bool readByte(uint8_t& byte, double timeout_sec);
    
    std::string port_;
    int baud_;
    int fd_;
};

} // namespace uav_msp_bridge

#endif // UAV_MSP_BRIDGE_MSP_CLIENT_HPP