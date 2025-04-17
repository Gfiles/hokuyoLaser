#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>
#include <map>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <nlohmann/json.hpp>

#ifdef _WIN32
#include <windows.h>
#else
#include <X11/Xlib.h>
#include <X11/extensions/XTest.h>
#endif

using json = nlohmann::json;
using namespace boost::asio;
using ip::tcp;
using ip::udp;

struct Zone {
    double angle;
    double distance;
    double radius;
    std::string in_event;
    std::string on_event;
    std::string out_event;
    std::string name;
};

class HokuyoZoneServer {
public:
    HokuyoZoneServer(const std::string& config_path)
        : io_(), serial_(io_), tcp_acceptor_(io_), udp_socket_(io_), running_(false) {
        loadConfig(config_path);
    }

    void run() {
        if (!openSerialPort()) {
            std::cerr << "Failed to open serial port\n";
            return;
        }
        if (outputType == "TCP") {
            startTCPServer();
        }
        else if (outputType == "OSC") {
            startOSCClient();
        }
        running_ = true;
        scanThread_ = std::thread(&HokuyoZoneServer::scanLoop, this);
        io_.run();
        scanThread_.join();
    }

    ~HokuyoZoneServer() {
        running_ = false;
        if (scanThread_.joinable()) scanThread_.join();
        closeSerialPort();
    }

private:
    // Configuration parameters
    std::string uartPort;
    unsigned int uartSpeed;
    double minDist, maxDist;
    double minAng, maxAng;
    double touchWidth, touchHeight;
    double widthOffset, heightOffset;
    double angleOffset;
    double time2Scan;
    int sendSpeed;
    int minSize, maxSize;
    int tcpPort;
    int oscPort;
    std::string oscServer;
    std::string oscAddress;
    std::string outputType;
    bool debug;

    std::vector<Zone> manual_zones;

    io_service io_;
    serial_port serial_;
    tcp::acceptor tcp_acceptor_;
    std::vector<std::shared_ptr<tcp::socket>> tcp_clients_;
    std::mutex clients_mutex_;

    udp::socket udp_socket_;
    udp::endpoint osc_endpoint_;

    std::thread scanThread_;
    std::atomic<bool> running_;

    // Platform-specific display for Linux keyboard simulation
#ifndef _WIN32
    Display* display_ = nullptr;
#endif

    void loadConfig(const std::string& config_path) {
        std::ifstream ifs(config_path);
        if (!ifs) {
            std::cerr << "Config file not found: " << config_path << "\n";
            throw std::runtime_error("Config file not found");
        }
        json config;
        ifs >> config;

        uartPort = config.value("uartPort", "COM4");
        uartSpeed = config.value("uartSpeed", 19200);
        minDist = config.value("minDist", 100.0);
        maxDist = config.value("maxDist", 4000.0);
        minAng = config.value("minAng", -90.0);
        maxAng = config.value("maxAng", 90.0);
        touchWidth = config.value("touchWidth", 4000.0);
        touchHeight = config.value("touchHeight", 2250.0);
        widthOffset = config.value("widthOffset", 0.0);
        heightOffset = config.value("heightOffset", 0.0);
        angleOffset = config.value("angleOffset", 0.0);
        time2Scan = config.value("time2Scan", 0.1);
        sendSpeed = config.value("sendSpeed", 1);
        minSize = config.value("minSize", 20);
        maxSize = config.value("maxSize", 300);
        tcpPort = config.value("tcpPort", 65432);
        oscPort = config.value("oscPort", 9000);
        oscServer = config.value("oscServer", "192.168.60.159");
        oscAddress = config.value("oscAddress", "/zones");
        outputType = config.value("outputType", "Zones");
        debug = config.value("debug", false);

        if (config.contains("manual_zones")) {
            for (auto& z : config["manual_zones"]) {
                Zone zone;
                zone.name = z.value("name", "");
                zone.angle = z.value("angle", 0.0);
                zone.distance = z.value("distance", 0.0);
                zone.radius = z.value("radius", 0.0);
                zone.in_event = z.value("in_event", "");
                zone.on_event = z.value("on_event", "");
                zone.out_event = z.value("out_event", "");
                manual_zones.push_back(zone);
            }
        }
    }

    bool openSerialPort() {
        boost::system::error_code ec;
        serial_.open(uartPort, ec);
        if (ec) {
            std::cerr << "Error opening serial port: " << ec.message() << "\n";
            return false;
        }
        serial_.set_option(serial_port_base::baud_rate(uartSpeed));
        serial_.set_option(serial_port_base::character_size(8));
        serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
        serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
        return true;
    }

    void closeSerialPort() {
        if (serial_.is_open()) {
            serial_.close();
        }
    }

    void startTCPServer() {
        tcp_acceptor_ = tcp::acceptor(io_, tcp::endpoint(tcp::v4(), tcpPort));
        acceptTCPClients();
    }

    void acceptTCPClients() {
        auto socket = std::make_shared<tcp::socket>(io_);
        tcp_acceptor_.async_accept(*socket, [this, socket](const boost::system::error_code& ec) {
            if (!ec) {
                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    tcp_clients_.push_back(socket);
                }
                if (debug) std::cout << "TCP client connected\n";
                startRead(socket);
            }
            acceptTCPClients();
        });
    }

    void startRead(std::shared_ptr<tcp::socket> socket) {
        auto buffer = std::make_shared<std::vector<char>>(1024);
        socket->async_read_some(boost::asio::buffer(*buffer), [this, socket, buffer](const boost::system::error_code& ec, std::size_t length) {
            if (!ec) {
                // Echo or handle client data if needed
                startRead(socket);
            }
            else {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                tcp_clients_.erase(std::remove(tcp_clients_.begin(), tcp_clients_.end(), socket), tcp_clients_.end());
                if (debug) std::cout << "TCP client disconnected\n";
            }
        });
    }

    void sendTCPMessage(const std::string& msg) {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (auto& client : tcp_clients_) {
            boost::system::error_code ec;
            boost::asio::write(*client, boost::asio::buffer(msg), ec);
            if (ec) {
                if (debug) std::cerr << "Error sending to TCP client: " << ec.message() << "\n";
            }
        }
    }

    void startOSCClient() {
        udp::endpoint local_endpoint(udp::v4(), 0);
        udp_socket_.open(udp::v4());
        osc_endpoint_ = udp::endpoint(ip::address::from_string(oscServer), oscPort);
    }

    void sendOSCMessage(const std::string& msg) {
        // Minimal OSC message: address + string argument
        // OSC message format is binary; here we build a simple message
        std::vector<char> buffer;
        // OSC address padded to 4 bytes
        std::string address = oscAddress;
        if (address.back() != '\0') address.push_back('\0');
        while (address.size() % 4 != 0) address.push_back('\0');
        buffer.insert(buffer.end(), address.begin(), address.end());

        // Type tag string: ,s
        std::string type_tag = ",s";
        if (type_tag.back() != '\0') type_tag.push_back('\0');
        while (type_tag.size() % 4 != 0) type_tag.push_back('\0');
        buffer.insert(buffer.end(), type_tag.begin(), type_tag.end());

        // String argument padded to 4 bytes
        std::string argument = msg;
        if (argument.back() != '\0') argument.push_back('\0');
        while (argument.size() % 4 != 0) argument.push_back('\0');
        buffer.insert(buffer.end(), argument.begin(), argument.end());

        udp_socket_.send_to(boost::asio::buffer(buffer), osc_endpoint_);
    }

    void simulateKeypress(const std::string& msg) {
#ifdef _WIN32
        for (char c : msg) {
            SHORT vk = VkKeyScanA(c);
            if (vk == -1) continue;
            INPUT input[2] = {};
            input[0].type = INPUT_KEYBOARD;
            input[0].ki.wVk = LOBYTE(vk);
            input[0].ki.dwFlags = 0;
            input[1].type = INPUT_KEYBOARD;
            input[1].ki.wVk = LOBYTE(vk);
            input[1].ki.dwFlags = KEYEVENTF_KEYUP;
            SendInput(2, input, sizeof(INPUT));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
#else
        if (!display_) {
            display_ = XOpenDisplay(nullptr);
            if (!display_) {
                std::cerr << "Failed to open X display for key simulation\n";
                return;
            }
        }
        for (char c : msg) {
            KeySym keysym = XStringToKeysym(std::string(1, c).c_str());
            if (keysym == NoSymbol) continue;
            KeyCode keycode = XKeysymToKeycode(display_, keysym);
            if (keycode == 0) continue;
            XTestFakeKeyEvent(display_, keycode, True, 0);
            XTestFakeKeyEvent(display_, keycode, False, 0);
            XFlush(display_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
#endif
    }

    double distBetweenPolar(double r1, double theta1, double r2, double theta2) {
        double theta1_rad = theta1 * M_PI / 180.0;
        double theta2_rad = theta2 * M_PI / 180.0;
        double x1 = r1 * cos(theta1_rad);
        double y1 = r1 * sin(theta1_rad);
        double x2 = r2 * cos(theta2_rad);
        double y2 = r2 * sin(theta2_rad);
        return std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
    }

    void scanLoop() {
        while (running_) {
            // TODO: Implement laser scanning and parsing data from serial port
            // For now, simulate scan data or read from serial port

            // Simulated scan data: map angle to distance
            std::map<double, double> scan;
            // TODO: Replace with actual laser scan data parsing

            // Process zones
            for (const auto& zone : manual_zones) {
                for (const auto& [angle, dist] : scan) {
                    if (distBetweenPolar(dist, angle, zone.distance, zone.angle) < zone.radius) {
                        if (debug) {
                            std::cout << zone.name << " - " << zone.on_event << std::endl;
                        }
                        std::string msg = zone.on_event;
                        if (outputType == "OSC") {
                            sendOSCMessage(msg);
                        }
                        else if (outputType == "TCP") {
                            sendTCPMessage(msg);
                        }
                        else if (outputType == "Keyboard") {
                            simulateKeypress(msg);
                        }
                        break;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(time2Scan));
        }
    }
};

int main(int argc, char* argv[]) {
    try {
        std::string config_path = "appconfig.json";
        if (argc > 1) {
            config_path = argv[1];
        }
        HokuyoZoneServer server(config_path);
        server.run();
    }
    catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}
