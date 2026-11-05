/**
 * =====================================================================
 * DummyMarkerSender
 * =====================================================================
 * This program sends mocap recordings to the Mocap-Pi Main Application.
 *
 * To specify the recording, use one of the following options:
 * './app --f [Folder To Recordings]'
 * './app --folder [Folder To Recordings]'
 *
 * If no folder is set, the behavior is undefined.
 *
 * Author: Andre Muehlenbrock, code cleaned/improved using ChatGPT.
 */

#include <boost/asio.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <tuple>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#define HOST_LISTENING_PORT 4578
#define CAMERA_LISTENING_PORT 4579

using boost::asio::ip::udp;
using namespace std::chrono;

struct Vec4f {
    float x;
    float y;
    float z;
    float w;
};

struct MarkerPaket {
    int camId = 0;
    double timestamp = 0.0;
    std::vector<Vec4f> vectors;
};

double getCurrentCamSideTime(const steady_clock::time_point& startTime){
    return duration<double>(steady_clock::now() - startTime).count();
}

class UDPPacketSender {
public:
    UDPPacketSender(boost::asio::io_context& ioContext, const std::string& host, const std::string& port, steady_clock::time_point startTime)
        : ioContext_(ioContext)
        , socket_(ioContext, udp::endpoint(udp::v4(), 0))
        , receiveSocket_(ioContext, udp::endpoint(udp::v4(), CAMERA_LISTENING_PORT))
        , startTime_(startTime)
    {
        udp::resolver resolver(ioContext_);
        const auto endpoints = resolver.resolve(host, port);

        if (endpoints.empty()) {
            throw std::runtime_error("No UDP target found for " + host + ":" + port + ".");
        }

        receiverEndpoint_ = endpoints.begin()->endpoint();

        std::cout << "UDP target: " << receiverEndpoint_.address().to_string() << ":" << receiverEndpoint_.port() << std::endl;

        isRunning_ = true;
        startReceive();
    }

    ~UDPPacketSender(){
        stop();
    }

    void startReceive(){
        receiveSocket_.async_receive_from(
            boost::asio::buffer(receiveBuffer_), senderEndpoint_,
            [this](const boost::system::error_code& error, std::size_t length) {
                if (!error) {
                    if (length >= 5 + sizeof(double) && static_cast<std::uint8_t>(receiveBuffer_[0]) == 0){
                        double receivedHostTimestamp = 0.0;
                        std::memcpy(&receivedHostTimestamp, receiveBuffer_.data() + 5, sizeof(receivedHostTimestamp));

                        std::lock_guard<std::mutex> lock(syncMutex_);
                        camSideTimestampOnSync_ = getCurrentCamSideTime(startTime_);
                        hostTimestampOnSync_ = receivedHostTimestamp;
                    }
                }
                
                if (isRunning_) {
                    startReceive();
                }
            });
    }

    void stop(){
        if (!isRunning_.exchange(false)) return;
        boost::asio::post(ioContext_, [this]() {
            boost::system::error_code ignoredError;
            std::ignore = receiveSocket_.cancel(ignoredError);
            std::ignore = receiveSocket_.close(ignoredError);
            std::ignore = socket_.close(ignoredError);
        });
    }

    void resetStartTime(steady_clock::time_point startTime){
        std::lock_guard<std::mutex> lock(syncMutex_);
        camSideTimestampOnSync_ -= duration<double>(startTime - startTime_).count();
        startTime_ = startTime;
    }

    void send(MarkerPaket data){
        data.timestamp = convertToHostTimestamp(data.timestamp);

        auto buffer = std::make_shared<std::vector<char>>();
        serializeData(data, *buffer);

        boost::asio::post(ioContext_, [this, buffer]() {
            socket_.async_send_to(
                boost::asio::buffer(*buffer),
                receiverEndpoint_,
                [buffer](const boost::system::error_code& error, std::size_t /*bytesSent*/) {
                    if (error && error != boost::asio::error::operation_aborted)
                        std::cerr << "UDP send error: " << error.message() << " (" << error.value() << ")" << std::endl;
                });
        });
    }

private:
    double convertToHostTimestamp(double camSideTimestamp)
    {
        std::lock_guard<std::mutex> lock(syncMutex_);
        return (camSideTimestamp - camSideTimestampOnSync_) + hostTimestampOnSync_;
    }

    void receiveLoop(){
        isRunning_ = true;

        while (isRunning_) {
            try {
                udp::endpoint senderEndpoint;
                std::array<char, 60000> receiveBuffer;

                const std::size_t length = receiveSocket_.receive_from(boost::asio::buffer(receiveBuffer), senderEndpoint);

                if (length >= 5 + sizeof(double) && static_cast<std::uint8_t>(receiveBuffer[0]) == 0){
                    double receivedHostTimestamp = 0.0;

                    std::memcpy(&receivedHostTimestamp, receiveBuffer.data() + 5, sizeof(receivedHostTimestamp));

                    std::lock_guard<std::mutex> lock(syncMutex_);

                    camSideTimestampOnSync_ = getCurrentCamSideTime(startTime_);

                    hostTimestampOnSync_ = receivedHostTimestamp;
                }
            } catch (const std::exception& error) {
                std::cerr << "Error: " << error.what() << std::endl;
            }
        }
    }

    static void serializeData(const MarkerPaket& data, std::vector<char>& buffer){
        const int numVectors = static_cast<int>(std::min<std::size_t>(data.vectors.size(), 200));

        const std::size_t bufferSize =
            + sizeof(data.camId)
            + sizeof(data.timestamp)
            + sizeof(numVectors)
            + std::size_t(numVectors) * sizeof(Vec4f) + 1;

        buffer.resize(bufferSize);
        char* ptr = buffer.data();

        const std::uint8_t packetType = 64;
        std::memcpy(ptr, &packetType, sizeof(packetType));
        ptr += sizeof(packetType);

        std::memcpy(ptr, &data.camId, sizeof(data.camId));
        ptr += sizeof(data.camId);

        std::memcpy(ptr, &data.timestamp, sizeof(data.timestamp));
        ptr += sizeof(data.timestamp);

        std::memcpy(ptr, &numVectors, sizeof(numVectors));
        ptr += sizeof(numVectors);

        for (int i = 0; i < numVectors; ++i) {
            std::memcpy(ptr, &data.vectors[i], sizeof(Vec4f));
            ptr += sizeof(Vec4f);
        }
    }

    boost::asio::io_context& ioContext_;
    udp::socket socket_;
    udp::endpoint receiverEndpoint_;
    udp::socket receiveSocket_;

    udp::endpoint senderEndpoint_;
    std::array<char, 60000> receiveBuffer_{};
    std::atomic_bool isRunning_ = false;

    std::mutex syncMutex_;
    double hostTimestampOnSync_ = 0.0;
    double camSideTimestampOnSync_ = 0.0;
    steady_clock::time_point startTime_;
};

std::vector<MarkerPaket> parseFile(const std::string& fileName, int camId){
    std::vector<MarkerPaket> markerPakete;
    std::ifstream file(fileName);

    if (!file.is_open()) {
        std::cerr << "File could not be opened: " << fileName << std::endl;
        return markerPakete;
    }

    std::string line;

    while (std::getline(file, line)) {
        std::istringstream lineStream(line);

        MarkerPaket paket;
        paket.camId = camId;

        if (!(lineStream >> paket.timestamp))
            continue;

        lineStream >> std::ws;

        if (lineStream.peek() == ',')
            lineStream.get();

        while (lineStream) {
            lineStream >> std::ws;

            if (lineStream.peek() != '[')
                break;

            lineStream.get(); // '['

            Vec4f vec;
            char separator = '\0';

            if (!(lineStream >> vec.x))
                break;

            lineStream >> std::ws >> separator; // ','
            if (!(lineStream >> vec.y))
                break;

            lineStream >> std::ws >> separator; // ','
            if (!(lineStream >> vec.z))
                break;

            lineStream >> std::ws >> separator; // ','
            if (!(lineStream >> vec.w))
                break;

            lineStream >> std::ws;

            if (lineStream.peek() == ']')
                lineStream.get();

            paket.vectors.push_back(vec);

            lineStream >> std::ws;

            if (lineStream.peek() == ',')
                lineStream.get();
        }

        markerPakete.push_back(std::move(paket));
    }

    return markerPakete;
}

void normalizePacketTimestamps(std::vector<MarkerPaket>& packets, double sharedZero, const std::string& streamName){
    if (packets.empty()) {
        return;
    }

    for (MarkerPaket& packet : packets) {
        packet.timestamp -= sharedZero;
    }

    std::cout << "\t" << streamName << ": first packet at +" << packets.front().timestamp * 1000.0 << " ms on the shared time base." << std::endl;
}

namespace fs = std::filesystem;

struct CameraInput {
    int cameraId = 0;
    fs::path filePath;
    std::vector<MarkerPaket> packets;
    std::size_t currentPacketId = 0;
};

std::optional<int> parseNumericCameraId(
    const fs::path& filePath,
    const std::string& expectedPrefix)
{
    if (filePath.extension() != ".txt")
        return std::nullopt;

    const std::string stem = filePath.stem().string();

    if (stem.rfind(expectedPrefix, 0) != 0)
        return std::nullopt;

    const std::string idText = stem.substr(expectedPrefix.size());
    if (idText.empty() || !std::all_of(idText.begin(), idText.end(), [](unsigned char c) {return std::isdigit(c) != 0;})){
        return std::nullopt;
    }

    try {
        return std::stoi(idText);
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

std::optional<int> getPrefixedCameraId(const fs::path& filePath){
    return parseNumericCameraId(filePath, "camera_");
}

std::optional<int> getLegacyCameraId(const fs::path& filePath){
    return parseNumericCameraId(filePath, "");
}

using CameraIdParser = std::optional<int> (*)(const fs::path&);

std::vector<CameraInput> collectCameraFiles(const fs::path& folderPath, CameraIdParser idParser){
    std::vector<CameraInput> cameras;

    for (const fs::directory_entry& entry : fs::directory_iterator(folderPath)) {
        if (!entry.is_regular_file())
            continue;

        const std::optional<int> cameraId = idParser(entry.path());
        if (!cameraId.has_value())
            continue;

        cameras.push_back(CameraInput{*cameraId, entry.path(), {}, 0});
    }

    return cameras;
}

void sortAndValidateCameraFiles(std::vector<CameraInput>& cameras){
    std::sort(cameras.begin(), cameras.end(), [](const CameraInput& lhs, const CameraInput& rhs) {
            return lhs.cameraId < rhs.cameraId;
    });
}

std::vector<CameraInput> findCameraFiles(const fs::path& folderPath)
{
    if (!fs::exists(folderPath) || !fs::is_directory(folderPath))
        throw std::runtime_error("Folder not found: " + folderPath.string());

    std::vector<CameraInput> cameras = collectCameraFiles(folderPath, getPrefixedCameraId);

    if (!cameras.empty()) {
        std::cout << "Found Camera files in format camera_<ID>.txt, using these." << std::endl;
    } else {
        std::cout << "Search for <ID>.txt." << std::endl;

        cameras = collectCameraFiles(folderPath, getLegacyCameraId);

        if (!cameras.empty())
            std::cout << "Use cameras in format <ID>.txt." << std::endl;
    }

    sortAndValidateCameraFiles(cameras);
    return cameras;
}

int main(int argc, char* argv[])
{
    // Default path, is overwritten by '--folder' argument if set:
    fs::path folderPath = fs::path(CMAKE_SOURCE_DIR) / "pub" / "synthetic-recordings" / "tool0";

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-f" || arg == "--folder") {
            if (i + 1 >= argc) {
                std::cerr << "--folder requires an argument." << std::endl;
                return 1;
            }

            folderPath = argv[++i];
        }
    }

    std::cout << "Read files from path: " << folderPath.string() << std::endl;
    std::vector<CameraInput> cameras;

    try {
        cameras = findCameraFiles(folderPath);
    } catch (const std::exception& error) {
        std::cerr << "Error searching for camera files in given folder: " << error.what() << std::endl;
        return 1;
    }

    if (cameras.empty())
        std::cerr << "No camera files found. They should be formatted like 'camera_[ID].txt' or '[ID].txt." << std::endl;

    for (CameraInput& camera : cameras) {
        camera.packets = parseFile(camera.filePath.string(), camera.cameraId);

        for (std::size_t i = 1; i < camera.packets.size(); ++i) {
            const double difference = camera.packets[i].timestamp - camera.packets[i - 1].timestamp;

            if (difference < -0.1 || difference > 1.0) {
                std::cerr << "Strange frame in camera "
                          << camera.cameraId << ": "
                          << camera.packets[i - 1].timestamp << " -> "
                          << camera.packets[i].timestamp << std::endl;
            }
        }

        std::sort(camera.packets.begin(),  camera.packets.end(), [](const MarkerPaket& a, const MarkerPaket& b) {
                return a.timestamp < b.timestamp;
        });


        std::cout << "\tRead " << camera.packets.size()
                  << " frames for camera " << camera.cameraId
                  << " from " << camera.filePath.filename().string()
                   << std::endl;
    }

    std::vector<MarkerPaket> groundTruthPakets;
    std::size_t groundTruthPacketID = 0;

    const fs::path groundTruthFilePath =
        folderPath / "GroundTruth3D.txt";

    if (fs::is_regular_file(groundTruthFilePath)) {
        groundTruthPakets = parseFile(groundTruthFilePath.string(), 999);

        std::sort(groundTruthPakets.begin(), groundTruthPakets.end(), [](const MarkerPaket& a, const MarkerPaket& b) {
                return a.timestamp < b.timestamp;
        });

        std::cout << "\tRead " << groundTruthPakets.size() << " ground-truth frames." << std::endl;
    }

    double sharedZero = std::numeric_limits<double>::infinity();
    for (const CameraInput& camera : cameras) {
        if (!camera.packets.empty())
            sharedZero = std::min(sharedZero, camera.packets.front().timestamp);
    }
    if (!groundTruthPakets.empty())
        sharedZero = std::min(sharedZero, groundTruthPakets.front().timestamp);

    if (sharedZero != std::numeric_limits<double>::infinity()) {
        std::cout << "Shared time base: source timestamp " << sharedZero << " s becomes 0.0 s." << std::endl;
        for (CameraInput& camera : cameras)
            normalizePacketTimestamps(camera.packets, sharedZero, "Camera " + std::to_string(camera.cameraId));
        normalizePacketTimestamps(groundTruthPakets, sharedZero, "Ground Truth");
    }

    const bool anyCameraData = std::any_of(cameras.begin(), cameras.end(), [](const CameraInput& camera) {
            return !camera.packets.empty();
        });

    if (!anyCameraData && groundTruthPakets.empty()) {
        std::cerr << "No input data found" << std::endl;
        return 1;
    }

    std::atomic_bool shouldStop{false};
    auto startTime = steady_clock::now();

    try {
        boost::asio::io_context ioContext;

        auto workGuard = boost::asio::make_work_guard(ioContext);

        UDPPacketSender sender(ioContext, "127.0.0.1", std::to_string(HOST_LISTENING_PORT), startTime);

        boost::asio::signal_set signals(ioContext, SIGINT, SIGTERM);

        signals.async_wait([&shouldStop](const boost::system::error_code& error, int) {
            if (!error)
                shouldStop = true;
        });

        std::thread ioThread([&ioContext]() {
            try {
                ioContext.run();
            } catch (const std::exception& error) {
                std::cerr << "io_context-Fehler: " << error.what() << std::endl;
            }
        });

        constexpr auto cycleDuration = std::chrono::milliseconds(1);

        auto nextSendTime = steady_clock::now() + cycleDuration;

        while (!shouldStop) {
            bool packetsStillAvailable = false;

            const double currentCamSideTime = getCurrentCamSideTime(startTime);

            while (groundTruthPacketID < groundTruthPakets.size() && groundTruthPakets[groundTruthPacketID].timestamp <= currentCamSideTime) {
                sender.send(groundTruthPakets[groundTruthPacketID]);
                ++groundTruthPacketID;
            }

            if (groundTruthPacketID < groundTruthPakets.size())
                packetsStillAvailable = true;

            for (CameraInput& camera : cameras) {
                std::size_t packetsSentThisCycle = 0;

                while (camera.currentPacketId < camera.packets.size()
                       && camera.packets[camera.currentPacketId].timestamp
                              <= currentCamSideTime) {
                    const MarkerPaket& paket = camera.packets[camera.currentPacketId];

                    sender.send(paket);
                    ++camera.currentPacketId;
                    ++packetsSentThisCycle;
                }

                if (packetsSentThisCycle > 0) {
                    const MarkerPaket& lastPacket = camera.packets[camera.currentPacketId - 1];

                    // std::cout << "Cam " << camera.cameraId
                    //     << " Time: " << lastPacket.timestamp
                    //     << " (CamSideTime: " << currentCamSideTime
                    //     << ", sent: "
                    //     << packetsSentThisCycle
                    //           << ")" << std::endl;
                }

                if (camera.currentPacketId < camera.packets.size())
                    packetsStillAvailable = true;
            }

            if (!packetsStillAvailable) {
                groundTruthPacketID = 0;

                for (CameraInput& camera : cameras) {
                    camera.currentPacketId = 0;
                }

                startTime = steady_clock::now();
                sender.resetStartTime(startTime);
                nextSendTime = startTime + cycleDuration;
            }

            std::this_thread::sleep_until(nextSendTime);
            nextSendTime += cycleDuration;

            const auto now = steady_clock::now();

            if (nextSendTime + cycleDuration < now) {
                nextSendTime = now + cycleDuration;
            }
        }

        sender.stop();

        workGuard.reset();
        ioContext.stop();

        if (ioThread.joinable()) {
            ioThread.join();
        }
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << std::endl;
        return 1;
    }

    return 0;
}
