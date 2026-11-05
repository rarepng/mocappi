#include "CameraDataProvider.h"

#include <fstream>
#include <sstream>
#include <unordered_map>

#include "src/util/network/CameraNetworkUtils.h"

using namespace boost::asio::ip;

CameraDataProvider::CameraDataProvider(DataModel& model)
    : Processor(model)
    , groundTruthFrameSet(MAX_FRAMESET_SIZE)
{
    timepointStart = std::chrono::steady_clock::now();

    receive_socket = new udp::socket(io_service, udp::endpoint(udp::v4(), HOST_LISTENING_PORT));
    broadcast_socket = new udp::socket(io_service, udp::endpoint(udp::v4(), 0));
    send_socket = new udp::socket(io_service, udp::endpoint(udp::v4(), 0));
    reply_socket = new udp::socket(io_service, udp::endpoint(udp::v4(), 0));

    is_running_ = true;

    // Receiving Thread:
    receive_thread = std::thread([this](){
        while (is_running_) {
            try {
                udp::endpoint sender_endpoint;
                std::vector<char> recv_buffer_(60000);
                size_t length = receive_socket->receive_from(boost::asio::buffer(recv_buffer_), sender_endpoint);

                interpretPacket(sender_endpoint, (uint8_t*)&recv_buffer_[0], length);
            } catch (std::exception& e) {
                if (is_running_) {
                    std::cerr << "Exception: " << e.what() << "\n";
                } else {
                    std::cout << "Receiver was stopped.\n";
                    break;
                }
            }
        }
    });

    // Broadcast Ping Thread:
    broadcast_ping_thread = std::thread([this](){
        try {
            // Socket und Endpunkt einrichten
            std::string broadcastAddr = CameraNetworkUtils::getBroadcastAddr();
            udp::endpoint broadcast_endpoint(boost::asio::ip::make_address(broadcastAddr), CAMERA_LISTENING_PORT);

            broadcast_socket->set_option(udp::socket::reuse_address(true));
            broadcast_socket->set_option(boost::asio::socket_base::broadcast(true));

            while (is_running_) {
                if(updateBroadcastAddrNextSend){
                    std::string broadcastAddr = CameraNetworkUtils::getBroadcastAddr();
                    broadcast_endpoint = udp::endpoint(boost::asio::ip::make_address(broadcastAddr), CAMERA_LISTENING_PORT);
                    updateBroadcastAddrNextSend = false;
                }

                uint8_t buffer[13];
                buffer[0] = 0;
                *((float*)(&buffer[1])) = requestedFramerate;
                *((double*)(&buffer[5])) = getHostTimestamp();
                broadcast_socket->send_to(boost::asio::buffer(buffer), broadcast_endpoint);

                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        } catch (std::exception& e) {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    });
}

void CameraDataProvider::reply(boost::asio::ip::udp::endpoint endpoint, const uint8_t* data, std::size_t length) {
    try {
        endpoint.port(CAMERA_LISTENING_PORT);
        // Send the data
        reply_socket->send_to(boost::asio::buffer(data, length), endpoint);
    } catch (std::exception& e) {
        std::cerr << "Exception in send_to: " << e.what() << "\n";
    }
}

double CameraDataProvider::getHostTimestamp(){
    return std::chrono::duration<long long, std::nano>{std::chrono::steady_clock::now() - timepointStart}.count() / 1000000000.0;
}

void CameraDataProvider::interpretPacket(udp::endpoint& sender_endpoint, uint8_t* recv_buffer, size_t length){
    if(length < 1)
        return;

    //std::cout << "Received Paket of Type " << int(recv_buffer[0]) << std::endl;

    if(recv_buffer[0] == 1){
        int uniqueCamID             = *((int*)&recv_buffer[1]);
        uint64_t camTimestamp       = *((uint64_t*)&recv_buffer[5]);
        float camPing               = *((float*)&recv_buffer[13]);
        bool sendRawImage           = recv_buffer[17] > 0;
        uint64_t exposureTime       = *((uint64_t*)&recv_buffer[18]);
        float lensPosition          = *((float*)&recv_buffer[26]);
        uint8_t brightnessThreshold = recv_buffer[30];
        float camFramerate          = *((float*)&recv_buffer[31]);
        int camFramedrops         = *((int*)&recv_buffer[35]);
        float syncError         = *((float*)&recv_buffer[39]);

        uniqueCamID = retrieveUniqueCameraID(uniqueCamID, sender_endpoint);

        CameraNetworkUtils::updateLastReceived(uniqueCamID, getHostTimestamp(), 1);

        // Insert new camera object into model when the ID was not seen yet:
        {
            model.semaphore.acquire();
            if(model.cameras.count(uniqueCamID) == 0)
                model.cameras.try_emplace(uniqueCamID);
            model.semaphore.release();
        }

        Camera& cam = model.cameras[uniqueCamID];

        cam.deviceState.uniqueCamID = uniqueCamID;
        cam.deviceState.deviceTimestamp = camTimestamp;

        if(abs(cam.deviceState.ping - camPing) < 1)
            cam.deviceState.ping = cam.deviceState.ping * 0.95f + camPing * 0.05f;
        else
            cam.deviceState.ping = camPing;

        cam.deviceState.sendRawImage = sendRawImage;
        cam.deviceState.exposureTime = exposureTime;
        cam.deviceState.lensPosition = lensPosition;
        cam.deviceState.brightnessThreshold = brightnessThreshold;
        cam.deviceState.measuredFramerate = camFramerate;
        cam.deviceState.frameDropsLastSecond = camFramedrops;

        if(abs(cam.deviceState.timeSyncError-syncError) < 1)
            cam.deviceState.timeSyncError = cam.deviceState.timeSyncError * 0.95f + abs(syncError) * 0.05f;
        else
            cam.deviceState.timeSyncError = syncError;

        uint8_t buffer[31];
        buffer[0] = 2;
        *((uint64_t*)&buffer[1]) = camTimestamp;
        *((double*)&buffer[9]) = getHostTimestamp();
        *((uint8_t*)&buffer[17]) = cam.deviceState.requestRawImage ? 1 : 0;
        *((uint8_t*)&buffer[18]) = uint8_t(cam.deviceState.requestedBrightnessThreshold);
        *((uint64_t*)&buffer[19]) = uint64_t(cam.deviceState.requestedExposureTime);
        *((float*)&buffer[27]) = float(cam.deviceState.requestLensPosition);

        reply(sender_endpoint, buffer, 31);
    } else if(recv_buffer[0] == 64){
        int uniqueCamID = 0;

        std::memcpy(&uniqueCamID, recv_buffer + 1, sizeof(uniqueCamID));
        uniqueCamID = retrieveUniqueCameraID(uniqueCamID, sender_endpoint);

        CameraFrame frame;
        deserializeFrame(frame, recv_buffer, length, uniqueCamID);
        storeFrame(frame, sender_endpoint);
    } else if(recv_buffer[0] == 65){
        deserializeAndStoreRawImage(recv_buffer, length, sender_endpoint);
    }
}


CameraDataProvider::~CameraDataProvider(){
    is_running_ = false;

    if(receive_socket->is_open())
        receive_socket->close();

    if(broadcast_socket->is_open())
        broadcast_socket->close();

    if(send_socket->is_open())
        send_socket->close();

    if(reply_socket->is_open())
        reply_socket->close();

    if (receive_thread.joinable())
        receive_thread.join();

    if (broadcast_ping_thread.joinable())
        broadcast_ping_thread.join();
}

void CameraDataProvider::deserializeFrame(CameraFrame& frame, const uint8_t* buffer, std::size_t bytes_recvd, int processedUniqueCamID){
    if (bytes_recvd < sizeof(int) * 2 + sizeof(double) + 1) {
        throw std::runtime_error("Received data is too small to contain required information.");
    }

    //std::cout << "Received " << bytes_recvd << " Bytes!" << std::endl;

    const uint8_t* ptr = buffer + 1;

    // Deserialize camId
    std::memcpy(&frame.uniqueCamID, ptr, sizeof(frame.uniqueCamID));
    ptr += sizeof(frame.uniqueCamID);

    // Overwrite with correct uniqueCamId (This is a bit cumbersome implemented
    // to avoid giving the sender_endpoint to this function):
    frame.uniqueCamID = processedUniqueCamID;

    // Deserialize timestamp
    std::memcpy(&frame.timestamp, ptr, sizeof(frame.timestamp));
    ptr += sizeof(frame.timestamp);

    // Deserialize num of vectors
    int numVectors;
    std::memcpy(&numVectors, ptr, sizeof(numVectors));
    ptr += sizeof(numVectors);

    //std::cout << "Received " << numVectors << " Markers!" << std::endl;
    frame.markers.resize(numVectors);

    std::size_t expectedSize = sizeof(int) * 2 + sizeof(double) + sizeof(Vec4f) * numVectors + 1;
    if (bytes_recvd != expectedSize) {
        throw std::runtime_error("Received data size does not match expected size.");
    }

    // Deserialize vectors
    for (int i = 0; i < numVectors; ++i) {
        Vec4f vec;
        std::memcpy(&vec, ptr, sizeof(Vec4f));

        float x = vec.x / vec.w;
        float y = vec.y / vec.w;
        float z = vec.z;
        float w = vec.w;

        frame.markers[i].x = x;
        frame.markers[i].y = y;
        frame.markers[i].pixels = z;
        frame.markers[i].intensity = w;

        ptr += sizeof(Vec4f);
    }

    // Filter markers that are in masked areas:
    if(cameraMasks.count(frame.uniqueCamID) > 0){
        frame.filterByCameraMask(cameraMasks[frame.uniqueCamID]);
    }

    if(shouldRecordAMask){
        addFrameToCameraMask(frame);
    }

    // Last Received:
    CameraNetworkUtils::updateLastReceived(frame.uniqueCamID, getHostTimestamp(), 64);

    frame.valid = true;
}

int CameraDataProvider::retrieveUniqueCameraID(int cameraID, boost::asio::ip::udp::endpoint endpoint){
    if(cameraID > 0)
        return cameraID;

    auto ip = endpoint.address();
    if (ip.is_v4()) {
        auto bytes = ip.to_v4().to_bytes();
        int lastIPBlock = static_cast<int>(bytes[3]);
        return lastIPBlock;
    }

    return -1;
}

void CameraDataProvider::storeFrame(CameraFrame& frame, udp::endpoint& sender_endpoint){
    if(frame.uniqueCamID == 999){
        groundTruthFrameSet.semaphore.acquire();
        groundTruthFrameSet.insertFrame(frame);
        groundTruthFrameSet.semaphore.release();
    } else {
        // Set correct unique camera id (in case of real camera it's the fourth IPv4 block):
        frame.uniqueCamID = retrieveUniqueCameraID(frame.uniqueCamID, sender_endpoint);

        // Insert new camera object into model when the ID was not seen yet:
        {
            model.semaphore.acquire();
            if(model.cameras.count(frame.uniqueCamID) == 0)
                model.cameras.try_emplace(frame.uniqueCamID);
            model.semaphore.release();
        }

        // Get correct camera struct:
        Camera& camera = model.cameras[frame.uniqueCamID];

        camera.buffer.semaphore.acquire();
        // Insert frame into correct camera struct:
        camera.buffer.insertFrame(frame);
        camera.buffer.semaphore.release();

        double captureToAvailableDelay = getHostTimestamp() - frame.timestamp;
        if(abs(captureToAvailableDelay - camera.deviceState.captureToAvailableDelay) < 1)
            camera.deviceState.captureToAvailableDelay = camera.deviceState.captureToAvailableDelay * 0.95f + (captureToAvailableDelay) * 0.05f;
        else
            camera.deviceState.captureToAvailableDelay = captureToAvailableDelay;
    }

    if(isRecording)
        recordedData.push_back(frame);
}

std::vector<uint8_t> CameraDataProvider::decodeImage(const uint8_t* encoded, unsigned int size) {
    std::vector<uint8_t> decoded;

    for (int i=0; i < size; ++i) {
        uint8_t byte = encoded[i];
        uint8_t pixelValue = (byte >> 4) * 17;
        uint8_t runLength = byte & 0x0F;

        // Insert every pixel:
        for (unsigned int h = 0; h < runLength; ++h) {
            decoded.push_back(pixelValue);
            //std::cout << int(pixelValue);
        }
    }

    return decoded;
}

void CameraDataProvider::deserializeAndStoreRawImage(const uint8_t* buffer, std::size_t bytes_recvd, udp::endpoint& sender_endpoint){
    if(bytes_recvd >= 31){
        int uniqueCamID             = retrieveUniqueCameraID(*((int*)&buffer[1]), sender_endpoint);
        uint32_t width              = *((uint32_t*)&buffer[5]);
        uint32_t height             = *((uint32_t*)&buffer[9]);
        uint32_t reduce             = *((uint32_t*)&buffer[13]);
        uint32_t encodedSize        = *((uint32_t*)&buffer[17]);

        if(width == 1280 && height == 720 && reduce == 4){
            std::vector<uint8_t> decodedImage = decodeImage(&buffer[21], bytes_recvd - 17);

            if(model.cameras.count(uniqueCamID) > 0){
                model.cameras[uniqueCamID].rawImage = decodedImage;
                model.cameras[uniqueCamID].lastRawImageReceived = getHostTimestamp();
            }
        }
    }
}



std::string generateFilename(int camID) {
    std::ostringstream oss;
    oss << "camera_" << camID << ".txt";
    return oss.str();
}


void CameraDataProvider::writeFrames() {
    isRecording = false;

    std::unordered_map<int, std::ofstream> fileStreams;

    for (const auto& frame : recordedData) {
        if (frame.valid) {
            int camID = frame.uniqueCamID;
            if (fileStreams.find(camID) == fileStreams.end()) {
                fileStreams[camID].open(generateFilename(camID));
                fileStreams[camID] << "Cam Recording" << "\n";
            }
            fileStreams[camID] << (frame.timestamp - recordedData[0].timestamp);
            for(const auto& m : frame.markers) {
                fileStreams[camID] << ",[" << (m.x * m.intensity) << "," << (m.y * m.intensity) << "," << m.pixels << "," << m.intensity << "]";
            }
            fileStreams[camID] << "\n";

            std::cout << "Wrote data to file streams!" << std::endl;
        }
    }
    
    // Close all file streams
    for (auto& entry : fileStreams) {
            entry.second.close();
    }       
}

void CameraDataProvider::process(TickData& tick){
    model.semaphore.acquire();

    for(auto& camera : model.cameras){
        camera.second.buffer.semaphore.acquire();

        CameraFrame frame = camera.second.buffer.getInterpFrameAt(tick.timestamp);
        frame.uniqueCamID = camera.first;

        if(!frame.valid)
            frame.timestamp = tick.timestamp;

        tick.frames.push_back(std::move(frame));
        camera.second.buffer.semaphore.release();
    }

    groundTruthFrameSet.semaphore.acquire();
    tick.markersGTPos = groundTruthFrameSet.getGroundTruthAt(tick.timestamp);
    groundTruthFrameSet.semaphore.release();

    model.semaphore.release();
}
