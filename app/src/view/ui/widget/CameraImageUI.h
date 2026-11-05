#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <imgui_internal.h>
#include <imgui.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cstdio>

#include "src/view/ViewData.h"

#include "src/model/DataModel.h"
#include "src/processors/ProcessorCollection.h"
#include "src/util/settings/Settings.h"

class CameraImageUI {
    struct CameraTexture {
        GLuint id = 0;
        int width = 0;
        int height = 0;
    };

    inline static std::map<int, CameraTexture> textures;

    static std::string toString(double d){
        std::ostringstream oss;
        oss << std::setprecision(2) << d;
        return oss.str();
    }

    static GLuint updateTexture(int camID, const std::vector<uint8_t>& image_data, int width, int height){
        CameraTexture& texture = textures[camID];

        if(texture.id == 0){
            glGenTextures(1, &texture.id);
            glBindTexture(GL_TEXTURE_2D, texture.id);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        } else {
            glBindTexture(GL_TEXTURE_2D, texture.id);
        }

        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        if(texture.width != width || texture.height != height){
            glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, image_data.data());
            texture.width = width;
            texture.height = height;
        } else {
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_UNSIGNED_BYTE, image_data.data());
        }

        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glBindTexture(GL_TEXTURE_2D, 0);

        return texture.id;
    }

public:
    // Nur bei gültigem OpenGL-Kontext aufrufen, bevor dieser zerstört wird.
    static void releaseTextures(){
        for(auto& [camID, texture] : textures){
            if(texture.id != 0)
                glDeleteTextures(1, &texture.id);
        }
        textures.clear();
    }

    static void renderCameraUI(int x, int y, int width, int height, DataModel& model, ProcessorCollection& processors, CameraFrame& frame){
        auto camIt = model.cameras.find(frame.uniqueCamID);
        if(camIt == model.cameras.end())
            return;

        constexpr float camWidth = 1280.0f;
        constexpr float camHeight = 720.0f;
        constexpr int rawWidth = 320;
        constexpr int rawHeight = 180;

        ImGui::SetNextWindowPos(ImVec2(float(x), float(y)));
        ImGui::SetNextWindowSize(ImVec2(float(width), float(height)));

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize;
        std::string windowName = "Camera " + std::to_string(frame.uniqueCamID);

        if(!ImGui::Begin(windowName.c_str(), nullptr, flags)){
            ImGui::End();
            return;
        }

        auto& camera = camIt->second;
        CameraDeviceState& camInfo = camera.deviceState;
        ImGuiStyle& style = ImGui::GetStyle();

        const bool showControls = Settings::instance.showCameraControls;
        const ImVec2 available = ImGui::GetContentRegionAvail();

        // Die Control-Zeile samt Abstand zum Bild reservieren.
        const float controlsHeight = showControls ? ImGui::GetFrameHeight() + style.ItemSpacing.y : 0.0f;
        const float imageWidth = ImMax(1.0f, available.x);
        const float imageHeight = ImMax(1.0f, available.y - controlsHeight - style.ItemSpacing.y);

        const ImVec2 imageSize(imageWidth, imageHeight);

        // Absolute Position des Bildes für Marker und Overlays.
        const ImVec2 imagePos = ImGui::GetCursorScreenPos();
        const std::vector<uint8_t>& rawImage = camera.rawImage;

        if(rawImage.size() >= rawWidth * rawHeight && camInfo.requestRawImage){
            GLuint textureID = updateTexture(frame.uniqueCamID, rawImage, rawWidth, rawHeight);
            ImGui::Image((ImTextureID)(intptr_t)textureID, imageSize);
        } else {
            ImGui::Dummy(imageSize);
        }

        ImDrawList* draw_list = ImGui::GetWindowDrawList();

        if(showControls){
            const float spacing = style.ItemSpacing.x;
            const float checkboxWidth = ImGui::GetFrameHeight();
            const float sliderWidth = ImMax(1.0f, (available.x - checkboxWidth - 3.0f * spacing) / 3.0f);

            ImGui::PushItemWidth(sliderWidth);

            ImGui::SliderInt("##Threshold", &camInfo.requestedBrightnessThreshold, 0, 255, "Trh: %d");
            ImGui::SameLine();

            ImGui::SliderInt("##Exposure", &camInfo.requestedExposureTime, 100, 10000, "Exp: %d");
            ImGui::SameLine();

            ImGui::SliderFloat("##Lens", &camInfo.requestLensPosition, 0.0f, 1.0f, "Lens: %.2f");
            ImGui::PopItemWidth();

            ImGui::SameLine();
            ImGui::Checkbox("##R", &camInfo.requestRawImage);
        }

        // Camera-Informationen als Overlay.
        if(Settings::instance.showCameraInfos){
            std::string str = "ID: " + std::to_string(camInfo.uniqueCamID)
                              + " | Marker: " + std::to_string(frame.markers.size())
                              + " | Trh: " + std::to_string(camInfo.brightnessThreshold)
                              + " | Exp: " + std::to_string(camInfo.exposureTime)
                              + " | Err: " + toString(camInfo.timeSyncError * 1000) + " ms";

            draw_list->AddText(ImVec2(imagePos.x + 10, imagePos.y + 5), ImGui::GetColorU32(ImGuiCol_Text), str.c_str());

            std::string str2 = "FPS: " + toString(camInfo.measuredFramerate)
                               + " | Drops/s: " + toString(camInfo.frameDropsLastSecond)
                               + " | Delay: " + toString(camInfo.captureToAvailableDelay * 1000) + " ms"
                               + " | Ping: " + toString(camInfo.ping) + " ms";

            draw_list->AddText(ImVec2(imagePos.x + 10, imagePos.y + 23), ImGui::GetColorU32(ImGuiCol_Text), str2.c_str());
        }

        const float xScale = imageWidth / camWidth;
        const float yScale = imageHeight / camHeight;

        // Bildmarker zeichnen.
        if(frame.valid){
            ImGui::PushFont(ViewData::getInstance().fontSmall);

            for(const ImageMarker& marker : frame.markers){
                float markerX = imagePos.x + marker.x * xScale;
                float markerY = imagePos.y + marker.y * yScale;

                std::string markerLabel;

                if(marker.marker3DIdx >= 0 && static_cast<size_t>(marker.marker3DIdx) < model.lastTick.markers.size())
                    markerLabel = std::to_string(model.lastTick.markers[marker.marker3DIdx].id % 100);

                draw_list->AddCircleFilled(ImVec2(markerX, markerY), fmaxf(1.f, sqrt(sqrt(marker.pixels)) / 1.5f), ImGui::GetColorU32(ImGuiCol_Text));

                if(Settings::instance.drawImageMarkerId)
                    draw_list->AddText(ImVec2(markerX, markerY), ImGui::GetColorU32(ImGuiCol_Text), markerLabel.c_str());

                if(Settings::instance.drawImageFlow)
                    draw_list->AddLine(ImVec2(markerX, markerY), ImVec2(markerX + marker.flowX * 3, markerY + marker.flowY * 3), ImGui::GetColorU32(ImGuiCol_Separator), 1.f);
            }

            ImGui::PopFont();
        }

        // Kalibrierungsmarker zeichnen.
        if(processors.cameraCalibrator->getIsCollectingObservations()){
            auto& observations = processors.cameraCalibrator->getObservations();
            auto obsIt = observations.find(frame.uniqueCamID);

            if(obsIt != observations.end()){
                for(const auto& observation : obsIt->second){
                    const ImageMarker& marker = observation.second;
                    float markerX = imagePos.x + marker.x * xScale;
                    float markerY = imagePos.y + marker.y * yScale;

                    draw_list->AddCircleFilled(ImVec2(markerX, markerY), fmaxf(1.f, sqrt(sqrt(marker.pixels)) / 2.0f), ImGui::GetColorU32(ImGuiCol_Separator));
                }
            }
        }

        static std::map<int, int> lastFrame;

        int currentFrame = ImGui::GetFrameCount();

        if(lastFrame.count(frame.uniqueCamID) && lastFrame[frame.uniqueCamID] == currentFrame)
            printf("DUPLICATE: Camera %d in ImGui frame %d\n", frame.uniqueCamID, currentFrame);

        lastFrame[frame.uniqueCamID] = currentFrame;

        ImGui::End();
    }
};
