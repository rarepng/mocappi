#pragma once

#include <imgui_internal.h>
#include <imgui.h>

#include "src/util/gl/Texture2D.h"

#include <nlohmann/json.hpp>

/**
 * View data (fonts, icons, etc.).
 *
 * Singleton pattern, because the textures have to be
 * initialized after OpenGL was setup (can't be static).
 */

struct ViewData {
    ImFont* fontSmall = nullptr;
    ImFont* fontDefault = nullptr;
    ImFont* fontSSH = nullptr;

    Texture2D camIconTexture = Texture2D(CMAKE_SOURCE_DIR "/data/icon/cam.png");
    Texture2D sshIconTexture = Texture2D(CMAKE_SOURCE_DIR "/data/icon/ssh.png");

    static ViewData& getInstance(){
        if(instance == nullptr){
            instance = new ViewData();
        }

        return *instance;
    }

private:
    static inline ViewData* instance{nullptr};
};
