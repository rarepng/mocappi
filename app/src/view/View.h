#pragma once

#define IMGUI_HEADCOL ImVec4(0.2f, 0.4f, 0.2f, 1.f)
#define IMGUI_HEADCOL_HOVER ImVec4(0.25f, 0.5f, 0.25f, 1.f)

#define IMGUI_HEADCOLBT ImVec4(0.2f, 0.4f, 0.2f, 1.f)
#define IMGUI_HEADCOLBT_HOVER ImVec4(0.25f, 0.5f, 0.25f, 1.f)

#define IMGUI_HEADCOLFR ImVec4(0.15f, 0.3f, 0.15f, 1.f)
#define IMGUI_HEADCOLFR_HOVER ImVec4(0.2f, 0.4f, 0.2f, 1.f)

#define IMGUI_BUTTONCOL_DISABLED ImVec4(0.4f, 0.4f, 0.4f, 1.f)
#define IMGUI_BUTTONCOL_DISABLED_HOVER ImVec4(0.5f, 0.5f, 0.5f, 1.f)

#define IMGUI_PIPELINE_HEADCOL ImVec4(0.4f, 0.2f, 0.2f, 1.f)
#define IMGUI_PIPELINE_HEADCOL_HOVER ImVec4(0.5f, 0.25f, 0.25f, 1.f)

// Menue width:
#define MENU_WIDTH 280

// Imgui
#include <imgui_internal.h>
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

// OpenGL
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// Include IO utilities:
#include <iostream>

#include "src/util/math/Mat4.h"
#include "src/util/gl/StaticMesh.h"
#include "src/util/gl/Shader.h"
#include "src/util/settings/Settings.h"

#include "src/view/ViewData.h"

#include "src/view/ui/widget/CameraImageUI.h"
#include "src/view/ui/menu/CalibrationPaneUI.h"
#include "src/view/ui/menu/RigidBodyPaneUI.h"
#include "src/view/ui/menu/SkeletonPaneUI.h"
#include "src/view/ui/menu/NetworkPaneUI.h"

#include "src/view/ui/elements/SwitchUI.h"

#include "src/model/DataModel.h"
#include "src/processors/ProcessorCollection.h"

#include "src/view/renderer/SkeletonRenderer.h"

using namespace std::chrono;

// Helper function to draw a 3D line:
void drawLine(Vec4f start, Vec4f end){
    GLuint VBO_LineDraw, VAO_LineDraw;
    glGenVertexArrays(1, &VAO_LineDraw);
    glGenBuffers(1, &VBO_LineDraw);

    glBindVertexArray(VAO_LineDraw);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_LineDraw);

    float vertices[] = {
        start.x, start.y, start.z,
        end.x, end.y, end.z
    };

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    glDrawArrays(GL_LINES, 0, 2); // Draw the line
    glBindVertexArray(0);

    glDeleteVertexArrays(1, &VAO_LineDraw);
    glDeleteBuffers(1, &VBO_LineDraw);
}

/**
 * The main view class using ImGUI and OpenGL
 */
class View {
    // These variables sum the mouse movements in x and y direction
    // while the mouse was pressed in window coordinates (i.e. pixels):
    float viewPositionX = 0.51f;
    float viewPositionY = -0.3f;
    float viewDistance = 2.0f;

    Vec4f viewTarget = Vec4f(0.65f, 1.f, 0.01f);

    // Opened windows:
    bool isImGuiDemoWindowOpen = false;

    // Rendering timer
    float worldGPUTime = 0.f;
    float worldCPUTime = 0.f;

    GLFWwindow* window;

    DataModel& model;
    ProcessorCollection& processors;

    // Last mouse position:
    ImVec2 lastMousePosition;
    ImVec2 selectionStartPosition;

    std::shared_ptr<StaticMesh> cameraMesh;
    std::shared_ptr<StaticMesh> rayMesh;
    std::shared_ptr<StaticMesh> sphereMesh;
    std::shared_ptr<StaticMesh> coordSystemMesh;
    std::shared_ptr<Shader> unlitColorShader;

    std::shared_ptr<SkeletonRenderer> skeletonRenderer;

public:
    View(DataModel& model, ProcessorCollection& processors)
        : model(model)
        , processors(processors)
    {
        // Setup window:
        glfwSetErrorCallback([](int error, const char* description) {
            fprintf(stderr, "Glfw Error %d: %s\n", error, description);
        });

        if (!glfwInit())
            exit(1);

        // Decide GL+GLSL versions:
        const char* glsl_version = "#version 330 core";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

        // Create window with graphics context:
        window = glfwCreateWindow(1120, 560, "MocapFramework (Dev)", NULL, NULL);
        if (window == NULL)
            exit(1);

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // Initialize GLAD functions:
        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Failed to initialize GLAD" << std::endl;
            exit(1);
        }

        // Setup Dear ImGui context:
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

        // Setup Dear ImGui style:
        ImGui::StyleColorsDark();

        // Setup Platform/Renderer backends:
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(glsl_version);

        // Enable depth test:
        glEnable(GL_DEPTH_TEST);

        // Last mouse position:
        lastMousePosition = ImGui::GetMousePos();

        ViewData::getInstance().fontDefault = io.Fonts->AddFontFromFileTTF(CMAKE_SOURCE_DIR "/lib/imgui-1.88/misc/fonts/Roboto-Medium.ttf", 16.0f);
        ViewData::getInstance().fontSmall = io.Fonts->AddFontFromFileTTF(CMAKE_SOURCE_DIR "/lib/imgui-1.88/misc/fonts/Roboto-Medium.ttf", 11.0f);
        ViewData::getInstance().fontSSH = io.Fonts->AddFontFromFileTTF(CMAKE_SOURCE_DIR "/lib/imgui-1.88/misc/fonts/ProggyClean.ttf", 16.0f);

        lastMousePosition = ImGui::GetMousePos();
        cameraMesh = std::make_shared<StaticMesh>(CMAKE_SOURCE_DIR "/data/model/camera.obj");
        rayMesh = std::make_shared<StaticMesh>(CMAKE_SOURCE_DIR "/data/model/ray.obj");
        sphereMesh = std::make_shared<StaticMesh>(CMAKE_SOURCE_DIR "/data/model/sphere.obj");
        coordSystemMesh = std::make_shared<StaticMesh>(CMAKE_SOURCE_DIR "/data/model/coordinateSystem.obj");
        unlitColorShader = std::make_shared<Shader>(CMAKE_SOURCE_DIR "/shader/unlitColor.vert", CMAKE_SOURCE_DIR "/shader/unlitColor.frag");

        skeletonRenderer = std::make_shared<SkeletonRenderer>();
    }

    ~View(){
        // Cleanup
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        // Cleanup
        glfwDestroyWindow(window);
        glfwTerminate();
    }

    void render(){
        auto frameStartTime = high_resolution_clock::now();

        // Start measuring time:
        GLuint timingQuery;
        glGenQueries(1, &timingQuery);
        glBeginQuery(GL_TIME_ELAPSED, timingQuery);

        // Processes all glfw events:
        glfwPollEvents();

        // Get the size of the window:
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        // Subtract right menu:
        display_w -= 2*MENU_WIDTH;

        // Create the GUI:
        {
            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            // Create Window with settings:
            ImGui::SetNextWindowPos(ImVec2(0,0));
            ImGui::SetNextWindowSize(ImVec2(float(MENU_WIDTH),float(display_h)));
            ImGui::Begin("Menu", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);


            // Collapsing Header Colors
            ImGui::PushStyleColor(ImGuiCol_Header, IMGUI_HEADCOL);
            ImGui::PushStyleColor(ImGuiCol_HeaderActive, IMGUI_HEADCOL);
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, IMGUI_HEADCOL_HOVER);

            if(ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)){

                SwitchUI::render({"3D View", "Camera View"}, Settings::instance.leftTabOpenIdx);
                ImGui::Separator();
                if(Settings::instance.leftTabOpenIdx == 0){
                    ImGui::Checkbox("Show 3D Markers", &Settings::instance.show3DMarkers);
                    ImGui::Checkbox("Show Intersection", &Settings::instance.show3DIntersections);
                    ImGui::Checkbox("Show Ground Truth", &Settings::instance.showGroundTruthMarkers);
                    ImGui::Checkbox("Show Rays", &Settings::instance.showMarkerRays);
                    ImGui::Checkbox("Show Rigid Bodies", &Settings::instance.showRigidBodies);
                } else {
                    ImGui::Checkbox("Show Camera Infos", &Settings::instance.showCameraInfos);
                    ImGui::Checkbox("Show Camera Controls", &Settings::instance.showCameraControls);
                    ImGui::Checkbox("Draw Marker Flow", &Settings::instance.drawImageFlow);
                    ImGui::Checkbox("Draw Marker ID", &Settings::instance.drawImageMarkerId);
                }
                ImGui::Separator();
                ImGui::Text("");
                ImGui::Separator();
            }


            if(ImGui::CollapsingHeader("Pipeline", ImGuiTreeNodeFlags_DefaultOpen)){
                int framerate = int(processors.cameraDataProvider->requestedFramerate);
                ImGui::SliderInt("Cam FPS", &framerate, 15, 100);
                processors.cameraDataProvider->requestedFramerate = framerate;

                ImGui::SliderFloat("Processing", &processors.requestedProcessingRate, 30, 240);
                ImGui::SliderFloat("Delay", &processors.processingDelay, 0.01, 0.2);

                ImGui::Separator();
                ImGui::Text("");
                ImGui::Separator();
            }


            if(ImGui::CollapsingHeader("Mask Areas", ImGuiTreeNodeFlags_DefaultOpen)){

                std::string buttonLabel = "Start Recording";

                if(processors.cameraDataProvider->shouldRecordAMask){
                    buttonLabel = "Stop Recording";
                }

                if(ImGui::Button(buttonLabel.c_str(), ImVec2(ImGui::GetContentRegionAvail().x, 24))){
                    processors.cameraDataProvider->shouldRecordAMask = !processors.cameraDataProvider->shouldRecordAMask;
                }

                if(ImGui::Button("Reset Masks", ImVec2(ImGui::GetContentRegionAvail().x, 24))){
                    processors.cameraDataProvider->resetCameraMasks();
                }

                ImGui::Separator();
                ImGui::Text("");
                ImGui::Separator();
            }

            // Render the calibration pane
            CalibrationPaneUI::render(model, processors);

            if(ImGui::CollapsingHeader("Markers", ImGuiTreeNodeFlags_DefaultOpen)){
                ImGui::Text("Markers: %i", int(model.lastTick.markers.size()));
                ImGui::Separator();
                ImGui::Text("Ground Truth Distance: %.2f mm", float(processors.markerTriangulator->medianGroundTruthDistance * 1000));
                ImGui::Text("3D Intersection Error: %.2f mm", float(processors.markerTriangulator->medianInnerClusterDistance * 1000));
                ImGui::Text("Intersections per Cluster: %.2f", float(processors.markerTriangulator->meanClusterNum));
                ImGui::Separator();
                ImGui::Text("");
                ImGui::Separator();
            }

            if(ImGui::CollapsingHeader("Performance", ImGuiTreeNodeFlags_DefaultOpen)){
                ImGui::Text("CPU Render Time: %.3f ms", worldCPUTime);
                ImGui::Text("GL Render Time: %.3f ms", worldGPUTime);
                ImGui::Separator();
                ImGui::Text("Processing Rate: %.2f", float(processors.measuredProcessingRate));
                ImGui::Separator();
                std::map<std::string, float> timeMeasures = processors.getTimeMeasuresInMilliSec();

                for (const auto &elem : timeMeasures) {
                    ImGui::Text((elem.first+": %.3f ms").c_str(), elem.second);
                }
            }

            ImGui::PopStyleColor(3);
            ImGui::Separator();

            bool isOpenTmp = isImGuiDemoWindowOpen;
            if(isOpenTmp){
                ImGui::PushStyleColor(ImGuiCol_Button, IMGUI_BUTTONCOL_DISABLED);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IMGUI_BUTTONCOL_DISABLED_HOVER);
            }
            if(ImGui::Button("(DEV): Open ImGui Demo Window", ImVec2(ImGui::GetContentRegionAvail().x, 24))){
                isImGuiDemoWindowOpen = !isImGuiDemoWindowOpen;
            }
            if(isOpenTmp){
                ImGui::PopStyleColor(2);
            }

            ImGui::End();

            if(isImGuiDemoWindowOpen){
                ImGui::ShowDemoWindow(&isImGuiDemoWindowOpen);
            }

            if(Settings::instance.leftTabOpenIdx == 1 && !model.lastTick.frames.empty()){
                int yNum = int(std::ceil(sqrt(model.lastTick.frames.size())));
                int xNum = int(std::ceil(model.lastTick.frames.size() / float(yNum)));

                if(xNum > 0 && yNum > 0){
                    int xSize = display_w / xNum;
                    int ySize = display_h / yNum;

                    for(int y = 0; y < yNum; ++y){
                        for(int x = 0; x < xNum; ++x){
                            if(x + y * xNum < model.lastTick.frames.size()){
                                CameraImageUI::renderCameraUI(MENU_WIDTH + x * xSize, y * ySize, xSize, ySize, model, processors, model.lastTick.frames[x + y * xNum]);
                            }
                        }
                    }
                }
            }

            //right window
        
            ImGui::SetNextWindowPos(ImVec2(float(ImGui::GetIO().DisplaySize.x-MENU_WIDTH),float(0)));
            ImGui::SetNextWindowSize(ImVec2(float(MENU_WIDTH),float(display_h)));
            ImGui::Begin("Body Pane", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

#ifdef DEVELOPMENT_MODE
            SwitchUI::render({"Network", "Rigid Bodies", "Skeletons"}, Settings::instance.rightTabOpenIdx);
#else
            SwitchUI::render({"Network", "Rigid Bodies"}, Settings::instance.rightTabOpenIdx);
#endif
            ImGui::Separator();
            if(Settings::instance.rightTabOpenIdx == 0){
                NetworkPaneUI::render(model, processors);
            } else if(Settings::instance.rightTabOpenIdx == 1){
                RigidBodyPaneUI::render(model, processors);
            } else if(Settings::instance.rightTabOpenIdx == 2){
                SkeletonPaneUI::render(model, processors);
            }

            ImGui::Separator();
            ImGui::End();
        }

        bool wasChanged = false;

        // If mouse is down, simply add the difference to viewPositionX and viewPositionY (ROTATION):
        if(Settings::instance.leftTabOpenIdx == 0 && ImGui::IsMouseDown(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse){
            ImVec2 mouseDelta = ImGui::GetMousePos() - lastMousePosition;

            // Add the delta to mouseMovementX and mouseMovementY:
            constexpr float orbitSensitivity = 0.3f * (M_PI / 180.0f);
            constexpr float maxPitch = 89.0f * (M_PI / 180.0f);
            viewPositionX = std::fmod(viewPositionX - mouseDelta.x * orbitSensitivity, 2.0f * M_PI);
            viewPositionY = std::clamp(viewPositionY - mouseDelta.y * orbitSensitivity, -maxPitch, maxPitch);

            wasChanged = true;
        }

        // Generate a rotation matrix (which is changed when the mouse is dragged):
        Mat4f rotationMat({cos(viewPositionX), -sin(viewPositionY) * -sin(viewPositionX), cos(viewPositionY) * -sin(viewPositionX), 0, 0, cos(viewPositionY), sin(viewPositionY), 0, sin(viewPositionX), -sin(viewPositionY) * cos(viewPositionX), cos(viewPositionY) * cos(viewPositionX), 0, 0,0,0,1});


        // If middle mouse is down, shift viewTarget (OFFSET):
        if(Settings::instance.leftTabOpenIdx == 0 && ImGui::IsMouseDown(ImGuiMouseButton_Middle) && !ImGui::GetIO().WantCaptureMouse){
            ImVec2 mouseDelta = ImGui::GetMousePos() - lastMousePosition;

            Mat4f invRotationMat = rotationMat.inverse();
            Vec4f xDir = invRotationMat * Vec4f(1, 0, 0, 0);
            Vec4f yDir = invRotationMat * Vec4f(0, 1, 0, 0);

            viewTarget = viewTarget + (-xDir * mouseDelta.x * 0.002f * viewDistance) + (yDir * mouseDelta.y * 0.002f * viewDistance);

            wasChanged = true;
        }

        /*
        if(wasChanged){
            std::cout << "ViewPositionX: " << viewPositionX << std::endl;
            std::cout << "ViewPositionY: " << viewPositionY << std::endl;
            std::cout << "ViewTargetX: " << viewTarget.x << std::endl;
            std::cout << "ViewTargetY: " << viewTarget.y << std::endl;
            std::cout << "ViewTargetZ: " << viewTarget.z << std::endl;
            std::cout << std::endl;
        }*/

        // If mouse wheel moved, change the view distance to the scene mid point:
        ImVec2 mousePos = ImGui::GetMousePos();
        bool is3DViewHovered = Settings::instance.leftTabOpenIdx == 0
                               && !ImGui::GetIO().WantCaptureMouse
                               && mousePos.x >= float(MENU_WIDTH) && mousePos.x < float(MENU_WIDTH + display_w)
                               && mousePos.y >= 0.f && mousePos.y < float(display_h);
        if(is3DViewHovered && !ImGui::GetIO().WantTextInput){
            Mat4f invRotationMat = rotationMat.inverse();
            Vec4f rightDir = invRotationMat * Vec4f(1.f, 0.f, 0.f, 0.f);
            Vec4f upDir = invRotationMat * Vec4f(0.f, 1.f, 0.f, 0.f);
            Vec4f forwardDir = invRotationMat * Vec4f(0.f, 0.f, 1.f, 0.f);
            Vec4f moveDir(0.f, 0.f, 0.f, 0.f);
            if(ImGui::IsKeyDown(ImGuiKey_W)) moveDir = moveDir + forwardDir;
            if(ImGui::IsKeyDown(ImGuiKey_S)) moveDir = moveDir - forwardDir;
            if(ImGui::IsKeyDown(ImGuiKey_D)) moveDir = moveDir + rightDir;
            if(ImGui::IsKeyDown(ImGuiKey_A)) moveDir = moveDir - rightDir;
            if(ImGui::IsKeyDown(ImGuiKey_E)) moveDir = moveDir + upDir;
            if(ImGui::IsKeyDown(ImGuiKey_Q)) moveDir = moveDir - upDir;
            if(moveDir.squaredLength() > 0.f){
                float speedMultiplier = 1.0f;
                if(ImGui::GetIO().KeyShift) speedMultiplier *= 4.0f;
                if(ImGui::GetIO().KeyCtrl) speedMultiplier *= 0.25f;
                float step = std::max(viewDistance, 0.02f) * speedMultiplier * ImGui::GetIO().DeltaTime;
                viewTarget = viewTarget + moveDir.normalized() * step;
                wasChanged = true;
            }
        }
        if(is3DViewHovered && ImGui::GetIO().MouseWheel != 0.0f){
            float wheel = ImGui::GetIO().MouseWheel;
            float zoomMultiplier = 1.0f;
            if(ImGui::GetIO().KeyShift) zoomMultiplier *= 4.0f;
            if(ImGui::GetIO().KeyCtrl) zoomMultiplier *= 0.25f;
            constexpr float minDistance = 0.02f;
            constexpr float maxDistance = 50.0f;
            constexpr float minStep = 0.005f;
            float step = std::max(viewDistance * 0.15f, minStep) * zoomMultiplier * std::abs(wheel);
            if(wheel > 0.0f){
                if(viewDistance - step >= minDistance){
                    viewDistance -= step;
                } else {
                    float push = minDistance - (viewDistance - step);
                    viewDistance = minDistance;
                    Vec4f forwardDir = (rotationMat.inverse() * Vec4f(0.f, 0.f, 1.f, 0.f)).normalized();
                    viewTarget = viewTarget + forwardDir * push;
                }
            } else {
                viewDistance = std::min(viewDistance + step, maxDistance);
            }
            wasChanged = true;
        }

        // Store this mouse position for the next frame:
        lastMousePosition = ImGui::GetMousePos();

        // Setup the GL viewport
        glViewport(MENU_WIDTH, 0, display_w, display_h);
        glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Calculate aspect ratio (might be used for perspective transformation):
        float aspectRatio = float(display_w) / display_h;

        // Ensure that both sides of the triangle are rendered:
        glDisable(GL_CULL_FACE);
        glCullFace(GL_FRONT);

        // Get projection matrix:
        Mat4f projection = Mat4f::perspectiveTransformation(aspectRatio, 75.f, 0.001f, 100.f);

        // Generate the view matrix by using the rotation matrix and translate the camera
        // by the distance to the scene mid point:
        Mat4f view = Mat4f::translation(0, 0, viewDistance) * rotationMat * Mat4f::translation(-viewTarget.x, -viewTarget.y, -viewTarget.z);

        // Render all the objects in the scene:
        {
            glDisable(GL_DEPTH_TEST);
            unlitColorShader->bind();

            unlitColorShader->setUniform("projection", projection);
            unlitColorShader->setUniform("view", view);
            unlitColorShader->setUniform("color", Vec4f(1,0,0));

            for(auto& camEntry : model.cameras){
                CameraCalibration& calibration = camEntry.second.calibration;
                unlitColorShader->setUniform("model", calibration.getModelMatrix() * Mat4f::scale(0.1f));
                cameraMesh->render(GL_TRIANGLES);
            }

            skeletonRenderer->render(projection, view, model);

            if(Settings::instance.showMarkerRays){
                for(CameraFrame frame : model.lastTick.frames){
                    if(model.cameras.count(frame.uniqueCamID) == 0)
                        continue;

                    Camera& camera = model.cameras[frame.uniqueCamID];
                    CameraCalibration& calibration = camera.calibration;

                    Vec4f color = Vec4f(0.f, 1.f, 1.f, 1.f);
                    unlitColorShader->setUniform("model", calibration.getModelMatrix());
                    unlitColorShader->setUniform("color", color);
                    for(ImageMarker& marker : frame.markers){
                        Vec4f m2D = marker.getCoordinates();
                        Vec4f ray = calibration.getRay(m2D.x, m2D.y);
                        drawLine(Vec4f(), Vec4f(ray.x * 5, ray.y * 5, ray.z * 5, 1));
                    }
                }
            }
            glEnable(GL_DEPTH_TEST);
        }

        if(Settings::instance.showGroundTruthMarkers){
            unlitColorShader->setUniform("color", Vec4f(0.5f, 0.6f, 0.7f));
            for(Vec4f p : model.lastTick.markersGTPos){
                unlitColorShader->setUniform("model", Mat4f::translation(p.x, p.y, p.z) * Mat4f::scale(0.01f));
                sphereMesh->render();
            }
        }

        if(Settings::instance.show3DMarkers){
            // draw unlabeled markers
            unlitColorShader->setUniform("color", Vec4f(0.65f, 0.65f, 0.65f));
            for(Marker marker3D : model.markers){
                if(marker3D.label.empty() && marker3D.obsoleteCounter == 0) {
                    Vec4f p = marker3D.position;
                    unlitColorShader->setUniform("model", Mat4f::translation(p.x, p.y, p.z) * Mat4f::scale(0.01f));
                    sphereMesh->render();
                }
            }

            // draw labeled markers, part of rigid bodies
            unlitColorShader->setUniform("color", Vec4f(0.65f, 0.65f, 0.65f));
            for(Marker marker : model.markers){
                if(!marker.label.empty()) {
                    //std::cout << marker3D.label << std::endl;
                    Vec4f p = marker.position;
                    unlitColorShader->setUniform("model", Mat4f::translation(p.x, p.y, p.z) * Mat4f::scale(0.04f));
                    sphereMesh->render();
                }
            }

            // draw selected markers
            unlitColorShader->setUniform("color", Vec4f(1.f, 0.f, 0.f));
            for(Marker marker : model.markers){
                if( marker.isSelected == true) {
                    //std::cout << marker3D.label << std::endl;
                    Vec4f p = marker.position;
                    unlitColorShader->setUniform("model", Mat4f::translation(p.x, p.y, p.z) * Mat4f::scale(0.04f));
                    sphereMesh->render();
                }
            }
        }

        if(Settings::instance.showRigidBodies){
            for(RigidBody& rb : model.rigidBodies){
                if(!rb.bodyFound)
                    continue;

                int i= rb.label == "Head" ? 0 : rb.label == "Hand Left" ? 1 : 2;

                unlitColorShader->setUniform("model", Mat4f());

                Mat4f pose = rb.pose;
                Vec4f position = pose.getPosition();
                Vec4f xPos = position + pose * Vec4f(0.05f, 0.0, 0.0, 0.0);
                Vec4f yPos = position + pose * Vec4f(0.f, 0.05f, 0.0, 0.0);
                Vec4f zPos = position + pose * Vec4f(0.f, 0.f, 0.05f, 0.0);

                unlitColorShader->setUniform("color", Vec4f(1.f, 0.f, 0.f, 1.f));
                drawLine(position, xPos);

                unlitColorShader->setUniform("color", Vec4f(0.f, 1.f, 0.f, 1.f));
                drawLine(position, yPos);

                unlitColorShader->setUniform("color", Vec4f(0.f, 0.f, 1.f, 1.f));
                drawLine(position, zPos);

                unlitColorShader->setUniform("color", Vec4f(i==0, i==1, i==2, 1.f));
                unlitColorShader->setUniform("model", Mat4f::translation(position.x, position.y, position.z) * Mat4f::scale(0.04f));
                sphereMesh->render();

                for(RigidBodyMarker& m : rb.markers){
                    unlitColorShader->setUniform("color", Vec4f(1.f, 1.f, 1.f, 1.f));
                    unlitColorShader->setUniform("model", pose * Mat4f::translation(m.position.x, m.position.y, m.position.z) * Mat4f::scale(0.03f));
                    sphereMesh->render();
                }
            }
        }


        if(Settings::instance.show3DIntersections){
            // draw unlabeled markers
            unlitColorShader->setUniform("color", Vec4f(0.5f, 0.5f, 1.f));
            for(Vec4f intersection : model.lastTick.triangulationIntersections){
                unlitColorShader->setUniform("model", Mat4f::translation(intersection.x, intersection.y, intersection.z) * Mat4f::scale(0.005f));
                sphereMesh->render();
            }
        }

        if(wasChanged){
            float crossSize = std::max(0.002f, viewDistance * 0.025f);
            unlitColorShader->setUniform("model", Mat4f());
            unlitColorShader->setUniform("color", Vec4f(1.f, 0.2f, 0.2f, 1.f));
            drawLine(viewTarget - Vec4f(crossSize, 0.f, 0.f, 0.f), viewTarget + Vec4f(crossSize, 0.f, 0.f, 0.f));
            unlitColorShader->setUniform("color", Vec4f(0.2f, 1.f, 0.2f, 1.f));
            drawLine(viewTarget - Vec4f(0.f, crossSize, 0.f, 0.f), viewTarget + Vec4f(0.f, crossSize, 0.f, 0.f));
            unlitColorShader->setUniform("color", Vec4f(0.2f, 0.6f, 1.f, 1.f));
            drawLine(viewTarget - Vec4f(0.f, 0.f, crossSize, 0.f), viewTarget + Vec4f(0.f, 0.f, crossSize, 0.f));
        }

        unlitColorShader->setUniform("color", Vec4f(1.f, 0.f, 0.f));

        // End measuring time:
        glEndQuery(GL_TIME_ELAPSED);

        // Wait until time is available:
        int resultAvailable = 0;
        while (!resultAvailable) {
            glGetQueryObjectiv(timingQuery, GL_QUERY_RESULT_AVAILABLE, &resultAvailable);
        }

        // Render the GUI and draw it to the screen:
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Rendering time:
        GLuint64 time = 0;
        glGetQueryObjectui64v(timingQuery, GL_QUERY_RESULT, &time);
        worldGPUTime = (time / 1000000.f) * 0.1f + worldGPUTime * 0.9f;

        // Delete queries:
        glDeleteQueries(1,&timingQuery);

        // Measure time (before swap):
        long long frameDuration = duration_cast<microseconds>(high_resolution_clock::now() - frameStartTime).count();
        worldCPUTime = (frameDuration *0.001f) * 0.1f + worldCPUTime * 0.9f;

        // Swap Buffers (waits for vsync (?)):
        glfwSwapBuffers(window);
    }

    GLFWwindow* getMainWindow(){
        return window;
    }
};


