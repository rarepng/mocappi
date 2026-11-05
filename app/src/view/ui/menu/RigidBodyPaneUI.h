
#pragma once

#include <imgui_internal.h>
#include <imgui.h>

#include "src/processors/ProcessorCollection.h"

#include <string>
#include <cstdio>
#include <cstring>
#include <algorithm>

struct RigidBodyPaneUI {

    static void render(DataModel& model, ProcessorCollection& processors) {
        static int editingBody = -1;
        static bool focusNameInput = false;
        static char nameBuffer[256] = {};

        if (!ImGui::CollapsingHeader("Rigid Bodies", ImGuiTreeNodeFlags_DefaultOpen)) return;

        if (ImGui::Button("+ Add Rigid Body", ImVec2(-1, 0))) {
            RigidBody body;
            body.label = "RigidBody " + std::to_string(model.rigidBodies.size() + 1);
            body.isTracked = false;
            model.rigidBodies.push_back(body);
        }

        ImGui::Spacing();

        int bodyToDelete = -1;

        for (int i = 0; i < static_cast<int>(model.rigidBodies.size()); ++i) {
            RigidBody& body = model.rigidBodies[i];
            ImGui::PushID(i);

            float frameHeight = ImGui::GetFrameHeight();
            float deleteWidth = frameHeight;
            float spacing = ImGui::GetStyle().ItemSpacing.x;

            // Tracking
            ImGui::Checkbox("##tracked", &body.isTracked);
            ImGui::SameLine();

            // Header-Bereich bis zum X-Button
            float headerStartX = ImGui::GetCursorPosX();
            float availableWidth = ImGui::GetContentRegionAvail().x;
            float deleteX = headerStartX + availableWidth - deleteWidth;
            float headerWidth = std::max(1.0f, availableWidth - deleteWidth - spacing);

            // TreeNode behält seinen Zustand unabhängig vom Namen.
            // Während der Bearbeitung bleibt der Arrow als TreeNode sichtbar.
            bool open = false;

            if (editingBody == i) {
                // Kleiner TreeNode nur als Aufklapp-Pfeil
                ImGui::SetNextItemOpen(true, ImGuiCond_Always);
                open = ImGui::TreeNodeEx("##body");

                // Input direkt neben dem Pfeil
                ImGui::SameLine();
                float inputWidth = std::max(1.0f, deleteX - ImGui::GetCursorPosX() - spacing);
                ImGui::SetNextItemWidth(inputWidth);

                if (focusNameInput) {
                    ImGui::SetKeyboardFocusHere();
                    focusNameInput = false;
                }

                bool confirmed = ImGui::InputText("##name", nameBuffer, sizeof(nameBuffer), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);

                if (confirmed) {
                    if (std::strlen(nameBuffer) > 0) body.label = nameBuffer;
                    editingBody = -1;
                } else if (ImGui::IsItemActive() && ImGui::IsKeyPressed(ImGuiKey_Escape)) {
                    editingBody = -1;
                }

            } else {
                open = ImGui::TreeNodeEx("##body", ImGuiTreeNodeFlags_DefaultOpen, "%s", body.label.c_str());

                if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    editingBody = i;
                    focusNameInput = true;
                    std::snprintf(nameBuffer, sizeof(nameBuffer), "%s", body.label.c_str());
                }
            }

            // X ganz rechts
            ImGui::SameLine();
            ImGui::SetCursorPosX(deleteX);

            if (ImGui::Button("X", ImVec2(deleteWidth, frameHeight))) {
                bodyToDelete = i;
            }

            if (open) {
                ImGui::Indent();

                ImGui::TextDisabled("%d Markers", static_cast<int>(body.markers.size()));

                // added colors for significance around < 0.5 mm :)
                if (body.groundTruthTipErrorP99 >= 0.f) {
                    const ImVec4 colour = body.groundTruthTipErrorP99 <= 0.5f ? ImVec4(0.4f, 0.9f, 0.4f, 1.f) : ImVec4(1.f, 0.6f, 0.3f, 1.f);
                    if (body.groundTruthTipError >= 0.f)
                        ImGui::TextColored(colour, "Tip error %.2f mm, P99 %.2f mm (10 s)", body.groundTruthTipError, body.groundTruthTipErrorP99);
                    else
                        ImGui::TextColored(colour, "Tip error: lost, P99 %.2f mm (10 s)", body.groundTruthTipErrorP99);
                }
                ImGui::Spacing();

                bool markersChanged = false;
                int markerToDelete = -1;

                // Marker anzeigen und bearbeiten
                for (int j = 0; j < static_cast<int>(body.markers.size()); ++j) {
                    RigidBodyMarker& marker = body.markers[j];
                    ImGui::PushID(j);

                    ImGui::AlignTextToFramePadding();
                    ImGui::TextDisabled("%d", j);
                    ImGui::SameLine();

                    float position[3] = {marker.position.x, marker.position.y, marker.position.z};

                    float removeWidth = ImGui::GetFrameHeight();
                    float positionWidth = std::max(1.0f, ImGui::GetContentRegionAvail().x - removeWidth - spacing);
                    ImGui::SetNextItemWidth(positionWidth);

                    if (ImGui::DragFloat3("##position", position, 0.1f, 0.0f, 0.0f, "%.3f")) {
                        marker.position.x = position[0];
                        marker.position.y = position[1];
                        marker.position.z = position[2];
                        marker.position.w = 1.0f;
                        markersChanged = true;
                    }

                    ImGui::SameLine();

                    if (ImGui::Button("X", ImVec2(removeWidth, 0))) {
                        markerToDelete = j;
                    }

                    ImGui::PopID();
                }

                // Marker entfernen
                if (markerToDelete >= 0) {
                    body.markers.erase(body.markers.begin() + markerToDelete);
                    markersChanged = true;
                }

                ImGui::Spacing();

                // Marker hinzufügen
                if (ImGui::SmallButton("+ Add Marker")) {
                    Vec4f position;
                    position.x = 0.0f;
                    position.y = 0.0f;
                    position.z = 0.0f;
                    position.w = 1.0f;

                    body.markers.emplace_back(position);
                    markersChanged = true;
                }

                // Mittelpunkt und Abstände aktualisieren
                if (markersChanged) {
                    if (!body.markers.empty()) {
                        body.init();
                    } else {
                        body.averagePoint = Vec4f();
                        body.averagePoint.w = 1.0f;
                        body.isTracked = false;
                    }
                }

                ImGui::Unindent();
                ImGui::TreePop();
            }

            ImGui::Separator();
            ImGui::PopID();
        }

        // Nach der Iteration löschen
        if (bodyToDelete >= 0) {
            model.rigidBodies.erase(model.rigidBodies.begin() + bodyToDelete);

            if (editingBody == bodyToDelete) {
                editingBody = -1;
                focusNameInput = false;
            } else if (editingBody > bodyToDelete) {
                --editingBody;
            }
        }
    }
};
