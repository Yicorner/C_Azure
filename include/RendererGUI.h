#ifndef RENDERERGUI_H
#define RENDERERGUI_H

#define IMGUI_IMPL_OPENGL_LOADER_GLAD

#include "GlfwManager.h"
#include "RendererCore.h"
#include "glm/vec3.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ImGuiFileBrowser.h"

class RendererGUI
{
    public:
        RendererGUI(int window_width, int window_height, std::string title, bool is_fullscreen = false);
        ~RendererGUI();
        void run();
        void run_only_save_image(std::string fn, std::string ext);
        void run_only_image_content();
		void run_only_render(BodyLocation bodylocation, std::vector<std::vector<float>> body3Dlocation_list, int min_val = 0, int max_val = 255, float alpha_scale = 1);
        RendererCore volren;
        void setShaderAndData();

    private:
        void startFrame();
        void renderFrame();
        void showMenu();
        void showProfiler();
        void showHistogram();
        void showTools();
        void showHounsfieldScale();
        void enableToolsGUI();
        void showMessageBox(std::string title, std::string msg);
        void showHelpMarker(std::string desc);
        bool showRawInfPanel();


        GlfwManager glfw_manager;
        imgui_addons::ImGuiFileBrowser file_dialog;
        std::string error_msg, error_title;
        float mspf, mspk;
        int workgroups_x, workgroups_y, profiler_wheight, tools_wheight;
        bool profiler_shown, histogram_shown, tools_shown, HU_scale_shown, renderer_start;

};

#endif // RENDERERGUI_H
