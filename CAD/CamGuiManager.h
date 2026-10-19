#pragma once

class CadApplication;

class CamGuiManager
{
public:
    explicit CamGuiManager(CadApplication& app) : m_app(app) {}

    void DrawMenu();
    bool ShouldDrawMillingPath() const { return m_showMillingPath; }
private:
    void DrawModeSelector();
    void DrawGCodeSection();
    void DrawMaterialSection();
    void DrawSimulationSection();
    void DrawCutterSection();

    CadApplication& m_app;

    std::string m_loadedGCodePath = "";
    std::string m_loadedGCodeFile = "None";

#pragma region Material
    int m_simDivX = 600;
    int m_simDivZ = 600;
    float m_simSizeX = 15.0f;
    float m_simSizeY = 5.0f;
    float m_simSizeZ = 15.0f;
    float m_simBaseHeight = 15.0f;
#pragma endregion

#pragma region Simulation    
    float m_simSpeed = 10.0f;
    bool m_showMillingPath = true;
    bool m_showErrorPopup = true;
#pragma endregion

#pragma region Cutter
    float m_cutterSize = 1.0f;
    float m_cutterHeight = 4.0f;
    bool m_cutterFlat = false;
#pragma endregion
};