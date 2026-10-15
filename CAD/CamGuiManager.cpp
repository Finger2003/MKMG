#include "pch.h"
#include "CamGuiManager.h"
#include "CadApplication.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"

namespace
{
	struct CutterConfig
	{
		bool isFlat = true;
		float diameter = 1.0f; // mm
	};

	std::optional<CutterConfig> ParseCutterFromExtension(const std::string& filepath)
	{
		size_t dotPos = filepath.find_last_of('.');
		if (dotPos == std::string::npos)
			return std::nullopt;

		std::string ext = filepath.substr(dotPos + 1); // e.g. "k08", "f12"
		if (ext.length() < 3)
			return std::nullopt;

		char typeChar = static_cast<char>(std::tolower(ext[0]));
		bool isFlat = false;

		if (typeChar == 'f')
			isFlat = true;  // flat end mill
		else if (typeChar == 'k')
			isFlat = false; // ball nose end mill
		else
			return std::nullopt;

		try
		{
			int diameterMm = std::stoi(ext.substr(1));
			if (diameterMm <= 0)
				return std::nullopt;

			return CutterConfig{ isFlat, static_cast<float>(diameterMm) };
		}
		catch (...)
		{
			return std::nullopt;
		}
	}
}

void CamGuiManager::DrawMenu()
{
	ImGui::SetNextWindowPos(ImVec2(m_app.m_renderSize.cx, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(m_app.cMenuWidth, m_app.m_renderSize.cy));

	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoCollapse;

	ImGui::Begin("Inspector", nullptr, windowFlags);

	DrawModeSelector();
	DrawGCodeSection();
	DrawMaterialSection();
	DrawSimulationSection();
	DrawCutterSection();

	ImGui::End();
}

void CamGuiManager::DrawModeSelector()
{
	int currentMode = static_cast<int>(m_app.getAppMode());
	ImGui::Text("Application Mode");
	if (ImGui::RadioButton("CAD Modeling", &currentMode, 0))
		m_app.setAppMode(AppMode::CAD_Modeling);

	ImGui::SameLine();
	if (ImGui::RadioButton("CAM Simulation", &currentMode, 1))
		m_app.setAppMode(AppMode::CAM_Simulation);

	ImGui::Separator();
	ImGui::Spacing();
}

void CamGuiManager::DrawGCodeSection()
{
	if (ImGui::Button("Open file", ImVec2(-1.0f, 0.0f)))
	{
		const std::vector<COMDLG_FILTERSPEC> gcodeFilters = {
			{ L"Machine Code Files (*.k*, *.f*)", L"*.k*;*.f*" },
			{ L"All Files (*.*)", L"*.*" }
		};
		std::wstring widePath = m_app.ShowFileDialog<IFileOpenDialog>(gcodeFilters, nullptr, nullptr);
		if (!widePath.empty())
		{
			std::string tempPath(widePath.begin(), widePath.end());

			size_t lastSlash = tempPath.find_last_of("\\/");
			std::string tempFilename = (lastSlash != std::string::npos)
				? tempPath.substr(lastSlash + 1)
				: tempPath;

			auto cutterCfgOpt = ParseCutterFromExtension(tempFilename);
			if (!cutterCfgOpt)
			{
				MessageBoxA(
					m_app.getWindowHandle(),
					"Type and diameter of the cutter could not be recognized from the file extension.\n\n"
					"Required extension format:\n"
					" - .kXX for ball nose end mill (e.g., .k08)\n"
					" - .fXX for flat end mill (e.g., .f12)\n\n"
					"File was not loaded.",
					"Extension Warning",
					MB_OK | MB_ICONWARNING
				);
			}
			else if (!m_app.getMillingSimulator().LoadGCode(tempPath))
			{
				MessageBoxA(
					m_app.getWindowHandle(),
					"Failed to open the file or no valid G01 instructions were found.",
					"G-Code Load Error",
					MB_OK | MB_ICONERROR
				);
			}
			else
			{
				m_loadedGCodePath = std::move(tempPath);
				m_loadedGCodeFile = std::move(tempFilename);

				m_cutterFlat = cutterCfgOpt->isFlat;
				m_cutterSize = cutterCfgOpt->diameter;

				m_app.getMillingSimulator().setCutterParams({ m_cutterSize, m_cutterHeight, m_cutterFlat });
			}
		}
	}

	ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "File:");
	ImGui::SameLine();
	ImGui::TextWrapped("%s", m_loadedGCodeFile.c_str());

	ImGui::Spacing();
	ImGui::Separator();
}

void CamGuiManager::DrawMaterialSection()
{
	if (ImGui::CollapsingHeader("Material", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::InputInt("divisions x", &m_simDivX);
		ImGui::InputInt("divisions z", &m_simDivZ);

		m_simDivX = std::max(2, m_simDivX);
		m_simDivZ = std::max(2, m_simDivZ);

		ImGui::InputFloat("size x [cm]", &m_simSizeX, 0.5f, 1.0f, "%.1f");
		ImGui::InputFloat("size y [cm]", &m_simSizeY, 0.5f, 1.0f, "%.1f");
		ImGui::InputFloat("size z [cm]", &m_simSizeZ, 0.5f, 1.0f, "%.1f");
		ImGui::InputFloat("base height", &m_simBaseHeight, 0.5f, 1.0f, "%.1f");
	}
}

void CamGuiManager::DrawSimulationSection()
{
	auto& sim = m_app.getMillingSimulator();

	if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if(ImGui::SliderFloat("speed", &m_simSpeed, 1.0f, 100.0f, "%.0f"))
		{
			sim.setSpeed(m_simSpeed);
		}

		ImGui::Spacing();

		float availWidth = ImGui::GetContentRegionAvail().x;
		float spacing = ImGui::GetStyle().ItemSpacing.x;
		float btnWidth = (availWidth - spacing * 2.0f) / 3.0f;

		ImGui::BeginDisabled(!sim.hasLoadedPath());
		if (ImGui::Button("Start", ImVec2(btnWidth, 0.0f)))
		{
			sim.Start();
		}

		ImGui::SameLine();
		if (ImGui::Button("Stop", ImVec2(btnWidth, 0.0f)))
		{
			sim.Stop();
		}

		ImGui::SameLine();
		if (ImGui::Button("Restart", ImVec2(btnWidth, 0.0f)))
		{
			sim.Restart();
		}
		ImGui::EndDisabled();

		const auto& pos = sim.getCutterPosition();
		ImGui::Spacing();
		ImGui::TextDisabled("Segment: %zu / %zu", sim.getCurrentSegmentIndex(), sim.getTotalSegments());
		ImGui::TextDisabled("Mill Pos: X:%.2f Y:%.2f Z:%.2f", pos.x, pos.y, pos.z);
	}
}

void CamGuiManager::DrawCutterSection()
{
	if (ImGui::CollapsingHeader("Cutter", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::InputFloat("size", &m_cutterSize, 0.1f, 0.5f, "%.2f");
		ImGui::InputFloat("height", &m_cutterHeight, 0.5f, 1.0f, "%.1f");
		ImGui::Checkbox("flat", &m_cutterFlat);
	}
}
