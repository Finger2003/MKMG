#include "pch.h"
#include "CamGuiManager.h"
#include "CadApplication.h"
#include "PathGenerator.h"
#include "Surface.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"
#include <algorithm>
#include <cctype>

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
	DrawPathGenerationSection();
	DrawMaterialSection();
	DrawSimulationSection();
	DrawCutterSection();

	ImGui::End();

	if (auto errOpt = m_app.getMillingSimulator().ConsumePendingError())
	{
		if (m_showErrorPopup)
		{
			const auto& err = *errOpt;
			std::ostringstream msg;
			msg << "Milling simulation was paused due to a milling error!\n\n"
				<< "Reason(s):\n" << err.errorDescription << "\n"
				<< "Original G-Code Line: " << err.lineNumber << "\n"
				<< std::fixed << std::setprecision(3)
				<< "Move From: X=" << err.moveStartGCode.x << " Y=" << err.moveStartGCode.y << " Z=" << err.moveStartGCode.z << "\n"
				<< "Move To:   X=" << err.moveEndGCode.x << " Y=" << err.moveEndGCode.y << " Z=" << err.moveEndGCode.z << "\n\n"
				<< "Error details saved to dump file:\n" << err.dumpFilePath;

			MessageBoxA(
				m_app.getWindowHandle(),
				msg.str().c_str(),
				"Milling Error Detected",
				MB_OK | MB_ICONERROR
			);
		}
	}
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
				m_cutterHeight = m_cutterSize * 2.0f;

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

void CamGuiManager::DrawPathGenerationSection()
{
	const auto& sceneObjects = m_app.getSceneObjects();
	if (sceneObjects.empty())
	{
		m_baseAutoDetected = false;
		m_baseSurface.reset();
	}

	auto selectedBase = m_baseSurface.lock();
	if (!m_baseAutoDetected && !sceneObjects.empty())
	{
		for (const auto& obj : sceneObjects)
		{
			if (auto surf = obj ? obj->AsShared<Surface>() : nullptr)
			{
				std::string lowerName = surf->name;
				std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(),
					[](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				if (lowerName.find("plane") != std::string::npos)
				{
					selectedBase = surf;
					m_baseSurface = surf;
					break;
				}
			}
		}
		m_baseAutoDetected = true;
	}

	if (ImGui::CollapsingHeader("Roughing Generation (K16)", ImGuiTreeNodeFlags_DefaultOpen))
	{
		const char* previewName = selectedBase ? selectedBase->name.c_str() : "(None - use base height)";
		if (ImGui::BeginCombo("base surface##rough", previewName))
		{
			bool noneSelected = (selectedBase == nullptr);
			if (ImGui::Selectable("(None - use base height)", noneSelected))
			{
				m_baseSurface.reset();
				selectedBase = nullptr;
			}
			if (noneSelected)
				ImGui::SetItemDefaultFocus();

			for (const auto& obj : sceneObjects)
			{
				auto surf = obj ? obj->AsShared<Surface>() : nullptr;
				if (!surf)
					continue;

				bool isSelected = (selectedBase == surf);
				std::string label = surf->name + "##" + std::to_string(surf->m_id);
				if (ImGui::Selectable(label.c_str(), isSelected))
				{
					m_baseSurface = surf;
					selectedBase = surf;
				}
				if (isSelected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		ImGui::InputFloat("stepover [mm]##rough", &m_roughStepover, 0.5f, 1.0f, "%.2f");
		ImGui::InputFloat("surface margin [mm]##rough", &m_roughSurfaceMargin, 0.1f, 0.5f, "%.2f");
		ImGui::InputFloat("base margin [mm]##rough", &m_roughBaseMargin, 0.1f, 0.5f, "%.2f");
		ImGui::InputFloat("layer 1 height [mm]##rough", &m_roughLayer1Height, 0.5f, 1.0f, "%.1f");

		m_roughStepover = std::clamp(m_roughStepover, 0.5f, 15.0f);
		m_roughSurfaceMargin = std::max(0.0f, m_roughSurfaceMargin);
		m_roughBaseMargin = std::max(0.1f, m_roughBaseMargin);
		m_roughLayer1Height = std::clamp(m_roughLayer1Height, m_simBaseHeight + 2.0f, m_simSizeY * 10.0f - 1.0f);

		ImGui::Spacing();
		if (ImGui::Button("Generate Roughing Path (1.k16)", ImVec2(-1.0f, 0.0f)))
		{
			const std::vector<COMDLG_FILTERSPEC> saveFilters = {
				{ L"K16 Ball-Nose Path (*.k16)", L"*.k16" },
				{ L"All Files (*.*)", L"*.*" }
			};
			std::wstring widePath = m_app.ShowFileDialog<IFileSaveDialog>(saveFilters, L"k16", L"1.k16");
			if (!widePath.empty())
			{
				std::string savePath(widePath.begin(), widePath.end());

				RoughingParameters params{};
				params.cutterDiameter = 16.0f;
				params.cutterFluteHeight = 32.0f;
				params.blockSizeX = m_simSizeX * 10.0f;
				params.blockSizeY = m_simSizeY * 10.0f;
				params.blockSizeZ = m_simSizeZ * 10.0f;
				params.baseHeight = m_simBaseHeight;
				params.gridDivX = std::max(2, m_simDivX);
				params.gridDivZ = std::max(2, m_simDivZ);
				params.surfaceMargin = m_roughSurfaceMargin;
				params.baseMargin = m_roughBaseMargin;
				params.layer1Height = m_roughLayer1Height;
				params.stepover = m_roughStepover;
				params.baseSurface = selectedBase.get();

				auto waypoints = PathGenerator::GenerateRoughingPath(sceneObjects, params);
				if (!PathGenerator::ExportGCode(savePath, waypoints))
				{
					MessageBoxA(
						m_app.getWindowHandle(),
						"Failed to export the generated roughing path to the selected file.",
						"Export Error",
						MB_OK | MB_ICONERROR
					);
				}
				else if (m_app.getMillingSimulator().LoadGCode(savePath))
				{
					size_t lastSlash = savePath.find_last_of("\\/");
					m_loadedGCodeFile = (lastSlash != std::string::npos)
						? savePath.substr(lastSlash + 1)
						: savePath;
					m_loadedGCodePath = std::move(savePath);

					m_cutterFlat = false;
					m_cutterSize = 16.0f;
					m_cutterHeight = 32.0f;
					m_app.getMillingSimulator().setCutterParams({ m_cutterSize, m_cutterHeight, m_cutterFlat });
				}
			}
		}
	}

	if (ImGui::CollapsingHeader("Flat Milling Generation (F10)", ImGuiTreeNodeFlags_DefaultOpen))
	{
		const char* previewName = selectedBase ? selectedBase->name.c_str() : "(Select base surface)";
		if (ImGui::BeginCombo("base surface##flat", previewName))
		{
			for (const auto& obj : sceneObjects)
			{
				auto surf = obj ? obj->AsShared<Surface>() : nullptr;
				if (!surf)
					continue;

				bool isSelected = (selectedBase == surf);
				std::string label = surf->name + "##flat_" + std::to_string(surf->m_id);
				if (ImGui::Selectable(label.c_str(), isSelected))
				{
					m_baseSurface = surf;
					selectedBase = surf;
				}
				if (isSelected)
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		ImGui::InputFloat("stepover [mm]##flat", &m_flatStepover, 0.5f, 1.0f, "%.2f");
		ImGui::InputFloat("zigzag margin [mm]##flat", &m_flatZigzagMargin, 0.1f, 0.5f, "%.2f");

		m_flatStepover = std::clamp(m_flatStepover, 0.5f, 9.5f);
		m_flatZigzagMargin = std::clamp(m_flatZigzagMargin, 0.1f, 5.0f);

		ImGui::Spacing();
		if (selectedBase == nullptr)
			ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "Select a base surface first.");

		ImGui::BeginDisabled(selectedBase == nullptr);
		if (ImGui::Button("Generate Flat Path (2.f10)", ImVec2(-1.0f, 0.0f)))
		{
			const std::vector<COMDLG_FILTERSPEC> saveFilters = {
				{ L"F10 Flat-End Path (*.f10)", L"*.f10" },
				{ L"All Files (*.*)", L"*.*" }
			};
			std::wstring widePath = m_app.ShowFileDialog<IFileSaveDialog>(saveFilters, L"f10", L"2.f10");
			if (!widePath.empty())
			{
				std::string savePath(widePath.begin(), widePath.end());

				FlatMillingParameters params{};
				params.cutterDiameter = 10.0f;
				params.cutterFluteHeight = 20.0f;
				params.blockSizeX = m_simSizeX * 10.0f;
				params.blockSizeY = m_simSizeY * 10.0f;
				params.blockSizeZ = m_simSizeZ * 10.0f;
				params.baseHeight = m_simBaseHeight;
				params.stepover = m_flatStepover;
				params.zigzagMargin = m_flatZigzagMargin;
				params.baseSurface = selectedBase.get();

				auto waypoints = PathGenerator::GenerateFlatPath(sceneObjects, params);
				if (!PathGenerator::ExportGCode(savePath, waypoints))
				{
					MessageBoxA(
						m_app.getWindowHandle(),
						"Failed to export the generated flat milling path to the selected file.",
						"Export Error",
						MB_OK | MB_ICONERROR
					);
				}
				else if (m_app.getMillingSimulator().LoadGCode(savePath))
				{
					size_t lastSlash = savePath.find_last_of("\\/");
					m_loadedGCodeFile = (lastSlash != std::string::npos)
						? savePath.substr(lastSlash + 1)
						: savePath;
					m_loadedGCodePath = std::move(savePath);

					m_cutterFlat = true;
					m_cutterSize = 10.0f;
					m_cutterHeight = 20.0f;
					m_app.getMillingSimulator().setCutterParams({ m_cutterSize, m_cutterHeight, m_cutterFlat });
				}
			}
		}
		ImGui::EndDisabled();
	}
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
		ImGui::InputFloat("base height [mm]", &m_simBaseHeight, 0.5f, 1.0f, "%.1f");

		ImGui::Spacing();
		if (ImGui::Button("Reset / Apply Material", ImVec2(-1.0f, 0.0f)))
		{
			BlockParameters blockParams;
			blockParams.divX = m_simDivX;
			blockParams.divZ = m_simDivZ;
			blockParams.sizeX = m_simSizeX * 10.0f; // cm -> mm
			blockParams.sizeY = m_simSizeY * 10.0f; // cm -> mm
			blockParams.sizeZ = m_simSizeZ * 10.0f; // cm -> mm
			blockParams.baseHeight = m_simBaseHeight;
			m_app.getMillingSimulator().InitBlock(blockParams);
		}
	}
}

void CamGuiManager::DrawSimulationSection()
{
	auto& sim = m_app.getMillingSimulator();

	if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::SliderFloat("speed", &m_simSpeed, 1.0f, 1000.0f, "%.0f"))
			sim.setSpeed(m_simSpeed);

		ImGui::Spacing();
		ImGui::Checkbox("Show milling path", &m_showMillingPath);
		if (ImGui::Checkbox("Show error popups & Pause", &m_showErrorPopup))
			sim.setPauseOnError(m_showErrorPopup);
		ImGui::Spacing();

		float availWidth = ImGui::GetContentRegionAvail().x;
		float spacing = ImGui::GetStyle().ItemSpacing.x;
		float btnWidth = (availWidth - spacing * 2.0f) / 2.0f;

		ImGui::BeginDisabled(!sim.hasLoadedPath());

		if (ImGui::Button("Start", ImVec2(btnWidth, 0.0f)))
			sim.Start();

		ImGui::SameLine();
		if (ImGui::Button("Stop", ImVec2(btnWidth, 0.0f)))
			sim.Stop();

		if (ImGui::Button("Restart", ImVec2(btnWidth, 0.0f)))
			sim.Restart();

		ImGui::SameLine();
		if (ImGui::Button("Instant", ImVec2(btnWidth, 0.0f)))
			sim.InstantComplete();

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
		bool changed = false;
		changed |= ImGui::InputFloat("size", &m_cutterSize, 0.1f, 0.5f, "%.2f");
		changed |= ImGui::InputFloat("height", &m_cutterHeight, 0.5f, 1.0f, "%.1f");
		changed |= ImGui::Checkbox("flat", &m_cutterFlat);

		if (changed)
			m_app.getMillingSimulator().setCutterParams({ m_cutterSize, m_cutterHeight, m_cutterFlat });
	}
}
