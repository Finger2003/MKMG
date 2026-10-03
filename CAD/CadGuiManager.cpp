#include "pch.h"
#include "CadGuiManager.h"
#include "CadApplication.h"
#include "GeometryMath.h"
#include "../ImGuiLib/imgui.h"
#include "../ImGuiLib/imgui_impl_win32.h"
#include "../ImGuiLib/imgui_impl_dx11.h"

namespace
{
	inline bool ContainsCaseInsensitive(const std::string& str, const std::string& substr)
	{
		auto it = std::search(
			str.begin(), str.end(),
			substr.begin(), substr.end(),
			[](char ch1, char ch2) { return std::tolower(ch1) == std::tolower(ch2); }
		);
		return (it != str.end());
	}
}

void CadGuiManager::DrawMenu()
{
	ImGui_ImplDX11_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	DrawSidebarPanel();
	DrawBoxSelectionOverlay();
	DrawIntersectionDialog();
	DrawTrimTextureOverlay();

	ImGui::Render();
}

void CadGuiManager::DrawSidebarPanel()
{
	ImGui::SetNextWindowPos(ImVec2(m_app.m_renderSize.cx, 0.0f));
	ImGui::SetNextWindowSize(ImVec2(m_app.cMenuWidth, m_app.m_renderSize.cy));

	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoCollapse;

	ImGui::Begin("Menu", nullptr);

	DrawActionCombo();
	ImGui::Separator();

	int selectedPoints = 0;
	int selectedCount = 0;
	int selectedCurves = 0;
	int selectedSurfaces = 0;
	int selectedIntersections = 0;
	std::weak_ptr<Curve> selectedCurve;
	std::weak_ptr<Surface> selectedSurface;
	std::weak_ptr<Intersection> selectedIntersection;

	for (const auto& obj : m_app.m_sceneObjects)
	{
		if (obj->selected)
		{
			selectedCount++;
			if (obj->type == ObjectType::Point)
				selectedPoints++;
			else if (auto sharedCurve = obj->AsShared<Curve>())
			{
				selectedCurves++;
				selectedCurve = sharedCurve;
				sharedCurve->CleanExpiredPoints();
			}
			else if (auto sharedSurface = obj->AsShared<Surface>())
			{
				selectedSurfaces++;
				selectedSurface = sharedSurface;
			}
			else if (auto sharedIntersection = obj->AsShared<Intersection>())
			{
				selectedIntersections++;
				selectedIntersection = sharedIntersection;
			}
		}
	}

	if (selectedCurves != 1)
		selectedCurve.reset();

	if (m_app.m_menuState == MenuState::List)
	{
		DrawListMenu(selectedCount, selectedPoints, selectedCurve.lock());

		if (auto curve = selectedCurve.lock())
		{
			if (curve->selected)
				DrawCurveList(curve.get(), selectedCount);
		}

		if (auto surface = selectedSurface.lock())
		{
			if (surface->selected && selectedSurfaces == 1)
				DrawSurfaceList(surface.get(), selectedCount);
		}

		if (auto intersection = selectedIntersection.lock())
		{
			if (intersection->selected && selectedIntersections == 1)
				DrawIntersectionList(intersection);
			else
				m_previewIntersection.reset();
		}
		else
			m_previewIntersection.reset();
	}
	else if (m_app.m_menuState == MenuState::Edit)
		DrawEditMenu();
	else if (m_app.m_menuState == MenuState::EditGroup)
		DrawEditGroupMenu(selectedCount);

	DrawCameraSettingsMenu();
	DrawCursorSettingsMenu();

	ImGui::End();
}

void CadGuiManager::DrawBoxSelectionOverlay()
{
	if (!m_app.m_isBoxSelecting)
		return;

	ImDrawList* drawList = ImGui::GetForegroundDrawList();
	auto [minX, maxX] = std::minmax(m_app.m_boxSelectStart.x, m_app.m_boxSelectCurrent.x);
	auto [minY, maxY] = std::minmax(m_app.m_boxSelectStart.y, m_app.m_boxSelectCurrent.y);
	ImVec2 p_min(static_cast<float>(minX), static_cast<float>(minY));
	ImVec2 p_max(static_cast<float>(maxX), static_cast<float>(maxY));
	bool ctrlHeld = (GetAsyncKeyState(VK_CONTROL) & 0x8000) != 0;
	bool shiftHeld = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;

	ImU32 fillCol = IM_COL32(0, 130, 255, 40);   // Default Blue
	ImU32 borderCol = IM_COL32(0, 130, 255, 220);

	if (ctrlHeld)
	{
		fillCol = IM_COL32(255, 50, 50, 40);   // Subtract Red
		borderCol = IM_COL32(255, 50, 50, 220);
	}
	else if (shiftHeld)
	{
		fillCol = IM_COL32(50, 220, 50, 40);   // Append Green
		borderCol = IM_COL32(50, 220, 50, 220);
	}

	drawList->AddRectFilled(p_min, p_max, fillCol);
	drawList->AddRect(p_min, p_max, borderCol);
}

void CadGuiManager::DrawIntersectionDialog()
{
	if (!m_showIntersectionPopup)
		return;

	ImGui::Begin("Intersection Parameters", &m_showIntersectionPopup, ImGuiWindowFlags_AlwaysAutoResize);

	ImGui::Checkbox("Use cursor as hint", &m_useCursorAsHint);
	ImGui::InputFloat("d", &m_intersectionStep);
	ImGui::Spacing();

	if (ImGui::Button("OK", ImVec2(120, 0)))
	{
		m_showIntersectionPopup = false;
		m_app.ActionFindIntersection();
	}
	ImGui::SameLine();
	if (ImGui::Button("Cancel", ImVec2(120, 0)))
		m_showIntersectionPopup = false;

	ImGui::End();
}

void CadGuiManager::DrawTrimTextureOverlay()
{
	auto previewInt = m_previewIntersection.lock();
	if (!previewInt)
		return;

	void* texToDraw = nullptr;
	float aspect = 1.0f;

	if (m_previewSurfaceIndex == 1 && previewInt->m_trimTexture1.texture)
	{
		texToDraw = (void*)previewInt->m_trimTexture1.srv.Get();
		aspect = previewInt->m_maxDomains.x / previewInt->m_maxDomains.y;
	}
	else if (m_previewSurfaceIndex == 2 && previewInt->m_trimTexture2.texture)
	{
		texToDraw = (void*)previewInt->m_trimTexture2.srv.Get();
		aspect = previewInt->m_maxDomains.z / previewInt->m_maxDomains.w;
	}

	if (texToDraw)
	{
		ImDrawList* bgDrawList = ImGui::GetBackgroundDrawList();

		float padding = 40.0f;
		float availableW = m_app.m_renderSize.cx - padding * 2.0f;
		float availableH = m_app.m_renderSize.cy - padding * 2.0f;

		float drawW = availableW;
		float drawH = availableW / aspect;

		if (drawH > availableH)
		{
			drawH = availableH;
			drawW = availableH * aspect;
		}

		float offsetX = (m_app.m_renderSize.cx - drawW) * 0.5f;
		float offsetY = (m_app.m_renderSize.cy - drawH) * 0.5f;

		ImVec2 pMin(offsetX, offsetY);
		ImVec2 pMax(offsetX + drawW, offsetY + drawH);

		bgDrawList->AddRectFilled(pMin, pMax, IM_COL32(255, 255, 255, 255));
		bgDrawList->AddImage(texToDraw, pMin, pMax);

		bgDrawList->AddRect(pMin, pMax, IM_COL32(100, 100, 100, 255), 0.0f, 0, 2.0f);
	}
}

void CadGuiManager::DrawObjectCreationSection(int selectedPoints, std::shared_ptr<Curve> activeCurve)
{
	if (ImGui::Button("Add Torus"))
	{
		m_app.AddTorusAtCursor();
	}

	if (ImGui::Button("Add Point"))
	{
		m_app.AddPointAtCursor(activeCurve);
	}

	ImGui::BeginDisabled(selectedPoints == 0);
	if (ImGui::Button("Add Bezier Curve"))
		m_app.CreateCurveFromSelectedPoints<BezierCurve>();
	if (ImGui::Button("Add B-Spline (C2) Curve"))
		m_app.CreateCurveFromSelectedPoints<BSplineCurve>();
	if (ImGui::Button("Add Interpolating Spline (C2)"))
		m_app.CreateCurveFromSelectedPoints<InterpolatingCurve>();
	ImGui::EndDisabled();

	std::vector<std::shared_ptr<Point>> pointsToAdd;
	if (activeCurve)
	{
		for (const auto& obj : m_app.m_sceneObjects)
		{
			if (obj->selected && obj->type == ObjectType::Point)
			{
				auto pt = std::static_pointer_cast<Point>(obj);
				bool exists = false;

				for (const auto& cpWeak : activeCurve->m_controlPoints)
				{
					if (cpWeak.lock() == pt)
					{
						exists = true;
						break;
					}
				}

				if (!exists)
					pointsToAdd.push_back(pt);
			}
		}
	}

	ImGui::BeginDisabled(pointsToAdd.empty() || !activeCurve);
	if (ImGui::Button("Add Points to Curve"))
	{
		activeCurve->m_controlPoints.insert(activeCurve->m_controlPoints.end(), pointsToAdd.begin(), pointsToAdd.end());
		for (const auto& pt : pointsToAdd)
			pt->AddDependent(activeCurve);

		activeCurve->MarkDirty();
	}
	ImGui::EndDisabled();
}

void CadGuiManager::DrawSurfaceCreationDialog()
{
	bool changed = false;
	if (ImGui::Button("Create C0 Surface"))
	{
		m_previewType = 0;
		changed = true;
		m_showSurfacePopup = true;
	}
	if (ImGui::Button("Create C2 Surface"))
	{
		m_previewType = 1;
		changed = true;
		m_showSurfacePopup = true;
	}

	if (!m_showSurfacePopup)
		return;

	const char* popupTitle = (m_previewType == 0) ? "C0 Surface Parameters" : "C2 Surface Parameters";
	ImGui::Begin(popupTitle, &m_showSurfacePopup);

	changed |= ImGui::RadioButton("Flat", &m_previewShape, 0);
	ImGui::SameLine();
	changed |= ImGui::RadioButton("Cylinder", &m_previewShape, 1);

	constexpr int MAX_SEGMENTS = 50;
	int minSegU = (m_previewShape == 1) ? 3 : 1;

	int oldU = m_previewSegU;
	int oldV = m_previewSegV;

	changed |= ImGui::SliderInt("Segments U", &m_previewSegU, minSegU, 10);
	changed |= ImGui::SliderInt("Segments V", &m_previewSegV, 1, 10);

	m_previewSegU = std::clamp(m_previewSegU, minSegU, MAX_SEGMENTS);
	m_previewSegV = std::clamp(m_previewSegV, 1, MAX_SEGMENTS);
	changed |= (m_previewSegU != oldU || m_previewSegV != oldV);

	if (m_previewShape == 0)
	{
		changed |= ImGui::DragFloat("Width", &m_previewWidth, 0.1f, 0.1f, 100.0f);
		changed |= ImGui::DragFloat("Length", &m_previewDim2, 0.1f, 0.1f, 100.0f);
	}
	else
	{
		changed |= ImGui::DragFloat("Radius", &m_previewRadius, 0.1f, 0.1f, 100.0f);
		changed |= ImGui::DragFloat("Height", &m_previewDim2, 0.1f, 0.1f, 100.0f);
	}

	if (changed)
	{
		m_app.m_surfaceBuilder.UpdateGeometry(
			m_app.m_device,
			(m_previewType == 0) ? SurfaceType::C0 : SurfaceType::C2,
			static_cast<SurfaceShape>(m_previewShape),
			m_previewSegU,
			m_previewSegV,
			(m_previewShape == 0) ? m_previewWidth : m_previewRadius,
			m_previewDim2,
			m_app.m_cursorPosition
		);
	}

	if (ImGui::Button("Add to Scene"))
	{
		m_app.AddSurface();
		m_showSurfacePopup = false;
	}
	ImGui::End();
}

void CadGuiManager::DrawObjectListSection()
{
	ImGui::InputText("##Search", m_filterBuffer, IM_ARRAYSIZE(m_filterBuffer));
	ImGui::SameLine();
	if (ImGui::Button("Clear"))
		m_filterBuffer[0] = '\0';
	std::string filterStr(m_filterBuffer);

	const ImGuiIO& io = ImGui::GetIO();
	ImGui::BeginChild("##ObjectListRegion", ImVec2(0, 250), true);
	for (int i = 0; i < m_app.m_sceneObjects.size(); i++)
	{
		auto& obj = m_app.m_sceneObjects[i];
		if (!filterStr.empty() && !ContainsCaseInsensitive(obj->name, filterStr))
			continue;
		ImGui::PushID(i);

		if (m_nameEditingIndex == i)
		{
			ImGui::SetKeyboardFocusHere();
			if (ImGui::InputText("##rename", m_renameBuffer, IM_ARRAYSIZE(m_renameBuffer),
				ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll))
			{

				obj->name = std::string(m_renameBuffer);
				m_nameEditingIndex = std::nullopt;
			}

			if (!ImGui::IsItemActive() && ImGui::IsMouseClicked(0))
			{
				m_nameEditingIndex = std::nullopt;
			}
		}
		else
		{
			if (ImGui::Selectable(obj->name.c_str(), obj->selected))
			{
				m_app.HandleObjectSelection(i, io.KeyCtrl, io.KeyShift);
			}

			if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0))
			{
				m_nameEditingIndex = i;
				strncpy_s(m_renameBuffer, obj->name.c_str(), sizeof(m_renameBuffer) - 1);
			}
		}

		ImGui::PopID();
	}
	ImGui::EndChild();
}

void CadGuiManager::DrawBatchActionSection(int selectedCount)
{
	ImGui::BeginDisabled(selectedCount == 0);
	if (selectedCount == 1)
	{
		if (ImGui::Button("Edit Selected", ImVec2(-1, 0)))
			m_app.m_menuState = MenuState::Edit;
	}
	else
	{
		if (ImGui::Button("Edit Group", ImVec2(-1, 0)))
			m_app.m_menuState = MenuState::EditGroup;
	}
	ImGui::EndDisabled();

	if (ImGui::Button("Select All", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		for (auto& obj : m_app.m_sceneObjects)
			obj->selected = true;
		m_app.m_selectionDirty = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Deselect All", ImVec2(-1, 0)))
	{
		m_app.ClearSelection();
	}

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

	if (ImGui::Button("Delete Selected", ImVec2(-1, 0)))
	{
		m_app.DeleteSelectedObjects();
	}

	ImGui::PopStyleColor(2);

	if (ImGui::Checkbox("Show Bernstein", &m_app.m_showBernsteinPoints))
	{
		m_app.m_enableVirtualEdit = false;
		m_app.m_activeVirtualEdit = std::nullopt;
		m_app.m_isEditing = false;
	}

	ImGui::BeginDisabled(!m_app.m_showBernsteinPoints);
	if (ImGui::Checkbox("Enable Virtual Edit", &m_app.m_enableVirtualEdit))
	{
		m_app.m_activeVirtualEdit = std::nullopt;
		m_app.m_isEditing = false;
	}
	ImGui::EndDisabled();
}

void CadGuiManager::DrawActionCombo()
{
	ImGui::Text("Interactive Action");
	const char* actions[] = {
		"None", "Free Translation", "Translate X", "Translate Y", "Translate Z",
		"Free Arcball", "Rotate X", "Rotate Y", "Rotate Z", "Scale", "Scale X", "Scale Y", "Scale Z"
	};
	int actionIndex = static_cast<int>(m_app.m_currentEditAction);

	if (ImGui::Combo("##ObjectAction", &actionIndex, actions, IM_ARRAYSIZE(actions)))
	{
		m_app.m_currentEditAction = static_cast<EditAction>(actionIndex);
	}
}

void CadGuiManager::DrawListMenu(int selectedCount, int selectedPoints, std::shared_ptr<Curve> activeCurve)
{
	DrawObjectCreationSection(selectedPoints, activeCurve);
	DrawSurfaceCreationDialog();

	ImGui::Separator();
	DrawObjectListSection();

	ImGui::Separator();
	DrawBatchActionSection(selectedCount);
}

void CadGuiManager::DrawCurveList(Curve* curve, int selectedCount)
{
	ImGui::TextDisabled("Selected Curve Control Points:");
	ImGui::Text("%s", curve->name.c_str());

	if (ImGui::Button("Select All Points", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		for (const auto& cpWeak : curve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = true;
		}
		m_app.m_selectionDirty = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Deselect All Points", ImVec2(-1, 0)))
	{
		for (const auto& cpWeak : curve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = false;
		}
		m_app.m_selectionDirty = true;
	}

	const ImGuiIO& io = ImGui::GetIO();
	if (ImGui::BeginListBox(("##PointsList_" + curve->name).c_str(), ImVec2(-1.0f, 0.0f)))
	{
		for (size_t i = 0; i < curve->m_controlPoints.size(); i++)
		{
			if (auto cp = curve->m_controlPoints[i].lock())
			{
				ImGui::PushID(static_cast<int>(i));
				std::string label = std::to_string(i) + ": " + cp->name;

				if (ImGui::Selectable(label.c_str(), cp->selected))
				{
					m_app.HandleCurveListSelection(curve, i, io.KeyCtrl, io.KeyShift);
				}
				ImGui::PopID();
			}
		}
		ImGui::EndListBox();
	}

	bool pointSelected = false;
	for (const auto& cpWeak : curve->m_controlPoints)
	{
		if (auto cp = cpWeak.lock())
		{
			if (cp->selected)
			{
				pointSelected = true;
				break;
			}
		}
	}
	ImGui::BeginDisabled(!pointSelected);
	if (ImGui::Button("Remove Selected Points from Curve", ImVec2(-1, 0)))
	{
		std::erase_if(curve->m_controlPoints, [curve](const auto& cpWeak) {
			if (auto cp = cpWeak.lock())
			{
				if (cp->selected)
				{
					cp->selected = false;
					cp->RemoveDependent(curve);
					return true;
				}
				return false;
			}
			return true;
			});
		curve->MarkDirty();
		m_app.m_selectionDirty = true;
	}
	ImGui::EndDisabled();
	ImGui::Separator();

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

	if (ImGui::Button("Delete Curve Only", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		std::erase_if(m_app.m_sceneObjects, [curve](const auto& obj) {
			return obj.get() == curve;
			});
	}

	ImGui::SameLine();
	if (ImGui::Button("Delete Curve and Control Points", ImVec2(-1, 0)))
	{
		std::vector<Point*> pointsToDelete;
		for (const auto& cpWeak : curve->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				pointsToDelete.push_back(cp.get());
		}

		std::erase_if(m_app.m_sceneObjects, [curve, &pointsToDelete](const auto& obj) {
			if (obj.get() == curve)
				return true;
			return std::find(pointsToDelete.begin(), pointsToDelete.end(), obj.get()) != pointsToDelete.end();
			});
	}
	ImGui::PopStyleColor(2);
}

void CadGuiManager::DrawSurfaceList(Surface* surface, int selectedCount)
{
	ImGui::TextDisabled("Selected Surface Control Points:");
	ImGui::Text("%s", surface->name.c_str());

	if (ImGui::Button("Select All Points", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = true;
		}
		m_app.m_selectionDirty = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Deselect All Points", ImVec2(-1, 0)))
	{
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->selected = false;
		}
		m_app.m_selectionDirty = true;
	}

	const ImGuiIO& io = ImGui::GetIO();
	if (ImGui::BeginListBox(("##SurfacePointsList_" + surface->name).c_str(), ImVec2(-1.0f, 0.0f)))
	{
		unsigned int pointsU = surface->m_gridPointsU;
		for (size_t i = 0; i < surface->m_controlPoints.size(); i++)
		{
			if (auto cp = surface->m_controlPoints[i].lock())
			{
				ImGui::PushID(static_cast<int>(i));

				int u = i % pointsU;
				int v = i / pointsU;
				std::string label = "[" + std::to_string(u) + ", " + std::to_string(v) + "] " + cp->name;

				if (ImGui::Selectable(label.c_str(), cp->selected))
				{
					m_app.HandleSurfaceListSelection(surface, i, io.KeyCtrl, io.KeyShift);
				}
				ImGui::PopID();
			}
		}
		ImGui::EndListBox();
	}

	ImGui::Separator();

	ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.1f, 0.1f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));

	if (ImGui::Button("Delete Surface Only", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0)))
	{
		// Unlock the points so the user can delete them later
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				cp->m_surfaceLockCount = std::max(0, cp->m_surfaceLockCount - 1);
		}

		// Delete just the surface object
		std::erase_if(m_app.m_sceneObjects, [surface](const auto& obj) {
			return obj.get() == surface;
			});

		m_app.ClearSelection();
	}

	ImGui::SameLine();
	if (ImGui::Button("Delete Surface and Control Points", ImVec2(-1, 0)))
	{
		std::vector<Point*> pointsToDelete;
		for (const auto& cpWeak : surface->m_controlPoints)
		{
			if (auto cp = cpWeak.lock())
				pointsToDelete.push_back(cp.get());
		}

		// delete the surface and the points
		std::erase_if(m_app.m_sceneObjects, [surface, &pointsToDelete](const auto& obj) {
			if (obj.get() == surface)
				return true;
			return std::find(pointsToDelete.begin(), pointsToDelete.end(), obj.get()) != pointsToDelete.end();
			});

		m_app.ClearSelection();
	}
	ImGui::PopStyleColor(2);
}

void CadGuiManager::DrawEditMenu()
{
	if (!m_app.m_lastClickedIndex || *m_app.m_lastClickedIndex >= m_app.m_sceneObjects.size())
	{
		m_app.m_menuState = MenuState::List;
	}
	else
	{
		auto& selectedObj = m_app.m_sceneObjects[*m_app.m_lastClickedIndex];
		if (ImGui::Button("< Back to List"))
		{
			m_app.m_menuState = MenuState::List;
		}
		ImGui::Separator();

		ImGui::Text("Editing: %s", selectedObj->name.c_str());
		ImGui::Spacing();

		if (auto point = selectedObj->As<Point>())
			DrawPointMenu(*point);
		else if (auto torus = selectedObj->As<Torus>())
			DrawTorusMenu(*torus);
		else if (auto surface = selectedObj->As<Surface>())
			DrawSurfaceMenu(*surface);
		else if (auto patch = selectedObj->As<GregoryPatch>())
			DrawPatchMenu(*patch);
	}
}

void CadGuiManager::DrawTorusMenu(Torus& torus)
{
	bool transformChanged = false;

	// --- Geometry Settings ---
	float tempMajor = torus.GetMajorRadius();
	float tempMinor = torus.GetMinorRadius();
	int tempSegs[2] = { torus.GetMajorSegments(), torus.GetMinorSegments() };

	ImGui::Text("Geometry");
	if (ImGui::SliderFloat("Major Radius", &tempMajor, Torus::cMinMajorRadius, Torus::cMaxMajorRadius))
		torus.SetMajorRadius(tempMajor);
	if (ImGui::SliderFloat("Minor Radius", &tempMinor, Torus::cMinMinorRadius, Torus::cMaxMinorRadius))
		torus.SetMinorRadius(tempMinor);
	if (ImGui::SliderInt2("Segments", tempSegs, Torus::cMinMajorSegments, Torus::cMaxMajorSegments))
		torus.SetSegments(tempSegs[0], tempSegs[1]);

	ImGui::Separator();

	if (ImGui::DragFloat3("Position", &torus.m_position.x, 0.01f))
	{
		transformChanged = true;
		m_app.m_selectionDirty = true;
	}

	MathLib::Vec3f eulerDegrees = torus.m_eulerAngles.ToVec3f() * (180.0f / std::numbers::pi_v<float>);
	if (ImGui::DragFloat3("Rotation", eulerDegrees.f, 1.0f, 0.0f, 0.0f, "%.2f"))
	{
		eulerDegrees *= (std::numbers::pi_v<float> / 180.0f);
		torus.m_eulerAngles = eulerDegrees;
		MathLib::Mat4f rotX = MathLib::Mat4f::RotationX(torus.m_eulerAngles.x);
		MathLib::Mat4f rotY = MathLib::Mat4f::RotationY(torus.m_eulerAngles.y);
		MathLib::Mat4f rotZ = MathLib::Mat4f::RotationZ(torus.m_eulerAngles.z);
		torus.m_rotationMatrix = rotZ * rotX * rotY;
		transformChanged = true;
	}
	if (ImGui::Button("Reset Rotation"))
	{
		torus.m_eulerAngles = { 0, 0, 0 };
		torus.m_rotationMatrix = MathLib::Mat4f::Identity();
		torus.m_baseRotationMatrix = MathLib::Mat4f::Identity();
		transformChanged = true;
	}
	ImGui::Separator();
	ImGui::Text("Scale");

	if (ImGui::DragFloat3("Scale XYZ", &torus.m_scale.x, 0.01f, Torus::cMinScale, Torus::cMaxScale))
		transformChanged = true;

	float uniformScale = torus.m_scale.x;
	if (ImGui::DragFloat("Uniform Scale", &uniformScale, 0.01f, Torus::cMinScale, Torus::cMaxScale))
	{
		torus.m_scale = { uniformScale, uniformScale, uniformScale };
		transformChanged = true;
	}

	if (transformChanged)
		torus.UpdateModelMatrix();
}

void CadGuiManager::DrawPointMenu(Point& selectedObj)
{
	if (ImGui::DragFloat3("Position", &selectedObj.m_position.x, 0.01f))
	{
		m_app.m_selectionDirty = true;
		selectedObj.NotifyDependents();
	}
}

void CadGuiManager::DrawSurfaceMenu(Surface& surface)
{
	ImGui::Text("Tessellation Parameters");
	ImGui::Spacing();
	// Sliders to control the shader density
	ImGui::SliderInt("Lines per Segment (U)", &surface.m_linesPerSegmentU, 2, 64);
	ImGui::SliderInt("Lines per Segment (V)", &surface.m_linesPerSegmentV, 2, 64);
	ImGui::SliderInt("Smoothness", &surface.m_smoothness, 2, 64);
	ImGui::Separator();
}

void CadGuiManager::DrawPatchMenu(GregoryPatch& patch)
{
	ImGui::Text("Tessellation Parameters");
	ImGui::Spacing();
	// Sliders to control the shader density
	ImGui::SliderInt("Lines per Segment (U)", &patch.m_linesPerSegmentU, 2, 64);
	ImGui::SliderInt("Lines per Segment (V)", &patch.m_linesPerSegmentV, 2, 64);
	ImGui::SliderInt("Smoothness", &patch.m_smoothness, 2, 64);
	ImGui::Separator();
}

void CadGuiManager::DrawEditGroupMenu(int selectedCount)
{
	if (ImGui::Button("< Back to List"))
	{
		m_app.m_menuState = MenuState::List;
	}

	ImGui::Separator();

	ImGui::Text("Editing Group (%d objects)", selectedCount);

	auto centerOpt = m_app.GetSelectionCenter();
	if (centerOpt)
	{
		ImGui::Text("Center: %.2f, %.2f, %.2f", centerOpt->x, centerOpt->y, centerOpt->z);
	}
	ImGui::Spacing();
}

void CadGuiManager::DrawCameraSettingsMenu()
{
	ImGui::Separator();
	ImGui::Text("Camera Settings");
	float tempFovDegrees = m_app.m_camera.GetFovY() * (180.0f / std::numbers::pi_v<float>);
	float tempNear = m_app.m_camera.GetNearPlane();
	float tempFar = m_app.m_camera.GetFarPlane();

	if (ImGui::SliderFloat("FOV", &tempFovDegrees, 30.0f, 120.0f))
		m_app.m_camera.SetFovY(tempFovDegrees * (std::numbers::pi_v<float> / 180.0f));

	if (ImGui::DragFloat("Near Plane", &tempNear, 0.01f, 0.001f, 10.0f) ||
		ImGui::DragFloat("Far Plane", &tempFar, 0.1f, 10.0f, 1000.0f))
		m_app.m_camera.SetPlanes(tempNear, tempFar);

	ImGui::Separator();
	ImGui::Text("Stereoscopy (Anaglyph 3D)");
	ImGui::Checkbox("Enable Stereoscopy", &m_app.m_enableStereo);

	ImGui::BeginDisabled(!m_app.m_enableStereo);
	ImGui::SliderFloat("Eye Separation", &m_app.m_eyeSeparation, 0.01f, 1.0f);
	ImGui::SliderFloat("Focal Distance", &m_app.m_focalLength, 0.1f, 100.0f);
	ImGui::ColorEdit3("Left Eye Tint", &m_app.m_leftEyeColor.x);
	ImGui::ColorEdit3("Right Eye Tint", &m_app.m_rightEyeColor.x);
	ImGui::EndDisabled();

	m_app.UpdateProjectionMatrix();
}

void CadGuiManager::DrawCursorSettingsMenu()
{
	ImGui::Separator();
	ImGui::Text("Cursor Settings");
	ImGui::DragFloat3("Cursor Position", &m_app.m_cursorPosition.x, 0.01f);
	MathLib::Vec4f worldPos = MathLib::Vec4f(m_app.m_cursorPosition.x, m_app.m_cursorPosition.y, m_app.m_cursorPosition.z, 1.0f);
	MathLib::Vec4f clipPos = m_app.m_camera.GetProjViewMatrix() * worldPos;
	int screenPos[2] = { 0 };
	if (std::abs(clipPos.w) > 0.0001f)
	{
		float ndcX = clipPos.x / clipPos.w;
		float ndcY = clipPos.y / clipPos.w;
		screenPos[0] = static_cast<int>(std::round((ndcX + 1.0f) * 0.5f * m_app.m_renderSize.cx));
		screenPos[1] = static_cast<int>(std::round((1.0f - ndcY) * 0.5f * m_app.m_renderSize.cy));
	}

	if (ImGui::DragInt2("Screen Position", screenPos))
	{
		screenPos[0] = std::clamp(screenPos[0], 0, static_cast<int>(m_app.m_renderSize.cx));
		screenPos[1] = std::clamp(screenPos[1], 0, static_cast<int>(m_app.m_renderSize.cy));
		auto [ndcX, ndcY] = GeometryMath::CalculateCoordsFromPixel(screenPos[0], screenPos[1], m_app.m_renderSize.cx, m_app.m_renderSize.cy);
		MathLib::Vec4f worldPos = m_app.m_cursorPosition.ToVec4f(1.0f);
		MathLib::Vec4f viewPos = m_app.m_camera.GetViewMatrix() * worldPos;
		float actualDepth = std::abs(viewPos.z);

		m_app.m_cursorPosition = m_app.m_camera.GetPositionAtDepth(ndcX, ndcY, actualDepth);
	}
}

void CadGuiManager::DrawIntersectionList(std::shared_ptr<Intersection> intersection)
{
	ImGui::Separator();
	ImGui::Text("Intersection Actions");

	auto getSrvPtr = [](std::weak_ptr<SceneObject> objWeak) -> Microsoft::WRL::ComPtr<ID3D11ShaderResourceView>*{
		if (auto obj = objWeak.lock())
		{
			if (auto surf = obj->As<Surface>()) return std::addressof(surf->m_trimTextureSRV);
			if (obj->type == ObjectType::Torus) return std::addressof(static_cast<Torus*>(obj.get())->m_trimTextureSRV);
		}
		return nullptr;
		};

	auto srv1Ptr = getSrvPtr(intersection->m_surface1);
	bool isTrimS1 = srv1Ptr && srv1Ptr->Get() == intersection->m_trimTexture1.srv.Get();
	if (ImGui::Button(isTrimS1 ? "Disable Trim S1" : "Enable Trim S1", ImVec2(-1, 0)))
	{
		if (srv1Ptr)
		{
			if (isTrimS1)
				srv1Ptr->Reset(); // Unbind texture
			else
				*srv1Ptr = intersection->m_trimTexture1.srv; // Bind texture
		}
	}

	auto srv2Ptr = getSrvPtr(intersection->m_surface2);
	bool isTrimS2 = srv2Ptr && srv2Ptr->Get() == intersection->m_trimTexture2.srv.Get();

	if (ImGui::Button(isTrimS2 ? "Disable Trim S2" : "Enable Trim S2", ImVec2(-1, 0)))
	{
		if (srv2Ptr)
		{
			if (isTrimS2)
				srv2Ptr->Reset(); // Unbind texture
			else
				*srv2Ptr = intersection->m_trimTexture2.srv; // Bind texture
		}
	}



	bool changed = false;
	if (ImGui::Button("Reverse S1 Trim", ImVec2(-1, 0)))
	{
		intersection->m_reverseTrimS1 = !intersection->m_reverseTrimS1;
		changed = true;
	}
	if (ImGui::Button("Reverse S2 Trim", ImVec2(-1, 0)))
	{
		intersection->m_reverseTrimS2 = !intersection->m_reverseTrimS2;
		changed = true;
	}

	if (changed)
		m_app.RenderTrimTextures(intersection.get(), m_app.m_device.getContext());

	bool isPreviewS1 = (m_previewIntersection.lock() == intersection && m_previewSurfaceIndex == 1);
	bool isPreviewS2 = (m_previewIntersection.lock() == intersection && m_previewSurfaceIndex == 2);

	if (ImGui::Button(isPreviewS1 ? "Close S1 Parametrization" : "S1 Parametrization", ImVec2(-1, 0)))
	{
		if (isPreviewS1)
		{
			m_previewIntersection.reset();
			m_previewSurfaceIndex = 0;
		}
		else
		{
			m_previewIntersection = intersection;
			m_previewSurfaceIndex = 1;
		}
	}

	if (ImGui::Button(isPreviewS2 ? "Close S2 Parametrization" : "S2 Parametrization", ImVec2(-1, 0)))
	{
		if (isPreviewS2)
		{
			m_previewIntersection.reset();
			m_previewSurfaceIndex = 0;
		}
		else
		{
			m_previewIntersection = intersection;
			m_previewSurfaceIndex = 2;
		}
	}
	if (intersection->type == ObjectType::LinearIntersection)
	{
		if (ImGui::Button("To Interpolation Bezier", ImVec2(-1, 0)))
		{
			auto bezInt = std::make_shared<BezierIntersection>(std::move(*std::static_pointer_cast<LinearIntersection>(intersection)));
			bezInt->InitBezierGeometry(m_app.m_device);

			auto it = std::find(m_app.m_sceneObjects.begin(), m_app.m_sceneObjects.end(), intersection);
			if (it != m_app.m_sceneObjects.end())
				*it = bezInt;

			bezInt->selected = true;
			if (m_previewIntersection.lock() == intersection)
				m_previewIntersection = bezInt;
		}
	}
}
