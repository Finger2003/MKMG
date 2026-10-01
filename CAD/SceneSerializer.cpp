#include "pch.h"
#include "SceneSerializer.h"

#include "Point.h"
#include "BezierCurve.h"
#include "BSplineCurve.h"
#include "InterpolatingCurve.h"
#include "BezierSurface.h"
#include "BSplineSurface.h"
#include "Torus.h"
#include "DxDevice.h" 
#include "../MathLib/Mat4f.h"

#include <nlohmann/json.hpp>
using namespace nlohmann;
using namespace MathLib;

namespace
{
	unsigned int ExtractIndexFromName(const std::string& name)
	{
		size_t lastDigitIdx = name.find_last_of("0123456789");
		if (lastDigitIdx == std::string::npos)
			return 0;

		size_t firstDigitIdx = lastDigitIdx;
		while (firstDigitIdx > 0 && std::isdigit(static_cast<unsigned char>(name[firstDigitIdx - 1])))
			firstDigitIdx--;

		std::string numStr = name.substr(firstDigitIdx, (lastDigitIdx - firstDigitIdx) + 1);
		try
		{
			return static_cast<unsigned int>(std::stoi(numStr));
		}
		catch (...)
		{
			return 0;
		}
	}
}

void SceneSerializer::Save(const std::wstring& filePath, const std::vector<std::shared_ptr<SceneObject>>& objects)
{
	json rootObject = json::object();
	json pointsArray = json::array();
	json geometryArray = json::array();

	for (const auto& obj : objects)
	{
		if (obj->type == ObjectType::Point)
			pointsArray.push_back(obj->Serialize());
		else
			geometryArray.push_back(obj->Serialize());
	}

	rootObject["points"] = pointsArray;
	rootObject["geometry"] = geometryArray;

	std::ofstream file(filePath);
	if (!file.is_open())
		throw std::runtime_error("Could not open the file for saving. Please check file permissions or path.");
	
	file << rootObject.dump(4);
}

std::vector<std::shared_ptr<SceneObject>> SceneSerializer::Load(const std::wstring& filePath, DxDevice& device)
{
	std::ifstream file(filePath);
	if (!file.is_open())
		throw std::runtime_error("Could not open file.");

	json rootObject;
	file >> rootObject;

	std::vector<std::shared_ptr<SceneObject>> newSceneObjects;
	std::unordered_map<unsigned int, std::shared_ptr<Point>> pointMap;

	auto parseNameData = [](const nlohmann::json& jsonNode) -> std::optional<ParsedNameData> {
		if (jsonNode.contains("name"))
		{
			std::string n = jsonNode["name"];
			if (!n.empty())
			{
				unsigned int idx = ExtractIndexFromName(n);
				return ParsedNameData{ std::move(n), idx };
			}
		}
		return std::nullopt;
		};

	if (rootObject.contains("points"))
	{
		for (const auto& ptJson : rootObject["points"])
		{
			unsigned int id = ptJson["id"];
			float3 pos = ptJson["position"];
			std::optional<ParsedNameData> nameData = parseNameData(ptJson);

			auto pt = std::make_shared<Point>(id, pos, std::move(nameData));
			pointMap[id] = pt;
			newSceneObjects.push_back(pt);
		}
	}

	if (rootObject.contains("geometry"))
	{
		for (const auto& geomJson : rootObject["geometry"])
		{
			unsigned int id = geomJson["id"];
			std::string type = geomJson["objectType"];
			std::optional<ParsedNameData> nameData = parseNameData(geomJson);

			std::vector<std::weak_ptr<Point>> controlPoints;
			if (geomJson.contains("controlPoints"))
			{
				for (const auto& cpRef : geomJson["controlPoints"])
				{
					unsigned int cpId = cpRef["id"];
					if (pointMap.count(cpId))
						controlPoints.push_back(pointMap[cpId]);
				}
			}

			std::shared_ptr<SceneObject> newObject = nullptr;
			auto assignCurve = [&]<typename CurveType>()
			{
				auto newCurve = std::make_shared<CurveType>(id, std::move(controlPoints), std::move(nameData));
				for (const auto& cpWeak : newCurve->m_controlPoints)
				{
					if (auto cp = cpWeak.lock())
						cp->AddDependent(newCurve);
				}
				newObject = std::move(newCurve);
			};

			if (type == BezierSurface::SchemaName || type == BSplineSurface::SchemaName)
			{
				uint2 size = geomJson["size"];
				uint2 samples = geomJson["samples"];

				std::shared_ptr<Surface> newSurface;
				auto assignSurface = [&]<typename SurfaceType>()
				{
					newSurface = std::make_shared<SurfaceType>(id, size, samples, std::move(controlPoints), std::move(nameData));
				};
				if (type == BezierSurface::SchemaName)
					assignSurface.operator() < BezierSurface > ();
				else
					assignSurface.operator() < BSplineSurface > ();


				for (const auto& cpWeak : newSurface->m_controlPoints)
				{
					if (auto cpShared = cpWeak.lock())
					{
						cpShared->AddDependent(newSurface);
						cpShared->m_surfaceLockCount++;
					}
				}

				newSurface->InitGeometry(device);
				newObject = std::move(newSurface);
			}
			else if (type == Torus::SchemaName)
			{
				float3 pos = geomJson["position"];
				float3 scale = geomJson["scale"];
				uint2 samples = geomJson["samples"];
				float largeRadius = geomJson["largeRadius"];
				float smallRadius = geomJson["smallRadius"];
				Quaternion rotation = geomJson["rotation"];
				Mat4f rotMatrix = Mat4f::FromQuaternion(rotation);
				newObject = std::make_shared<Torus>(id, pos, scale, rotMatrix, largeRadius, smallRadius, samples, std::move(nameData));
			}
			else if (type == BezierCurve::SchemaName)
				assignCurve.operator() < BezierCurve > ();
			else if (type == BSplineCurve::SchemaName)
				assignCurve.operator() < BSplineCurve > ();
			else if (type == InterpolatingCurve::SchemaName)
				assignCurve.operator() < InterpolatingCurve > ();

			if (newObject)
				newSceneObjects.push_back(std::move(newObject));
		}
	}
	return newSceneObjects;
}
