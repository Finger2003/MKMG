#pragma once
#include "SceneObject.h"
#include "Point.h"

enum class SurfaceShape { Flat, Cylinder };

struct SurfaceBuilder;
struct Surface : public SceneObject
{
	DEFINE_TYPE(SceneObject, ObjectType::Surface);
	SurfaceShape shapeType;

	int m_gridPointsU = 0;
	int m_gridPointsV = 0;
	bool m_isDirty = true;

	std::vector<std::weak_ptr<Point>> m_controlPoints;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_polylineVertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_polylineIndexBuffer;
	UINT m_polylineIndexCount = 0;

	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchVertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_patchIndexBuffer;
	UINT m_patchIndexCount = 0;


	int m_linesPerSegmentU = 4;
	int m_linesPerSegmentV = 4;
	int m_smoothness = 25;


	std::shared_ptr<Point> GetPoint(int u, int v) const;

	virtual void InitGeometry(const DxDevice& device) = 0;
	virtual void UpdateVertices(const DxDevice& device) = 0;
	void MarkDirty() override { m_isDirty = true; }
	friend SurfaceBuilder;

	std::vector<unsigned int> GenerateLineIndices() const;


	Surface(int uGrid, int vGrid, SurfaceShape shape, std::string&& name)
		: SceneObject(std::move(name), ObjectType::Surface), m_gridPointsU(uGrid), m_gridPointsV(vGrid), shapeType(shape)
	{}

	Surface(unsigned int id, std::string&& name, 
		int gridPointsU, int gridPointsV, int linesPerSegmentU, int linesPerSegmentV, 
		SurfaceShape shape, std::vector<std::weak_ptr<Point>>&& controlPoints)
		: SceneObject(id, std::move(name), ObjectType::Surface),
		m_gridPointsU(gridPointsU), m_gridPointsV(gridPointsV), m_linesPerSegmentU(linesPerSegmentU), m_linesPerSegmentV(linesPerSegmentV), 
		shapeType(shape), m_controlPoints(std::move(controlPoints))
	{}
	virtual ~Surface() = default;
protected:
	unsigned int GetControlPointIndex(int u, int v) const;
	virtual unsigned int GetPatchDataIndex(int u, int v) const = 0;

	nlohmann::json Serialize() const override;
};

struct SurfaceGenerationResult
{
	std::unique_ptr<Surface> surface;
	std::vector<std::shared_ptr<Point>> points;
};

