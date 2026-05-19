#pragma once
#include "SceneObject.h"
#include "Point.h"

enum class SurfaceShape { Flat, Cylinder };
//enum class SurfaceType { C0, C2 };

struct SurfaceBuilder;
struct Surface : public SceneObject
{
	DEFINE_TYPE(SceneObject, ObjectType::Surface);
	SurfaceShape shapeType;
	int segmentsU;
	int segmentsV;
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
	Surface(int uSeg, int vSeg, SurfaceShape shape, std::string&& name = "Surface")
		: SceneObject(std::move(name), ObjectType::Surface), segmentsU(uSeg), segmentsV(vSeg), shapeType(shape)
	{}
	virtual ~Surface() = default;
protected:


	//std::vector<unsigned int> GenerateLineIndices() const;
	//std::vector<unsigned int> GeneratePatchIndices() const;
	unsigned int GetControlPointIndex(int u, int v) const;

	virtual unsigned int GetGridPointsU() const = 0;
	virtual unsigned int GetGridPointsV() const = 0;
	virtual unsigned int GetPatchDataIndex(int u, int v) const = 0;
	virtual unsigned int GetExportPointsU() const = 0;
	virtual unsigned int GetExportPointsV() const = 0;

	nlohmann::json Serialize() const override;
};

struct SurfaceGenerationResult
{
	std::unique_ptr<Surface> surface;
	std::vector<std::shared_ptr<Point>> points;
};

