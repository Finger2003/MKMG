#pragma once
#include "structs.h"
#include "SceneObject.h"
#include "../MathLib/Mat4f.h"

class DxDevice;
struct IPointDependent;
struct Point : public TransformableObject, public NamedObjectCounter<Point>
{
	DEFINE_TYPE(TransformableObject, ObjectType::Point);
	static void InitSharedGeometry(const DxDevice& device);
	static void ReleaseSharedGeometry();
	Point(float3 position);
	Point(unsigned int id, float3 position, std::optional<ParsedNameData>&& nameData);


	static const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetSharedVertexBuffer() { return s_vertexBuffer; }
	const Microsoft::WRL::ComPtr<ID3D11Buffer>& GetVertexBuffer() const { return s_vertexBuffer; }
	std::vector<std::weak_ptr<IPointDependent>> m_dependents;
	int m_surfaceLockCount = 0;
	//bool isLockedToSurface = false;

	void AddDependent(std::weak_ptr<IPointDependent> obj);

	template <std::ranges::input_range R>
		requires std::convertible_to<std::ranges::range_reference_t<R>, std::weak_ptr<IPointDependent>>
	void AddDependents(R&& objs) { m_dependents.insert(m_dependents.end(), std::ranges::begin(objs), std::ranges::end(objs)); }


	void RemoveDependent(IPointDependent* obj);
	void NotifyDependents();
	MathLib::Mat4f GetModelMatrix() const;
private:
	inline static Microsoft::WRL::ComPtr<ID3D11Buffer> s_vertexBuffer = nullptr;
};


struct IPointDependent
{
	virtual ~IPointDependent() = default;
	virtual void MarkDirty() = 0;
	virtual void ReplacePoint(Point* oldPoint, std::shared_ptr<Point> newPoint) = 0;
};