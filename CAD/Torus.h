#pragma once
#include "../MathLib/Mat4f.h"
#include "structs.h"
#include "SceneObject.h"

class DxDevice;


struct Torus : public TransformableObject, public NamedObjectCounter<Torus>
{
	DEFINE_TYPE(TransformableObject, ObjectType::Torus);
	Torus(float3 position);
	Torus(unsigned int id, unsigned int torusIndex, std::string&& name,
		float3 position, float3 scale, const MathLib::Mat4f& rotationMatrix,
		float majorRadius, float minorRadius, uint2 samples);

	std::vector<VertexPosition> vertices;
	std::vector<unsigned int> indices;

#pragma region Constants
	static constexpr float cMinMajorRadius = 0.1f;
	static constexpr float cMaxMajorRadius = 1.0f;
	static constexpr float cMinMinorRadius = 0.1f;
	static constexpr float cMaxMinorRadius = 1.0f;
	static constexpr int cMinMajorSegments = 3;
	static constexpr int cMaxMajorSegments = 100;
	static constexpr int cMinMinorSegments = 3;
	static constexpr int cMaxMinorSegments = 100;
	static constexpr float cMinScale = 0.1f;
	static constexpr float cMaxScale = 100.0f;
#pragma endregion

#pragma region Setters
	void SetMajorRadius(float radius);
	void SetMinorRadius(float radius);
	void SetSegments(int major, int minor);
	//void SetScale(float scale);
	void SetScale(const float3& scale);
#pragma endregion

	bool IsDirty() const { return dirty; }
	void UpdateMesh(const DxDevice& device);

#pragma region Getters
	float GetMajorRadius() const { return majorRadius; }
	float GetMinorRadius() const { return minorRadius; }
	int GetMajorSegments() const { return majorSegments; }
	int GetMinorSegments() const { return minorSegments; }
	float3 GetScale() const { return m_scale; }
	//float GetScale() const { return m_scale; }

	Microsoft::WRL::ComPtr<ID3D11Buffer> GetVertexBuffer() const { return m_vertexBuffer; }
	Microsoft::WRL::ComPtr<ID3D11Buffer> GetIndexBuffer() const { return m_indexBuffer; }
#pragma endregion

	//float3 m_position{ 0, 0,-2 };
	float3 m_eulerAngles{ 0, 0, 0 };
	//float m_scale = 1.0f;
	//float m_baseScale = 1.0f;
	float3 m_scale{ 1.0f, 1.0f, 1.0f };
	float3 m_baseScale{ 1.0f, 1.0f, 1.0f };
	MathLib::Mat4f m_rotationMatrix = MathLib::Mat4f::Identity();
	MathLib::Mat4f m_baseRotationMatrix = MathLib::Mat4f::Identity();
	MathLib::Mat4f m_modelMatrix{};// = MathLib::Mat4f::Translation(m_position.x, m_position.y, m_position.z);

	void UpdateModelMatrix();
	nlohmann::json Serialize() const override;
	const char* GetSchemaType() const override { return "torus"; }

	virtual ~Torus() = default;

private:
	void GenerateMesh();
	void GenerateVertices();
	void GenerateIndices();
	void UpdateBuffers(const DxDevice& device);

	float majorRadius = 0.5f;
	float minorRadius = 0.2f;
	int majorSegments = 20;
	int minorSegments = 10;
	bool dirty = true; // Indicates whether the mesh needs to be regenerated.


	Microsoft::WRL::ComPtr<ID3D11Buffer> m_vertexBuffer;
	Microsoft::WRL::ComPtr<ID3D11Buffer> m_indexBuffer;

	//static unsigned int s_nextId;
};