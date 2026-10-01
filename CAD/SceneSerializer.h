#pragma once

struct SceneObject;
class DxDevice;

namespace SceneSerializer
{
    // Throws std::exception on file I/O failure
	void Save(const std::wstring& filePath, const std::vector<std::shared_ptr<SceneObject>>& objects);

    // Returns a populated vector if successful, or throws std::exception on parse failure
	std::vector<std::shared_ptr<SceneObject>> Load(const std::wstring& filePath, DxDevice& device);
}