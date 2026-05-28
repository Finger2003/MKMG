#pragma once
#include "Point.h"
#include "BezierSurface.h"

struct PatchEdge
{
	Point* startPoint;
	Point* endPoint;
	std::vector<std::weak_ptr<Point>> boundaryPoints; // P_3i dependencies
	std::vector<std::weak_ptr<Point>> innerPoints;    // P_2i dependencies
	std::weak_ptr<BezierSurface> surface;

	PatchEdge() = default;
	PatchEdge(Point* start, Point* end, std::vector<std::weak_ptr<Point>>&& boundaryPts, std::vector<std::weak_ptr<Point>>&& innerPts, std::weak_ptr<BezierSurface> surf)
		: startPoint(start), endPoint(end), boundaryPoints(std::move(boundaryPts)), innerPoints(std::move(innerPts)), surface(std::move(surf))
	{}
};

struct Hole3Cycle
{
	std::array<PatchEdge, 3> edges;
};

struct EdgeKey
{
	Point* p1;
	Point* p2;
	bool operator==(const EdgeKey& other) const
	{
		return p1 == other.p1 && p2 == other.p2;
		//return (p1 == other.p1 && p2 == other.p2) || (p1 == other.p2 && p2 == other.p1);
	}
};

struct EdgeKeyHash
{
	std::size_t operator()(const EdgeKey& k) const
	{
		auto h1 = std::hash<Point*>{}(k.p1);
		auto h2 = std::hash<Point*>{}(k.p2);
		return h1 ^ (h2 << 1);
	}
};

std::vector<Hole3Cycle> Find3SidedHoles(const std::vector<std::shared_ptr<BezierSurface>>& surfaces);