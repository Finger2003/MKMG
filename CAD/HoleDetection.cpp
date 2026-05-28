#include "pch.h"
#include "HoleDetection.h"

std::vector<Hole3Cycle> Find3SidedHoles(const std::vector<std::shared_ptr<BezierSurface>>& surfaces)
{
	std::vector<PatchEdge> allEdges;
	for (const auto& surf : surfaces)
	{
		unsigned int u = surf->m_gridPointsU;
		unsigned int v = surf->m_gridPointsV;

		unsigned int segmentsU = surf->GetSegmentsU();
		unsigned int segmentsV = surf->GetSegmentsV();

		auto extractEdge = [&](unsigned int segments, auto getBoundaryIdx, auto getInnerIdx)
			{
				for (unsigned int seg = 0; seg < segments; seg++)
				{
					auto startShared = surf->m_controlPoints[getBoundaryIdx(seg, 0)].lock();
					auto endShared = surf->m_controlPoints[getBoundaryIdx(seg, 3)].lock();
					if (!startShared || !endShared || (startShared == endShared))
						continue;

					std::vector<std::weak_ptr<Point>> boundaryPts;
					std::vector<std::weak_ptr<Point>> innerPts;
					bool edgeValid = true;

					for (unsigned int i = 0; i < 4; i++)
					{
						auto bPt = surf->m_controlPoints[getBoundaryIdx(seg, i)].lock();
						auto iPt = surf->m_controlPoints[getInnerIdx(seg, i)].lock();

						if (!bPt || !iPt)
						{
							edgeValid = false;
							break;
						}
						boundaryPts.push_back(std::move(bPt));
						innerPts.push_back(std::move(iPt));
					}

					if (edgeValid)
					{
						allEdges.emplace_back(startShared.get(), endShared.get(),
							std::move(boundaryPts), std::move(innerPts), surf);
					}
				}
			};

		// 1. Bottom Edges (V = 0), Inner Row (V = 1)
		extractEdge(segmentsU,
			[u](unsigned int seg, unsigned int i) { return (seg * 3) + i; },
			[u](unsigned int seg, unsigned int i) { return 1 * u + (seg * 3) + i; }
		);
		// 2. Top Edges (V = v-1), Inner Row (V = v-2)
		extractEdge(segmentsU,
			[u, v](unsigned int seg, unsigned int i) { return (v - 1) * u + (seg * 3) + i; },
			[u, v](unsigned int seg, unsigned int i) { return (v - 2) * u + (seg * 3) + i; }
		);
		// 3. Left Edges (U = 0), Inner Row (U = 1)
		extractEdge(segmentsV,
			[u](unsigned int seg, unsigned int j) { return ((seg * 3) + j) * u; },
			[u](unsigned int seg, unsigned int j) { return ((seg * 3) + j) * u + 1; }
		);
		// 4. Right Edges (U = u-1), Inner Row (U = u-2)
		extractEdge(segmentsV,
			[u](unsigned int seg, unsigned int j) { return ((seg * 3) + j) * u + (u - 1); },
			[u](unsigned int seg, unsigned int j) { return ((seg * 3) + j) * u + (u - 2); }
		);
	}



	// Filter for boundary edges
	std::unordered_map<EdgeKey, std::vector<PatchEdge>, EdgeKeyHash> edgeFrequencyMap;
	for (const auto& edge : allEdges)
	{
		Point* minPt = std::min(edge.startPoint, edge.endPoint);
		Point* maxPt = std::max(edge.startPoint, edge.endPoint);

		edgeFrequencyMap[{minPt, maxPt}].push_back(edge);
	}

	std::vector<PatchEdge> boundaryEdges;
	for (auto& [key, edgesOnSeam] : edgeFrequencyMap)
	{
		// A hole edge must belong to exactly one surface. 
		// If size > 1, it's a shared internal seam.
		if (edgesOnSeam.size() == 1)
			boundaryEdges.push_back(std::move(edgesOnSeam[0]));
	}

	// Build adjacency list 
	std::unordered_map<Point*, std::vector<PatchEdge>> graph;
	for (const auto& edge : boundaryEdges)
	{
		graph[edge.startPoint].push_back(edge);
		graph[edge.endPoint].push_back(edge);
	}

	// Traverse graph to find 3-cycles
	std::vector<Hole3Cycle> holes;
	for (const auto& [nodeU, neighborsU] : graph)
	{
		for (const auto& edge1 : neighborsU)
		{
			Point* nodeV = (edge1.startPoint == nodeU) ? edge1.endPoint : edge1.startPoint;
			if (nodeV <= nodeU)
				continue;

			for (const auto& edge2 : graph[nodeV])
			{
				Point* nodeW = (edge2.startPoint == nodeV) ? edge2.endPoint : edge2.startPoint;
				if (nodeW <= nodeV)
					continue;

				for (const auto& edge3 : graph[nodeW])
				{
					Point* nodeX = (edge3.startPoint == nodeW) ? edge3.endPoint : edge3.startPoint;

					// The loop closes if nodeX connects back to nodeU
					if (nodeX == nodeU)
					{
						holes.push_back({ {edge1, edge2, edge3} });
						break;
					}
				}
			}
		}
	}
	return holes;
}
