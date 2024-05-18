#pragma once
#include <Engine/Math/Vec3.hpp>
#include <Engine/Renderer/VertexBuffer.hpp>
#include <Engine/Renderer/IndexBuffer.hpp>
#include <Engine/Core/Vertex_PCUTBN.hpp>
#include <vector>

class Shader;

struct SoftBodyEdge {
public:
	unsigned int m_positionIndices[2] = { 0, 0 };
	int m_triangleIndices[2] = { -1, -1 };	//-1 if couldn't find it

	double m_initialLength = 0.0f;

	bool operator==(const SoftBodyEdge& other) const {
		bool positionMatch =
			((m_positionIndices[0] == other.m_positionIndices[0] && m_positionIndices[1] == other.m_positionIndices[1]) ||
				(m_positionIndices[0] == other.m_positionIndices[1] && m_positionIndices[1] == other.m_positionIndices[0]));

		/*
		bool triangleMatch =
			((m_triangleIndices[0] == other.m_triangleIndices[0] && m_triangleIndices[1] == other.m_triangleIndices[1]) ||
				(m_triangleIndices[0] == other.m_triangleIndices[1] && m_triangleIndices[1] == other.m_triangleIndices[0]));
		*/

		return positionMatch; //&& triangleMatch;
	}
};

struct SoftBodyTriangle {
public:
	unsigned int m_positionIndices[3] = {0, 0, 0};

	bool ContainsPositionIndices(unsigned int posIndex1, unsigned int posIndex2) const {
		bool foundPosIndex1 = false;
		bool foundPosIndex2 = false;

		for (int i = 0; i < 3; ++i) {
			if (m_positionIndices[i] == posIndex1) {
				foundPosIndex1 = true;
			}
			if (m_positionIndices[i] == posIndex2) {
				foundPosIndex2 = true;
			}
		}

		return foundPosIndex1 && foundPosIndex2;
	}
};

class SoftBody {
	friend class OBJLoader;
	friend class SoftBodySimulator;
public:
	~SoftBody() { delete m_vbo; delete m_ibo; };
	void SetShader(Shader* shader);

	void ToggleWireframe();
	float CalculateVolume() const;

	float GetPressure() const;
	void SetPressure(float pressure);

	float GetInitialVolume() const;

	size_t GetNumTriangles() const;
	size_t GetNumEdges() const;

private:
	void UpdateVertices(class Renderer& renderer);
	void Reset();

private:
	std::vector<Vec3> m_positions;
	std::vector<Vec3> m_initialPositions;
	std::vector<Vec3> m_velocities;

	std::vector<SoftBodyTriangle> m_triangles;
	std::vector<SoftBodyEdge> m_edges;

	float m_initialVolume = 0.0f;

	//Render data
	std::vector<Vertex_PCUTBN> m_verts;
	std::vector<unsigned int> m_indices;
	VertexBuffer* m_vbo = nullptr;
	IndexBuffer* m_ibo = nullptr;
	std::vector<unsigned int> m_vertsToParticlesMap;

	Shader* m_shader = nullptr;
	bool m_isWireframe = false;

	float m_pressure = 1.0f;
};