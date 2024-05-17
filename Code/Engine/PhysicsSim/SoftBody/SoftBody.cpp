#include "Engine/PhysicsSim/SoftBody/SoftBody.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/VertexUtils.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Math/MathUtils.hpp"

void SoftBody::SetShader(Shader* shader)
{
	m_shader = shader;
}

void SoftBody::ToggleWireframe()
{
	m_isWireframe = !m_isWireframe;
}

float SoftBody::CalculateVolume() const
{
	float volume = 0.0f;
	float inv_6 = 1.0f / 6.0f;
	for (const auto& triangle : m_triangles) {
		volume += ScalarTripleProduct(m_positions[triangle.m_positionIndices[0]], m_positions[triangle.m_positionIndices[1]], m_positions[triangle.m_positionIndices[2]]) * inv_6;
	}
	return volume;
}

float SoftBody::GetPressure() const
{
	return m_pressure;
}

void SoftBody::SetPressure(float pressure)
{
	GUARANTEE_OR_DIE(pressure >= 0.0f, "pressure < 0.0f");
	m_pressure = pressure;
}

void SoftBody::UpdateVertices(Renderer& renderer)
{
	for (int i = 0; i < m_vertsToParticlesMap.size(); i++) {
		m_verts[i].m_position = m_positions[m_vertsToParticlesMap[i]];
	}
	CalculateAveragedNormals(m_verts, m_indices);
	renderer.CopyCPUToGPU(m_verts.data(), m_verts.size() * sizeof(Vertex_PCUTBN), sizeof(Vertex_PCUTBN), m_vbo);
}

void SoftBody::Reset()
{
	m_positions = m_initialPositions;
	for (int i = 0; i < m_velocities.size(); i++) {
		m_velocities[i] = Vec3();
	}
}
