#include "Engine/PhysicsSim/SoftBody/SoftBody.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Core/VertexUtils.hpp"

void SoftBody::SetShader(Shader* shader)
{
	m_shader = shader;
}

void SoftBody::ToggleWireframe()
{
	m_isWireframe = !m_isWireframe;
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
