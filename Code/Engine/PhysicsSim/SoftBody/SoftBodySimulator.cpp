#include "Engine/PhysicsSim/SoftBody/SoftBodySimulator.hpp"
#include "Engine/PhysicsSim/SoftBody/SoftBody.hpp"
#include "Engine/Core/ErrorWarningAssert.hpp"
#include "Engine/Renderer/Renderer.hpp"
#include "Engine/Math/MathUtils.hpp"

SoftBodySimulator::SoftBodySimulator(SoftBody& body, Renderer& renderer) : m_softBody(body), m_renderer(renderer)
{
	GUARANTEE_OR_DIE(m_softBody.m_positions.size() == m_softBody.m_velocities.size(), "Soft Body Initialization is wrong!");
}

void SoftBodySimulator::Update()
{
	float timeStepSquare = m_timeStep * m_timeStep;
	const Vec3 gravity(0.0f, 0.0f, -9.8f * 0.05f);

	std::vector<Vec3> prevPositions = m_softBody.m_positions;

	//Do the whole solving thing here
	//1. Predict position
	for (int particleIdx = 0; particleIdx < m_softBody.m_positions.size(); particleIdx++) {
		Vec3& particlePos = m_softBody.m_positions[particleIdx];
		particlePos += m_timeStep * m_softBody.m_velocities[particleIdx] + timeStepSquare * gravity;	//Applying external force: gravity
	}

	//2. Initialize solve and multipliers
	std::vector<float> distanceLambdas(m_softBody.m_edges.size(), 0.0f);
	const float distanceCompliance = m_inverseDistanceStiffness / (timeStepSquare);
	float volumeLambda = 0.0f;
	const float volumeCompliance = m_inverseVolumeStiffness / (timeStepSquare);

	//3. Solver iterations
	for (unsigned int solverIter = 0; solverIter < m_solverIterations; solverIter++) {
		SolveDistanceConstraints(distanceLambdas, distanceCompliance);
		SolveVolumeConstraint(volumeLambda, volumeCompliance);
	}

	// Solve the ground constraints
	for (int particleIdx = 0; particleIdx < m_softBody.m_positions.size(); particleIdx++) {
		Vec3& pos = m_softBody.m_positions[particleIdx];
		if (pos.z < 0.0f) {
			pos.z = 0.0f;
		}
	}

	//Update velocities
	for (int particleIdx = 0; particleIdx < m_softBody.m_positions.size(); particleIdx++) {
		Vec3 newVelocity = (m_softBody.m_positions[particleIdx] - prevPositions[particleIdx]) / m_timeStep;
		
		//Add some damping to the horizontal velocities
		if (m_softBody.m_positions[particleIdx].z == 0.0f) {
			newVelocity *= 0.9f;
		}

		m_softBody.m_velocities[particleIdx] = newVelocity;
	}

	m_softBody.UpdateVertices(m_renderer);
}

void SoftBodySimulator::Render() const
{
	if (m_softBody.m_isWireframe == false) {
		m_renderer.SetRasterizerMode(RasterizerMode::SOLID_CULL_BACK);
	}
	else {
		m_renderer.SetRasterizerMode(RasterizerMode::WIREFRAME_CULL_NONE);
	}
	m_renderer.SetBlendMode(BlendMode::OPAQUE);
	m_renderer.SetDepthStencilMode(DepthStencilMode::DEPTH_ENABLED_STENCIL_DISABLED);
	m_renderer.BindTexture(nullptr);

	m_renderer.SetModelConstants();

	m_renderer.BindShader(m_softBody.m_shader);

	m_renderer.DrawVertexAndIndexBuffer(m_softBody.m_vbo, m_softBody.m_ibo, (int)m_softBody.m_indices.size());
}

void SoftBodySimulator::SetTimeStep(float newTimeStep)
{
	GUARANTEE_OR_DIE(newTimeStep > 0.0f, "newTimeStep <= 0.0f");
	m_timeStep = newTimeStep;
}

float SoftBodySimulator::GetTimeStep() const
{
	return m_timeStep;
}

void SoftBodySimulator::SetSolverIterations(unsigned int solverIterations)
{
	m_solverIterations = solverIterations;
}

unsigned int SoftBodySimulator::GetSolverIterations() const
{
	return m_solverIterations;
}

void SoftBodySimulator::SetInverseDistanceStiffness(float inverseDistanceStiffness)
{
	GUARANTEE_OR_DIE(inverseDistanceStiffness >= 0.0f, "inverseDistanceStiffness < 0.0f");
	m_inverseDistanceStiffness = inverseDistanceStiffness;
}

float SoftBodySimulator::GetInverseDistanceStiffness() const
{
	return m_inverseDistanceStiffness;
}

void SoftBodySimulator::Reset()
{
	m_softBody.Reset();
}

inline float SoftBodySimulator::GetDistanceConstraint(const Vec3& pos1, const Vec3& pos2, float originalDist) const
{
	return (pos1 - pos2).GetLength() - originalDist;
}

inline Vec3 SoftBodySimulator::GetDistanceConstraintPartialDerivativePos1(const Vec3& pos1, const Vec3& pos2) const
{
	float length = (pos1 - pos2).GetLength();
	return (pos1 - pos2) / length;
}

inline float SoftBodySimulator::GetVolumeConstraint() const
{
	return m_softBody.CalculateVolume() - m_softBody.m_initialVolume;
}

 void SoftBodySimulator::SolveVolumeConstraint(float& volumeLambda, float compliance) const
{
	float volumeConstraint =  GetVolumeConstraint();
	if (volumeConstraint == 0.0f) {
		return;
	}
	std::vector<Vec3> constraintPDs(m_softBody.m_positions.size(), Vec3());	//Partial derivatives of each Jj
	float inv_6 = 1.0f / 6.0f;
	for (const auto& triangles : m_softBody.m_triangles) {
		int idx0 = triangles.m_positionIndices[0];
		int idx1 = triangles.m_positionIndices[1];
		int idx2 = triangles.m_positionIndices[2];

		const Vec3& pos0 = m_softBody.m_positions[idx0];;
		const Vec3& pos1 = m_softBody.m_positions[idx1];
		const Vec3& pos2 = m_softBody.m_positions[idx2];

		constraintPDs[idx0] += CrossProduct3D(pos1, pos2) * inv_6;
		constraintPDs[idx1] += CrossProduct3D(pos2, pos0) * inv_6;
		constraintPDs[idx2] += CrossProduct3D(pos0, pos1) * inv_6;
	}

	float denominator = compliance;
	for (const Vec3& pd : constraintPDs) {
		denominator += pd.GetLengthSquared();
	}

	float deltaLambda = -(volumeConstraint + compliance * volumeLambda) / denominator;
	for (int i = 0; i < m_softBody.m_positions.size(); i++) {
		Vec3& pos = m_softBody.m_positions[i];
		pos += constraintPDs[i] * deltaLambda;
	}
	volumeLambda += deltaLambda;
}

void SoftBodySimulator::SolveDistanceConstraints(std::vector<float>& lambdas, float compliance) const
{
	for (int edgeIdx = 0; edgeIdx < m_softBody.m_edges.size(); edgeIdx++) { // Solve the distance constraints
		const auto& edge = m_softBody.m_edges[edgeIdx];
		int idx1 = edge.m_positionIndices[0];
		int idx2 = edge.m_positionIndices[1];

		Vec3& pos1 = m_softBody.m_positions[idx1];
		Vec3& pos2 = m_softBody.m_positions[idx2];

		float distanceConstraint = GetDistanceConstraint(pos1, pos2, (float)edge.m_initialLength);

		Vec3 gradC1 = GetDistanceConstraintPartialDerivativePos1(pos1, pos2);
		Vec3 gradC2 = -gradC1;

		float effectiveMass = 1.0f + 1.0f; // Assuming mass = 1 for both particles
		float denom = effectiveMass + compliance;
		float deltaLambda = -(distanceConstraint + compliance * lambdas[edgeIdx]) / denom;

		pos1 += deltaLambda * gradC1;
		pos2 += deltaLambda * gradC2;

		lambdas[edgeIdx] += deltaLambda;
	}
}
