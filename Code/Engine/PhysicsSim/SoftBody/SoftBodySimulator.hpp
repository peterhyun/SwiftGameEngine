#pragma once
#include <vector>
class SoftBody;
class Renderer;
struct SoftBodyEdge;
struct Vec3;

class SoftBodySimulator {
public:
	SoftBodySimulator(SoftBody& body, Renderer& renderer);
	void Update();
	void Render() const;

	//Getter Setters
	void SetTimeStep(float newTimeStep);
	float GetTimeStep() const;
	void SetSolverIterations(unsigned int solverIterations);
	unsigned int GetSolverIterations() const;
	void SetSolverSubsteps(unsigned int solverSubsteps);
	unsigned int GetSolverSubsteps() const;

	//Getter Setter for distance constraint
	void SetInverseDistanceStiffness(float inverseDistanceStiffness);
	float GetInverseDistanceStiffness() const;
	bool UseDistanceConstraints() const;
	void SetUseDistanceConstraints(bool useDistanceConstraints);
	//Getter Setter for volume constraint
	void SetInverseVolumeStiffness(float inverseVolumeStiffness);
	float GetInverseVolumeStiffness() const;
	bool UseVolumeConstraint() const;
	void SetUseVolumeConstraints(bool useVolumeConstraint);

	void Reset();

	float GetParticleWeights() const;
	void SetParticleWeights(float particleWeights);

private:
	inline float GetDistanceConstraint(const Vec3& pos1, const Vec3& pos2, float originalDist) const;
	inline Vec3 GetDistanceConstraintPartialDerivativePos1(const Vec3& pos1, const Vec3& pos2) const;
	void SolveDistanceConstraints(std::vector<float>& lambdas, float compliance) const;

	inline float GetVolumeConstraint() const;
	void SolveVolumeConstraint(float& volumeLambda, float compliance) const;

private:
	Renderer& m_renderer;
	SoftBody& m_softBody;
	float m_timeStep = 0.02f;

	unsigned int m_solverIterations = 30;
	unsigned int m_solverSubsteps = 10;

	//Distance Constraint related
	float m_inverseDistanceStiffness = 0.0010f;
	bool m_isUsingDistraintConstraints = true;

	//Volume Constraint related
	float m_inverseVolumeStiffness = 0.0010f;

	bool m_useDistanceConstraints = true;
	bool m_useVolumeConstraint = true;
};