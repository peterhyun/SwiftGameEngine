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
	//Getter Setter for distance constraint
	void SetInverseDistanceStiffness(float inverseDistanceStiffness);
	float GetInverseDistanceStiffness() const;

	void Reset();

private:
	inline float GetDistanceConstraint(const Vec3& pos1, const Vec3& pos2, float originalDist);
	inline Vec3 GetDistanceConstraintPartialDerivativePos1(const Vec3& pos1, const Vec3& pos2);
	void SolveDistanceConstraints(std::vector<float>& lambdas, float compliance);
	void SolveVolumeConstraint();

private:
	Renderer& m_renderer;
	SoftBody& m_softBody;
	float m_timeStep = 0.002f;
	unsigned int m_solverIterations = 30;

	//Distance Constraint related
	float m_inverseDistanceStiffness = 0.0010f;
	bool m_isUsingDistraintConstraints = true;
};