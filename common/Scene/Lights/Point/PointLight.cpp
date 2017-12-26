#include "common/Scene/Lights/Point/PointLight.h"
#include <random>

void PointLight::ComputeSampleRays(std::vector<Ray>& output, glm::vec3 origin, glm::vec3 normal) const
{
    origin += normal * LARGE_EPSILON;
    const glm::vec3 lightPosition = glm::vec3(GetPosition());
    const glm::vec3 rayDirection = glm::normalize(lightPosition - origin);
    const float distanceToOrigin = glm::distance(origin, lightPosition);
    output.emplace_back(origin, rayDirection, distanceToOrigin);
}

float PointLight::ComputeLightAttenuation(glm::vec3 origin) const
{
    return 1.f;
}

void PointLight::GenerateRandomPhotonRay(Ray& ray) const
{
    // Assignment 8 TODO: Fill in the random point light samples here.
	auto pos = this->GetPosition();
	ray.SetRayPosition(glm::vec3(pos.x, pos.y, pos.z));
	std::random_device rd;  //Will be used to obtain a seed for the random number engine
	std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> dis(-1.0, 1.0);
	float x = 0, y = 0, z = 0;
	do
	{
		x = dis(gen);
		y = dis(gen);
		z = dis(gen);
	} while (x*x+y*y+z*z > 1);
	ray.SetRayDirection(glm::normalize(glm::vec3(x, y, z)));
}
