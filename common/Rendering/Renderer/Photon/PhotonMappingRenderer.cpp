#include "common/Rendering/Renderer/Photon/PhotonMappingRenderer.h"
#include "common/Scene/Scene.h"
#include "common/Sampling/ColorSampler.h"
#include "common/Scene/Lights/Light.h"
#include "common/Scene/Geometry/Primitives/Primitive.h"
#include "common/Scene/Geometry/Mesh/MeshObject.h"
#include "common/Rendering/Material/Material.h"
#include "common/Intersection/IntersectionState.h"
#include "common/Scene/SceneObject.h"
#include "common/Scene/Geometry/Mesh/MeshObject.h"
#include "common/Rendering/Material/Material.h"
#include "glm/gtx/component_wise.hpp"

#define VISUALIZE_PHOTON_MAPPING 1

PhotonMappingRenderer::PhotonMappingRenderer(std::shared_ptr<class Scene> scene, std::shared_ptr<class ColorSampler> sampler):
    BackwardRenderer(scene, sampler), 
    diffusePhotonNumber(1000000),
    maxPhotonBounces(1000)
{
    srand(static_cast<unsigned int>(time(NULL)));
}

void PhotonMappingRenderer::InitializeRenderer()
{
    // Generate Photon Maps
    GenericPhotonMapGeneration(diffuseMap, diffusePhotonNumber);
    diffuseMap.optimise();
}

void PhotonMappingRenderer::GenericPhotonMapGeneration(PhotonKdtree& photonMap, int totalPhotons)
{
	hitCount = 0;
	bounceHitCount = 0;
	srand(time(NULL));
    float totalLightIntensity = 0.f;
    size_t totalLights = storedScene->GetTotalLights();
    for (size_t i = 0; i < totalLights; ++i) {
        const Light* currentLight = storedScene->GetLightObject(i);
        if (!currentLight) {
            continue;
        }
        totalLightIntensity += glm::length(currentLight->GetLightColor());
    }

    // Shoot photons -- number of photons for light is proportional to the light's intensity relative to the total light intensity of the scene.
    for (size_t i = 0; i < totalLights; ++i) {
        const Light* currentLight = storedScene->GetLightObject(i);
        if (!currentLight) {
            continue;
        }

        const float proportion = glm::length(currentLight->GetLightColor()) / totalLightIntensity;
        const int totalPhotonsForLight = static_cast<const int>(proportion * totalPhotons);
        const glm::vec3 photonIntensity = currentLight->GetLightColor() / static_cast<float>(totalPhotonsForLight);
        for (int j = 0; j < totalPhotonsForLight; ++j) {
            Ray photonRay;
            std::vector<char> path;
            path.push_back('L');
            currentLight->GenerateRandomPhotonRay(photonRay);
            TracePhoton(photonMap, &photonRay, photonIntensity, path, 1.f, maxPhotonBounces);
        }
    }
	std::cout << "hitCount " << hitCount << std::endl;
	std::cout << "bounceHitCount " << bounceHitCount << std::endl;
}

void PhotonMappingRenderer::TracePhoton(PhotonKdtree& photonMap, Ray* photonRay, glm::vec3 lightIntensity, std::vector<char>& path, float currentIOR, int remainingBounces)
{
    /*
     * Assignment 8 TODO: Trace a photon into the scene and make it bounce.
     *    
     *    How to insert a 'Photon' struct into the photon map.
     *        Photon myPhoton;
     *        ... set photon properties ...
     *        photonMap.insert(myPhoton);
     */

	if (remainingBounces < 0) return;

    assert(photonRay);
    IntersectionState state(0, 0);
    state.currentIOR = currentIOR;
	if (!storedScene->Trace(photonRay, &state)) {
		return;
	}
	else if (path.size() == 1)
	{
		hitCount++;
	}
	else if (path.size() > 1) {
		bounceHitCount++;
		const auto intersectionPoint = state.intersectionRay.GetRayPosition(state.intersectionT);
		auto toLignthRay = Ray(*photonRay);
		toLignthRay.SetRayDirection(-(photonRay->GetRayDirection()));
		Photon photon;
		photon.position = intersectionPoint;
		photon.intensity = lightIntensity;
		photon.toLightRay = toLignthRay;
		photonMap.insert(photon);
	}

	const auto hitMeshObject = state.intersectedPrimitive->GetParentMeshObject();
	const auto hitMaterial = hitMeshObject->GetMaterial();

	const auto reflection = hitMaterial->GetBaseDiffuseReflection();
	const auto pr = glm::max(glm::max(reflection.x, reflection.y), reflection.z);
	const auto p = ((float)rand()) / RAND_MAX;
	if (p > pr)
	{
		// photon absorption
		return;
	}
	// photon scatter
	path.push_back('I');
	const auto u = ((float)rand()) / RAND_MAX;
	const auto v = ((float)rand()) / RAND_MAX;
	const auto r = glm::sqrt(u);
	const auto theta = 2.0f * glm::pi<float>() * v;
	const auto x = r * glm::cos(theta);
	const auto y = r * glm::sin(theta);
	const auto z = glm::sqrt(1 - u);

	const auto localRayDir = glm::normalize(glm::vec3(x, y, z));

	glm::vec3 xdir;
	const auto n = state.ComputeNormal();
	const auto testParallel = glm::dot(n, glm::vec3(1, 0, 0));
	if (1 - glm::abs(testParallel) < 0.1) {
		xdir = glm::vec3(0, 1, 0);
	}
	else {
		xdir = glm::vec3(1, 0, 0);
	}
	const auto t = glm::cross(n, xdir);
	const auto b = glm::cross(n, t);
	const auto mat = glm::mat3x3(t, b, n);

	const auto worldRayDir = mat * localRayDir;
	photonRay->SetRayDirection(worldRayDir);

	TracePhoton(photonMap, photonRay, lightIntensity, path, currentIOR, remainingBounces - 1);
}

glm::vec3 PhotonMappingRenderer::ComputeSampleColor(const struct IntersectionState& intersection, const class Ray& fromCameraRay) const
{
    glm::vec3 finalRenderColor = BackwardRenderer::ComputeSampleColor(intersection, fromCameraRay);
#if VISUALIZE_PHOTON_MAPPING
    Photon intersectionVirtualPhoton;
    intersectionVirtualPhoton.position = intersection.intersectionRay.GetRayPosition(intersection.intersectionT);

    std::vector<Photon> foundPhotons;
    diffuseMap.find_within_range(intersectionVirtualPhoton, 0.003f, std::back_inserter(foundPhotons));
    if (!foundPhotons.empty()) {
        finalRenderColor += glm::vec3(1.f, 0.f, 0.f);
    }
#endif
    return finalRenderColor;
}

void PhotonMappingRenderer::SetNumberOfDiffusePhotons(int diffuse)
{
    diffusePhotonNumber = diffuse;
}
