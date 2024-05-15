//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"
#include "future"
#include "ppl.h"


using namespace dae;

//#define ASYNC
#define PARALLEL_FOR

Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();

	const float aspectRatio = {m_Width / static_cast<float>(m_Height)};

	const float ar{ float(m_Width * 1.f / m_Height) };
	const float fovAngle = camera.fovAngle * TO_RADIANS;
	const float fov = tan( fovAngle / 2.f );

	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const uint32_t numPixels = m_Width * m_Height;

#if defined(ASYNC)
	//ASYNC EXE
	const uint32_t numCores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> async_futures{};
	const uint32_t numPixelsPerTask = numPixels / numCores;
	uint32_t numUnassignedPixels = numPixels % numCores;
	uint32_t currPixelIndex = 0;

	for(uint32_t coreId{0}; coreId < numCores; ++coreId)
	{
		uint32_t taskSize = numPixelsPerTask;
		if( numUnassignedPixels > 0)
		{
			++taskSize;
			--numUnassignedPixels;
		}

		async_futures.push_back(
			std::async(std::launch::async, [=, this]
			{
					const uint32_t pixelIndexEnd = currPixelIndex + taskSize;
					for (uint32_t pixelIndex{ currPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
				{
					RenderPixel(pScene, pixelIndex, fov, aspectRatio, camera, lights, materials);
				}
			})
		);
		currPixelIndex += taskSize;
	}

	for (const std::future<void>& f : async_futures)
	{
		f.wait();
	}

#elif defined(PARALLEL_FOR)
	//PARALLEL-FOR
	concurrency::parallel_for(0u, numPixels, [=, this](int i)
		{
			RenderPixel(pScene, i, fov, aspectRatio, camera, lights, materials);
		});

#else

	for (uint32_t i{0}; i < numPixels; ++i)
	{
		RenderPixel(pScene, i, fov, aspectRatio, pScene->GetCamera(), lights, materials);
	}
#endif
	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void Renderer::RenderPixel(Scene* pscene, uint32_t pixelIndex, float fov, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	const int px = pixelIndex % m_Width;
    const int py = pixelIndex / m_Width ;

    const float rx = px + 0.5f;
    const float ry = py + 0.5f;

    const float cx{ (2 * rx / float(m_Width) - 1) * aspectRatio * fov };
    const float cy{ (1 - 2 * (ry / float(m_Height))) * fov };

	Vector3 rayDirection{ cx * camera.right + cy * camera.up + 1.0f * camera.forward };
	Vector3 normalRayDir{ rayDirection.Normalized() };
	
	Ray viewRay{ camera.origin, normalRayDir };

	ColorRGB finalColor{};

	HitRecord closestHit{};

	pscene->GetClosestHit(viewRay, closestHit);


	if (closestHit.didHit)
	{
		auto material{ materials[closestHit.materialIndex] };

		for (unsigned long i{}; i < lights.size(); ++i)
		{
			

			Vector3 directionToLight = LightUtils::GetDirectionToLight(lights[i], closestHit.origin);
			float mag{ directionToLight.Magnitude() };
			directionToLight.Normalize();
			float observedArea{ Vector3::Dot(closestHit.normal, directionToLight) };
			float LambertVal{ Vector3::Dot(directionToLight, closestHit.normal) };
			Ray rayToLight = Ray{ closestHit.origin, directionToLight, 0.0001f, mag };
			
			if (observedArea >= 0.f && (!m_ShadowsActive || !pscene->DoesHit(rayToLight)))
			{
				ColorRGB radiance = LightUtils::GetRadiance(lights[i], closestHit.origin);
				ColorRGB BRDF = material->Shade(closestHit, directionToLight, -viewRay.direction);

				switch (m_CurrentLightingMode)
				{
				case LightingMode::ObservedArea:
					finalColor += ColorRGB(1.f, 1.f, 1.f) * observedArea;
					break;
				case LightingMode::Radiance:
					finalColor += radiance;
					break;
				case LightingMode::BRDF:
					finalColor += BRDF;
					break;
				case LightingMode::Combined:
					finalColor += radiance * observedArea * BRDF;
					break;
				}
			}
			
		}
	}

	//Update Color in Buffer
	finalColor.MaxToOne(); //Goes over 255 so starts over again reason why it's black

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode()
{
	switch (m_CurrentLightingMode)
	{
	case dae::Renderer::LightingMode::ObservedArea:
		m_CurrentLightingMode = LightingMode::Radiance;
		break;
	case dae::Renderer::LightingMode::Radiance:
		m_CurrentLightingMode = LightingMode::BRDF;
		break;
	case dae::Renderer::LightingMode::BRDF:
		m_CurrentLightingMode = LightingMode::Combined;
		break;
	case dae::Renderer::LightingMode::Combined:
		m_CurrentLightingMode = LightingMode::ObservedArea;
		break;
	}
}