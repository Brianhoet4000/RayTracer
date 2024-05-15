#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
		}


		Vector3 origin{};
		float fovAngle{45.f};

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{ 0.f };
		float totalYaw{ 0.f };

		Matrix cameraToWorld{};

		int tempMovementY{};

		Matrix CalculateCameraToWorld()
		{
			//todo: W2
			Vector4 t{ origin, 1.f };

			Matrix cam{ forward, up, right, t };
			return cam;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float speed{ 50 };
			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			if (pKeyboardState[SDL_SCANCODE_W] ||pKeyboardState[SDL_SCANCODE_UP])
			{
				origin.z += speed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_S] || pKeyboardState[SDL_SCANCODE_DOWN])
			{
				origin.z -= speed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_D] || pKeyboardState[SDL_SCANCODE_RIGHT])
			{
				origin.x += speed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_A] || pKeyboardState[SDL_SCANCODE_LEFT])
			{
				origin.x -= speed * deltaTime;
			}

			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);
			if ((mouseState & SDL_BUTTON_LMASK) && (mouseState & SDL_BUTTON_RMASK))
			{
				if (mouseY < -10)
				{
					++origin.y * deltaTime;
				}
				else if (mouseY > 10)
				{
					--origin.y * deltaTime;
				}
			}
			else if(mouseState & SDL_BUTTON_LMASK)
			{
				if (mouseY < -10)
				{
					++origin.z * deltaTime;
				}
				else if (mouseY > 10)
				{
					--origin.z * deltaTime;
				}
				if (mouseX < -10)
				{
					totalYaw += 0.05f;
				}
				else if (mouseX > 10)
				{
					totalYaw -= 0.05f;
				}
			}
			else if (mouseState & SDL_BUTTON_RMASK)
			{
				if (mouseX < -10)
				{
					totalYaw += 0.05f;
				}
				else if (mouseX > 10)
				{
					totalYaw -= 0.05f;
				}
				if (mouseY < -10)
				{
					totalPitch += 0.05f;
				}
				else if (mouseY > 10)
				{
					totalPitch -= 0.05f;
				}

			}
			Matrix pitchRotation = Matrix::CreateRotationX(totalPitch);
			Matrix yawRotation = Matrix::CreateRotationY(totalYaw);
			Matrix finalRotation = yawRotation * pitchRotation;
			forward = finalRotation.TransformVector(Vector3::UnitZ);
			forward.Normalize();
		}
	};
}
