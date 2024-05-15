#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <iostream>

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float a{ Vector3::Dot(ray.direction, ray.direction) };
			Vector3 oDiff{ ray.origin - sphere.origin };
			float b{ 2 * Vector3::Dot(ray.direction, oDiff) };
			float c{ Vector3::Dot(oDiff, oDiff) - sphere.radius * sphere.radius };

			//d is the discriminant of the equation
			float d = b * b - 4 * a * c;

			//We are only interested in full intersection (so discriminant > 0).
			if (d > 0)
			{
				//Use subtraction, except when t < tMin, then use addition for t.
				float t = (-b - sqrtf(d)) / 2 / a;

				if (t < ray.min)
				{
					t = (-b + sqrtf(d)) / 2 / a;
				}

				if (t >= ray.min && t <= ray.max)
				{
					hitRecord.t = t;
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + t * ray.direction;
					hitRecord.materialIndex = sphere.materialIndex;
					hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();
					return true;
				}
			}

			return false;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W1
			const float dotProduct{ Vector3::Dot(ray.direction, plane.normal) };

			if (dotProduct < 0)
			{
				float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / dotProduct;

				if (t >= ray.min && t <= ray.max)
				{
					hitRecord.t = t;
					hitRecord.didHit = true;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.normal = plane.normal;
					hitRecord.origin = ray.origin + t * ray.direction;
					return true;
				}
			}
			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			const Vector3 edge1{ triangle.v1 - triangle.v0 };
			const Vector3 edge2{ triangle.v2 - triangle.v0 };
			
			const Vector3 h{ Vector3::Cross(ray.direction, edge2) };
			const float a{ Vector3::Dot(edge1, h) };

			if (a < -FLT_EPSILON)
			{
				if (!ignoreHitRecord && triangle.cullMode == TriangleCullMode::BackFaceCulling)
					return false;
			}
			else if (a > FLT_EPSILON)
			{
				if (!ignoreHitRecord && triangle.cullMode == TriangleCullMode::FrontFaceCulling)
					return false;
			}
			else
			{
				return false;
			}

			const float f{ 1.0f / a };
			const Vector3 s{ ray.origin - triangle.v0 };
			const float u{ f * Vector3::Dot(s, h) };

			if (u < 0.0f || u > 1.0f)
				return false;

			const Vector3 q{ Vector3::Cross(s, edge1) };
			const float v{ f * Vector3::Dot(ray.direction, q) };

			if (v < 0.0f || u + v > 1.0f)
				return false;

			const float t{ f * Vector3::Dot(edge2, q) };
			if (t > ray.min && t < ray.max)
			{
				if (ignoreHitRecord) return true;
				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.origin = ray.origin + (t * ray.direction);
				hitRecord.normal = triangle.normal;
				hitRecord.t = t;
				return true;
			}
			return false;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			// Perform slabtest on the mesh (acceleration structures)

			float tx1 = (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x;
			float tx2 = (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y;
			float ty2 = (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z;
			float tz2 = (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;

		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			if (!SlabTest_TriangleMesh(mesh, ray))
				return false;

			// Loop through all triangles in the mesh, and check if they hit the ray.
			Triangle triangle{};
			const size_t trianglePositionsSize{ mesh.positions.size() };
			const size_t meshIndicesSize{ mesh.indices.size() };

			for (size_t i{}; i < meshIndicesSize; i += 3)
			{
				triangle.v0 = mesh.transformedPositions[mesh.indices[i]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[i + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[i + 2]];
				triangle.normal = mesh.transformedNormals[i / 3];
				triangle.cullMode = mesh.cullMode;
				triangle.materialIndex = mesh.materialIndex;

				HitRecord tempHitrecord{};
				if (HitTest_Triangle(triangle, ray, tempHitrecord, ignoreHitRecord))
				{
					if (ignoreHitRecord)
					{
						return true;
					}
					else if (tempHitrecord.t > 0.0f && tempHitrecord.t < hitRecord.t)
					{
						hitRecord = tempHitrecord;
					}

				}
			}
			return hitRecord.didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			switch (light.type)
			{
			case dae::LightType::Point:
				return light.origin - origin;
				break;
			case dae::LightType::Directional:
				return -light.direction * FLT_MAX;
				break;
			default:
				return -light.direction * FLT_MAX;
				break;
			}
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			switch (light.type)
			{
			default:
			case dae::LightType::Point:
			{
				const float radiantPower{ light.intensity };
				const float sphereRadiusSquared((light.origin - target).SqrMagnitude()); 
				const float irradiance{ radiantPower / sphereRadiusSquared };

				return light.color * irradiance;
				break;
			}
			case dae::LightType::Directional:
				return light.color * light.intensity;
				break;
			}
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}