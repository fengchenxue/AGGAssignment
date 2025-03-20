#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float phi = 2.0f * M_PI * r1;
		float z = r2;
		float r = sqrtf(std::max(0.0f, 1.0f - z * z));
		float x = r * cosf(phi);
		float y = r * sinf(phi);
		return Vec3(x, y, z);
	}
	static float uniformHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return 1.0f / (2.0f * M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		// Add code here
		float r = sqrtf(r1);
		float phi = 2.0f * M_PI * r2;
		float x = r * cosf(phi);
		float y = r * sinf(phi);
		float z = sqrtf(std::max(0.0f, 1.0f - x*x-y*y));
		return Vec3(x, y, z);
	}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		// Add code here
		return wi.z / M_PI;
	}
	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// Add code here
		float phi = 2.0f * M_PI * r1;
		float z = 2.0f * r2 - 1.0f;
		float r = sqrtf(std::max(0.0f, 1.0f - z * z));
		float x = r * cosf(phi);
		float y = r * sinf(phi);
		return Vec3(x, y, z);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1.0f/(4.0f*M_PI);
	}
};