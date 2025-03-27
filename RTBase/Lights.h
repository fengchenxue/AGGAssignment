#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

//used in EnvironmentMap for importance sampling
class Distribution1D
{
public:
	// density function
	std::vector<float> func;
	// normalized and cumulative density function
	std::vector<float> cdf;
	float area;
	Distribution1D(std::vector<float>& _func) :func(_func)
	{
		cdf.resize(func.size()+1);
		cdf[0] = 0;
		for (int i = 0; i < func.size(); i++)
		{
			cdf[i+1] = cdf[i] + func[i];
		}
		area = cdf[cdf.size()];

		//robustness measure
		if (fabs(area)<EPSILON){
			for (int i = 1; i <= cdf.size(); i++){
				cdf[i] = (float)i / (float)cdf.size();
			}
			area = 1.0f;
		}
		else
		{
			for (int i = 0; i < cdf.size(); i++)
			{
				cdf[i] = cdf[i] / area;
			}
		}
	}
	//return the neareast index of a random index
	int sample(float u, float &pdf)
	{
		auto in = std::lower_bound(cdf.begin(), cdf.end(), u);
		int index = std::max(0, (int)(in - cdf.begin() - 1));
		pdf = func[index] / area;
		return index;
	}

	float pdf(int index)
	{
		return func[index] / area;
	}
};

class Distribution2D
{
public:
	std::vector<Distribution1D*> pConditionalV;
	Distribution1D* pMarginal;
	Distribution2D(Texture* envmap)
	{
		int width = envmap->width;
		int height = envmap->height;

		std::vector<float> marginalFunc(height);
		pConditionalV.resize(height);

		for (int v = 0; v < height; v++)
		{
			std::vector<float> f(width);
			float sinTheta = sinf(M_PI * ((float)v + 0.5f) / (float)height);
			for (int u = 0; u < width; u++)
			{
				f[u] = envmap->texels[(v * width) + u].Lum() * sinTheta;
			}
			pConditionalV[v] = new Distribution1D(f);
			marginalFunc[v] = pConditionalV[v]->area;
		}
		pMarginal = new Distribution1D(marginalFunc);

	}
	void sampleContinuous(float u0, float u1, float& u, float& v, float& pdf) const {
		float pdf0, pdf1;
		int vIdx = pMarginal->sample(u1, pdf1);
		int uIdx = pConditionalV[vIdx]->sample(u0, pdf0);

		u = (uIdx + 0.5f) / pConditionalV[vIdx]->func.size();
		v = (vIdx + 0.5f) / pMarginal->func.size();

		pdf = pdf0 * pdf1;
	}

	float pdf(float u, float v) const {
		int iu = std::clamp(int(u * pConditionalV[0]->func.size()), 0, int(pConditionalV[0]->func.size()) - 1);
		int iv = std::clamp(int(v * pMarginal->func.size()), 0, int(pMarginal->func.size()) - 1);

		float pdfU = pConditionalV[iv]->pdf(iu);
		float pdfV = pMarginal->pdf(iv);

		return pdfU * pdfV;
	}

	~Distribution2D() {
		for (auto c : pConditionalV)
			delete c;
		delete pMarginal;
	}

};



class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		/*Vec3 wi = Vec3(0, 0, 1);
		pdf = 1.0f;
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);*/
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wi);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wi);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = evaluate(wi);
		return wi;

		//float u1 = sampler->next();
		//float u2 = sampler->next();



	}
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};