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
	// normalized and cumulative density function, ranges from 0 to 1
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
		area = cdf[func.size()];

		//robustness measure
		if (area<EPSILON){
			float uniform = 1.0f / (float)func.size();
			for (int i = 0; i < func.size(); i++) {
				func[i] = uniform;
			}
			for (int i = 0; i < func.size(); i++){
				cdf[i + 1] = (float)(1 + i) * uniform;
			}
			area = 1.0f;
		}
		else
		{
			for (int i = 1; i < cdf.size(); i++)
			{
				cdf[i] = cdf[i] / area;
			}
		}
	}
	//return the neareast index(>=u) -1 with a random input number
	int sample(float u, float &pdf)
	{
		auto in = std::lower_bound(cdf.begin(), cdf.end(), u);
		int index = std::clamp((int)(in - cdf.begin()) - 1, 0, (int)func.size()-1);
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
	// every pixel has a weight
	std::vector<Distribution1D*> pConditionalV;
	// the total weight of each row
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
			// +0.5f because v starts from 0 rather than 1
			float sinTheta = sinf(M_PI * ((float)v + 0.5f) / (float)height);
			for (int u = 0; u < width; u++){
				// *sinTheta because the weight of the top and bottom of the sphere should be smaller than the middle,
				// or the samples will be too concentrated in the top and bottom
				// dA=sin(theta)d(theta)d(phi) , so the weight should multiple sin(theta) 
				f[u] = envmap->texels[(v * width) + u].Lum() * sinTheta;
			}
			pConditionalV[v] = new Distribution1D(f);
			marginalFunc[v] = pConditionalV[v]->area;
		}
		pMarginal = new Distribution1D(marginalFunc);

	}
	//input: 2 random numbers
	//output: the normalized position of the sample in the texture and PDF
	void sampleContinuous(float u1, float u2, float u3, float u4, float& u, float& v, float& pdf) const {
		float pdf1, pdf2;
		int vIdx = pMarginal->sample(u2, pdf2);
		int uIdx = pConditionalV[vIdx]->sample(u1, pdf1);

		float width = pConditionalV[vIdx]->func.size();
		float height = pMarginal->func.size();

		//normalize the position to [0,1]
		u = (uIdx + u3) / width;
		v = (vIdx + u4) / height;
		// Discrete PDF to continuous PDF
		pdf = pdf1 * pdf2 * width * height;
	}

	float pdf(float u, float v) const {
		int iv = std::clamp(int(v * pMarginal->func.size()), 0, int(pMarginal->func.size() - 1));
		int iu = std::clamp(int(u * pConditionalV[iv]->func.size()), 0, int(pConditionalV[iv]->func.size()-1));
		

		float pdfU = pConditionalV[iv]->pdf(iu);
		float pdfV = pMarginal->pdf(iv);

		float width = pConditionalV[iv]->func.size();
		float height = pMarginal->func.size();
		return pdfU * pdfV * width * height;
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
	virtual Vec3 sample(Sampler* sampler, Colour& emittedColour, float& pdf, const Vec3& startPos = Vec3()) = 0;
	//wi is toward the light
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const Vec3& wi, const Vec3& startPos = Vec3(), const Vec3& TargetPos = Vec3()) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	//return wo
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
	//used for PPM to get the photon flux
	virtual Colour getTotalPhotonFlux(const Vec3& wi) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(Sampler* sampler, Colour& emittedColour, float& pdf, const Vec3& startPos = Vec3())
	{
		emittedColour = emission;
		Vec3 pos= triangle->sample(sampler, pdf);

		float l2 = (startPos - pos).lengthSq();
		Vec3 wo = (startPos - pos).normalize();
		
		float cosTheta = Dot(wo, triangle->gNormal());
		if (cosTheta < EPSILON) { 
			pdf = 0.0f;
			return Vec3(0.0f, 0.0f, 0.0f); 
		}
		pdf = l2 / (triangle->area * cosTheta);
		return pos;
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal())< 0.0f)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const Vec3& wi, const Vec3& startPos = Vec3(), const Vec3& TargetPos = Vec3())
	{
		float l2 = (TargetPos - startPos).lengthSq();
		float cosTheta = Dot(-wi, triangle->gNormal());
		if (cosTheta < EPSILON) return 0.0f;
		return l2 / (triangle->area * cosTheta);
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum()) * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wo = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wo);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wo);
	}
	//flux=pi*area*radiance
	Colour getTotalPhotonFlux(const Vec3& wi)
	{
		return evaluate(wi) * triangle->area * M_PI;
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
	Vec3 sample(Sampler* sampler, Colour& reflectedColour, float& pdf, const Vec3& startPos = Vec3())
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
	float PDF(const Vec3& wi, const Vec3& startPos = Vec3(), const Vec3& TargetPos = Vec3())
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const Vec3& wi)
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
		pdf = 1/4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return -wi;
	}
	Colour getTotalPhotonFlux(const Vec3& wi)
	{
		return evaluate(wi) * 4.0f * M_PI;

	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	Distribution2D* distribution;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
		distribution = new Distribution2D(env);
	}
	~EnvironmentMap()
	{
		delete distribution;
	}
	//return wi toward the light
	Vec3 sample(Sampler* sampler, Colour& reflectedColour, float& pdf, const Vec3& startPos = Vec3())
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		float mapPDF;
		float u, v;
		distribution->sampleContinuous(sampler->next(), sampler->next(), sampler->next(), sampler->next(),u, v, mapPDF);

		float theta = v * M_PI;
		float phi = u * 2.0f * M_PI;

		float sinTheta = std::max(sinf(theta),EPSILON);
		Vec3 wi = Vec3(cosf(phi) * sinTheta, cosf(theta), sinf(phi) * sinTheta);
		
		//mapPDF is PDF_u * PDF_v,that means mapPDF=dP/(dv*du). 
		//Now we need to get dP/dw. dw=2*pi*pi*sin(theta)*du*dv
		//dP/dw= dP/(dv*du) /(2*pi*pi*sin(theta)). So the final PDF is density function of a solid angle.
		pdf = (sinTheta <= EPSILON) ? 0.0f : mapPDF / (2.0f * M_PI * M_PI * sinTheta);
		reflectedColour = evaluate(wi);

		return wi;
	}
	//u islongitude, v is latitude. They are normalized to [0,1]
	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const Vec3& wi, const Vec3& startPos = Vec3(), const Vec3& TargetPos = Vec3())
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u/ (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;

		float sinTheta = sin(v * M_PI);
		if (sinTheta< EPSILON) return 0.0f;

		float mapPdf = distribution->pdf(u, v);
		return mapPdf / (2.0f * M_PI * M_PI * sinTheta);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const Vec3& wi)
	{
		return -wi;
	}
	//power = 2*pi*pi/width/height*sum(Lum*sin(theta))
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i+0.5) * M_PI / (float)env->height);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 2.0f * M_PI * M_PI;
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

		Colour emittedColour;
		return -sample(sampler, emittedColour, pdf);
	}
	// flux=Le*2*pi*pi
	Colour getTotalPhotonFlux(const Vec3& wi)
	{
		return evaluate(wi) * 2.0f * M_PI * M_PI;
	}
};