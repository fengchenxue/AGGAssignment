#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

inline Vec3 reflect(const Vec3& v, const Vec3& n)
{
	return v - n * 2.0f * v.dot(n);
}


class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorI, float iorT)
	{
		// Add code here
		cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
		bool entering = cosTheta > 0.0f;
		if (!entering) {
			std::swap(iorI, iorT);
			cosTheta = std::abs(cosTheta);
		}

		float sinThetaI = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
		float sinThetaT = (iorI / iorT) * sinThetaI;

		if (sinThetaT >= 1.0f) return 1.0f;

		float cosThetaT = std::sqrt(std::max(0.0f, 1.0f - sinThetaT * sinThetaT));
		float rParallel = (iorT * cosTheta - iorI * cosThetaT) / (iorT * cosTheta + iorI * cosThetaT);
		float rPerpendicular = (iorI * cosTheta - iorT * cosThetaT) / (iorI * cosTheta + iorT * cosThetaT);
		return 0.5f * (rParallel * rParallel + rPerpendicular * rPerpendicular);

	}
	static Colour fresnelConductor(float cosTheta, Colour eta, Colour k)
	{
		// Add code here
		float cosTheta2 = cosTheta * cosTheta;
		Colour eta2 = eta * eta;
		Colour k2 = k * k;
		Colour cos2 = Colour(cosTheta2, cosTheta2, cosTheta2);
		Colour sin2 = Colour(1.0f, 1.0f, 1.0f) - cos2;
		Colour tan2 = sin2 / cos2;

		Colour term1 = (eta2 + k2 - eta * cosTheta * 2.0f + cos2)/
			(eta2 + k2 + eta * cosTheta * 2.0f + cos2);

		Colour numerator = eta2 * 4.0f * k2 * sin2 * tan2;
		Colour denominator = (eta2 + k2 + eta * 2.0f * cosTheta + cos2);
		Colour term2 = numerator / (denominator*denominator);
		return term1 + term2;
	}
	static float lambdaGGX(const Vec3 & v, float alpha)
	{
		// Add code here
		float cosTheta = std::abs(v.z);
		float sinTheta2 = std::max(1.0f - cosTheta * cosTheta, 0.0f);
		float tanTheta2 = sinTheta2 / (cosTheta * cosTheta + 1e-6f);
		float alpha2 = alpha * alpha;

		return (-1.0f + std::sqrtf(1.0f + alpha2 * tanTheta2)) * 0.5f;
	}
	static float G1GGX(const Vec3 &v, float alpha)
	{
		float cosTheta = v.z;
		if (cosTheta <= 0.0f) return 0.0f;

		float alpha2 = alpha * alpha;
		float tan2 = (1.0f - cosTheta * cosTheta) / (cosTheta * cosTheta);
		return 2.0f / (1.0f + sqrtf(1.0f + alpha2 * tan2));
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		
		// two ways to calculate G
		//return G1GGX(wi, alpha) * G1GGX(wo, alpha);
		return 1.0f / (1.0f + lambdaGGX(wi, alpha) + lambdaGGX(wo, alpha));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		float cosTheta2 = h.z * h.z;
		float alpha2 = alpha * alpha;
		float variable = cosTheta2 * (alpha2 - 1.0f) + 1.0f;
		return alpha2 / (M_PI * variable * variable);
	}
};
#define EMISSIVE_LUMINANCE_THRESHOLD 0.5f

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	//if lum >= 1, it is a light source, if 1 > lum > 0, it is emissive material
	bool isLight()
	{
		return emission.Lum() >= EMISSIVE_LUMINANCE_THRESHOLD ? true : false;
	}
	bool isEmissive()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	//light source emission
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 wilocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wilocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code

		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wilocal = Vec3(-wolocal.x, -wolocal.y, wolocal.z);

		pdf = 1.0f;

		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);

		return shadingData.frame.toWorld(wilocal);

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code

		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		if (wolocal.z <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}
		Vec3 h = SamplingDistributions::ggxSampleHemisphere(sampler->next(), sampler->next(), alpha);
		
		//-wolocal because relect function regards w as incoming direction
		Vec3 wilocal = reflect(-wolocal, h).normalize();
		if (wilocal.z <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		//Fresnel
		Colour F = ShadingHelper::fresnelConductor(Dot(wilocal, h), eta, k);

		//PDF
		float D = ShadingHelper::Dggx(h, alpha);
		float cosTheta = Dot(wolocal, h);
		if (cosTheta <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}
		pdf = D * h.z / (4.0f * cosTheta);

		//BRDF
		float G = ShadingHelper::Gggx(wilocal, wolocal, alpha);

		float denominator = 4.0f * wolocal.z * wilocal.z;

		reflectedColour = F * D * G / denominator;

		return shadingData.frame.toWorld(wilocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = shadingData.frame.toLocal(wi).normalize();
		if (wilocal.z <= 0.0f || wolocal.z <= 0.0f) return Colour(0.0f,0.0f,0.0f);
		Vec3 h = (wolocal + wilocal).normalize();

		Colour F = ShadingHelper::fresnelConductor(Dot(wilocal, h), eta, k);
		float D = ShadingHelper::Dggx(h, alpha);
		float G = ShadingHelper::Gggx(wilocal, wolocal, alpha);

		float denominator = 4.0f * wolocal.z * wilocal.z;
		return  F * D * G / denominator;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF

		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = shadingData.frame.toLocal(wi).normalize();
		if (wilocal.z <= 0.0f || wolocal.z<=0.0f) return 0.0f;
		Vec3 h = (wolocal + wilocal).normalize();

		float D = ShadingHelper::Dggx(h, alpha);
		float cosTheta = Dot(wolocal, h);
		if (cosTheta <= 0.0f) {
			return 0.0f;
		}
		float pdf = D * h.z / (4.0f * cosTheta);
		return pdf;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code

		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 normal(0, 0, 1);
		float exaI = extIOR, exaE = intIOR;
		//float exaI = intIOR, exaE = extIOR;
		bool entering = wolocal.z < 0.0f;

		//if entering,  Wo enters the material. if not, Wo leaves the material
		if (!entering) { 
			std::swap(exaI, exaE);
			normal = -normal;
			
		}

		float eta = exaI / exaE;
		float cosThetaO = std::abs(wolocal.z);

		// Compute Fresnel term
		float F = ShadingHelper::fresnelDielectric(cosThetaO, exaI, exaE);

		// Handle total internal reflection
		float sin2ThetaI = 1.0f - cosThetaO * cosThetaO;
		float sin2ThetaT = eta * eta * sin2ThetaI;
		if (sin2ThetaT >= 1.0f) {
			F = 1.0f;
		}

		Vec3 wi;
		if (F >= 1.0f) { // Total reflection
			// Reflect the outgoing direction
			Vec3 wiLocal = wolocal - normal*2.0f * wolocal.z ;
			wi = shadingData.frame.toWorld(wiLocal);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F;
			pdf = 1.0f;
		}
		else {
			float rand = sampler->next();
			if (rand < F) { // Reflect
				Vec3 wiLocal = wolocal - normal * 2.0f * wolocal.z ;
				wi = shadingData.frame.toWorld(wiLocal);
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F;
				pdf = F;
			}
			else { // Refract
				// Compute refraction direction
				Vec3 incidentDir = -wolocal;
				float cosThetaI = Dot(incidentDir, normal);
				float k = 1.0f - eta * eta * (1.0f - cosThetaI * cosThetaI);
				if (k < 0.0f) { // Total reflection (shouldn't happen here)
					Vec3 wiLocal = wolocal - normal*2.0f * wolocal.z ;
					wi = shadingData.frame.toWorld(wiLocal);
					reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F;
					pdf = F;
				}
				else {
					float cosThetaT = std::sqrt(k);
					Vec3 wiLocal = incidentDir *eta + normal*(eta * cosThetaI - cosThetaT) ;
					wiLocal = wiLocal.normalize();
					wi = shadingData.frame.toWorld(wiLocal);
					// Account for solid angle compression
					reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1 - F) * (1.0f/(eta * eta));
					pdf = (1 - F) * (eta * eta) * abs(Dot(wiLocal, normal)) / abs(Dot(wolocal, normal));
				}
			}
		}
		return wi;
	}	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		//return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		/*Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);*/
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
		//Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		//float exaI = intIOR, exaE = extIOR;
		//bool entering = wolocal.z > 0.0f;
		////if entering, light enters the material. if not, light leaves the material
		//if (!entering) std::swap(exaI, exaE);

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};