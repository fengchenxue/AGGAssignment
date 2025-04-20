#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

#define EMISSIVE_LUMINANCE_THRESHOLD 0.5f

//v = incident direction pointing toward the surface
//n = surface normal pointing outward
//for all coordinate systems
inline Vec3 reflect(const Vec3& v, const Vec3& n)
{
	return n * 2.0f * v.dot(n) - v;
}

inline Vec3 refract(const Vec3& wi, float eta, float cosThetaT) {

	return Vec3(-eta * wi.x, -eta * wi.y, -cosThetaT);
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
	static float fresnelDielectric(float cosThetaI, float cosThetaT, float eta)
	{
		// Add code here
		float rs = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
		float rp = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
		return 0.5f * (rs * rs + rp * rp);

	}
	static Colour fresnelConductor(float cosTheta, Colour eta, Colour k)
	{
		// Add code here
		float cosTheta2 = cosTheta * cosTheta;
		float sinTheta2 = 1.0f - cosTheta2;
		Colour eta2 = eta * eta;
		Colour k2 = k * k;

		// Compute Rs (perpendicular component)
		Colour rs_num = eta2 + k2 - eta * 2 * cosTheta + Colour(cosTheta2, cosTheta2, cosTheta2);
		Colour rs_den = eta2 + k2 + eta * 2 * cosTheta + Colour(cosTheta2, cosTheta2, cosTheta2);
		Colour Rs = rs_num / rs_den;

		// Compute Rp (parallel component)
		Colour rp_num = (eta2 + k2) * cosTheta2 - eta * 2 * cosTheta + Colour(sinTheta2, sinTheta2, sinTheta2);
		Colour rp_den = (eta2 + k2) * cosTheta2 + eta * 2 * cosTheta + Colour(sinTheta2, sinTheta2, sinTheta2);
		Colour Rp = rp_num / rp_den;

		// Average for unpolarized light
		return (Rs + Rp) * 0.5f;
	}
	static float lambdaGGX(const Vec3 & v, float alpha)
	{
		// Add code here
		float cosTheta = std::abs(v.z);
		float sinTheta2 = std::max(1.0f - cosTheta * cosTheta, 0.0f);
		float tanTheta2 = sinTheta2 / (cosTheta * cosTheta + EPSILON);
		float alpha2 = alpha * alpha;

		return (-1.0f + std::sqrt(1.0f + alpha2 * tanTheta2)) * 0.5f;
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
	static float schlick_avy(float F0)
	{
		return F0 + (1.0f - F0) * 0.5f;
	}
};

class BSDF
{
public:
	Colour emission;
	//toward outside
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	//wi toward outside
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

		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = Vec3(-wolocal.x, -wolocal.y, wolocal.z);

		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
		return shadingData.frame.toWorld(wilocal).normalize();
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = Vec3(-wolocal.x, -wolocal.y, wolocal.z);
		Vec3 wiWorld = shadingData.frame.toWorld(wilocal).normalize();
		if (Dot(wiWorld, wi) > 0.999999f) {
			return albedo->sample(shadingData.tu, shadingData.tv);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = Vec3(-wolocal.x, -wolocal.y, wolocal.z);
		Vec3 wiWorld = shadingData.frame.toWorld(wilocal).normalize();
		if (Dot(wiWorld, wi) > 0.999999f) {
			return 1.0f;
		}
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
		
		Vec3 wilocal = reflect(wolocal, h).normalize();
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

		return shadingData.frame.toWorld(wilocal).normalize();
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
	//intIOR means medium IOR such as 1.33. extIOR means external IOR such as 1.0
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
		//Vec3 normal(0, 0, 1);

		//check if entering
		bool entering = wolocal.z < 0.0f;
		float iorI = entering ? extIOR : intIOR;
		float iorT = entering ? intIOR : extIOR;
		float eta = iorI / iorT;
		//normal = entering ? -normal : normal;
		
		//compute angle
		float cosThetaT, sin2ThetaT, sin2ThetaI, cosThetaI;
		cosThetaT = fabs(wolocal.z);
		sin2ThetaT = 1.0f - cosThetaT * cosThetaT;
		sin2ThetaI = sin2ThetaT / (eta * eta);
		// Handle total internal reflection
		float F;
		if (sin2ThetaI >= 1.0f) { 
			// Total internal reflection
			cosThetaI = 0.0f; 
			F = 1.0f;
		}
		else { 
			cosThetaI = sqrt(1.0f - sin2ThetaI);
			F = ShadingHelper::fresnelDielectric(cosThetaI, cosThetaT, eta);
		}
		
		//sample
		Vec3 wi;
		if (F >= 1.0f|| sampler->next() < F) {
			// Reflection
			Vec3 wiLocal = reflect(wolocal, Vec3(0,0,1)).normalize();
			wi = shadingData.frame.toWorld(wiLocal);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F;
			pdf = std::clamp(F, 0.0f, 1.0f);
		}
		else { 
			// Refraction
			//Vec3 wiLocal = refract(wolocal, 1/eta,cosThetaI).normalize();
			Vec3 wiLocal = Vec3(-wolocal.x/eta,
				-wolocal.y/eta,
				entering ? cosThetaI : -cosThetaI).normalize();
			wi = shadingData.frame.toWorld(wiLocal);
			// Account for solid angle compression
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1 - F) / (eta * eta * fabs(wolocal.z));
			pdf = 1 - F;
			
		}
		return wi.normalize();
	}	
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();

		//check if entering
		bool entering = wolocal.z < 0.0f;
		float iorI = entering ? extIOR : intIOR;
		float iorT = entering ? intIOR : extIOR;
		float eta = iorI / iorT;

		//compute angle
		float cosThetaT, sin2ThetaT, sin2ThetaI, cosThetaI;
		cosThetaT = fabs(wolocal.z);
		sin2ThetaT = 1.0f - cosThetaT * cosThetaT;
		sin2ThetaI = sin2ThetaT / (eta * eta);
		// Handle total internal reflection
		float F;
		if (sin2ThetaI >= 1.0f) {
			// Total internal reflection
			cosThetaI = 0.0f;
			F = 1.0f;
		}
		else {
			cosThetaI = sqrt(1.0f - sin2ThetaI);
			F = ShadingHelper::fresnelDielectric(cosThetaI, cosThetaT, eta);
		}

		// Reflection
		Vec3 wiLocal = reflect(wolocal, Vec3(0, 0, 1)).normalize();
		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal).normalize();
		if (Dot(wiWorld, wi) > 0.999999f) {
			return albedo->sample(shadingData.tu, shadingData.tv) * F;
		}

		// Refraction
		wiLocal = Vec3(-wolocal.x / eta,
			-wolocal.y / eta,
			entering ? cosThetaI : -cosThetaI).normalize();
		wiWorld = shadingData.frame.toWorld(wiLocal).normalize();
		if (Dot(wiWorld, wi) > 0.999999f) {
			return albedo->sample(shadingData.tu, shadingData.tv) * (1 - F) / (eta * eta * fabs(wolocal.z));
		}

		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlatssPDF
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();

		//check if entering
		bool entering = wolocal.z < 0.0f;
		float iorI = entering ? extIOR : intIOR;
		float iorT = entering ? intIOR : extIOR;
		float eta = iorI / iorT;

		//compute angle
		float cosThetaT, sin2ThetaT, sin2ThetaI, cosThetaI;
		cosThetaT = fabs(wolocal.z);
		sin2ThetaT = 1.0f - cosThetaT * cosThetaT;
		sin2ThetaI = sin2ThetaT / (eta * eta);
		// Handle total internal reflection
		float F;
		if (sin2ThetaI >= 1.0f) {
			// Total internal reflection
			cosThetaI = 0.0f;
			F = 1.0f;
		}
		else {
			cosThetaI = sqrt(1.0f - sin2ThetaI);
			F = ShadingHelper::fresnelDielectric(cosThetaI, cosThetaT, eta);
		}

		// Reflection
		Vec3 wiLocal = reflect(wolocal, Vec3(0, 0, 1)).normalize();
		Vec3 wiWorld = shadingData.frame.toWorld(wiLocal).normalize();
		if (Dot(wiWorld, wi) > 0.999999f) {
			return std::clamp(F, 0.0f, 1.0f);
		}

		// Refraction
		wiLocal = Vec3(-wolocal.x / eta,
			-wolocal.y / eta,
			entering ? cosThetaI : -cosThetaI).normalize();
		wiWorld = shadingData.frame.toWorld(wiLocal).normalize();
		if (Dot(wiWorld, wi) > 0.999999f) {
			return (1.0-F);
		}
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
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		
		//check if entering
		bool entering = wolocal.z < 0.0f;
		float iorI = entering ? extIOR : intIOR;
		float iorT = entering ? intIOR : extIOR;
		float eta = iorI / iorT;

		Vec3 h = SamplingDistributions::ggxSampleHemisphere(sampler->next(), sampler->next(), alpha);
		float WoDotH = Dot(wolocal, h);
		if (WoDotH <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		float F = ShadingHelper::fresnelDielectric(WoDotH, Dot(h, Vec3(0, 0, 1)), eta);
		
		Vec3 wilocal;
		Vec3 wi;
		if (sampler->next() < F) {
			// Reflection
			Vec3 wiLocal = reflect(wolocal, h).normalize();
			wi = shadingData.frame.toWorld(wiLocal).normalize();

			float D = ShadingHelper::Dggx(h, alpha);
			float G = ShadingHelper::Gggx(wilocal, wolocal, alpha);
			float denominator = 4.0f * std::abs(wolocal.z * wilocal.z);

			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * F * D * G / denominator;
			pdf = F * D * h.z / (4.0f * WoDotH);
		}
		else {
			// Refraction
			float sin2ThetaI = std::max(0.0f, 1.0f - WoDotH * WoDotH);
			float sin2ThetaT = sin2ThetaI *eta * eta;

			if (sin2ThetaT >= 1.0f) {
				pdf = 0;
				reflectedColour = Colour(0.0f, 0.0f, 0.0f);
				return Vec3(0.0f, 0.0f, 0.0f);
			}
			float cosThetaT = sqrt(1.0f - sin2ThetaT);
			Vec3 tdir = (-wolocal*eta + h* (eta * WoDotH - cosThetaT)).normalize();
			if (tdir.z <= 0.0f) {
				pdf = 0;
				reflectedColour = Colour(0.0f, 0.0f, 0.0f);
				return Vec3(0.0f, 0.0f, 0.0f);
			}
			float D = ShadingHelper::Dggx(h, alpha);
			float G = ShadingHelper::Gggx(wilocal, wolocal, alpha);
			float denominator = std::abs(WoDotH + eta * Dot(tdir, h));
			float factor = eta * eta * Dot(tdir, h) / (WoDotH * denominator * denominator);
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1 - F) * D * G * factor / tdir.z;
			pdf = (1 - F) * D * h.z * Dot(tdir, h)*eta*eta / (denominator*denominator);
			wilocal = tdir;
			wi = shadingData.frame.toWorld(wilocal).normalize();
		}
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 0.0f;
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
		Vec3 wilocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wilocal.z / M_PI;
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();

		float sigma2 = sigma * sigma;
		float A = 1.0f - (sigma2 / (2.0f * (sigma2 + 0.33f)));
		float B = 0.45f * sigma2 / (sigma2 + 0.09f);

		float cosThetaI = wilocal.z;
		float cosThetaO = wolocal.z;
		
		float thetaI = acosf(cosThetaI);
		float thetaO = acosf(cosThetaO);

		float phiI = atan2f(wilocal.y, wilocal.x);
		float phiO = atan2f(wolocal.y, wolocal.x);

		float term = A + B * std::max(0.0f, cosf(phiI - phiO)) * sinf(std::max(thetaI, thetaO)) * tanf(std::min(thetaI, thetaO));
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * term / M_PI;

		return shadingData.frame.toWorld(wilocal).normalize();
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		Vec3 wilocal = shadingData.frame.toLocal(wi).normalize();
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();

		if (wilocal.z <= 0.0f || wolocal.z <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);

		float sigma2 = sigma * sigma;
		float A = 1.0f - (sigma2 / (2.0f * (sigma2 + 0.33f)));
		float B = 0.45f * sigma2 / (sigma2 + 0.09f);

		float cosThetaI = wilocal.z;
		float cosThetaO = wolocal.z;

		float thetaI = acosf(cosThetaI);
		float thetaO = acosf(cosThetaO);

		float phiI = atan2f(wilocal.y, wilocal.x);
		float phiO = atan2f(wolocal.y, wolocal.x);

		float term = A + B * std::max(0.0f, cosf(phiI - phiO)) * sinf(std::max(thetaI, thetaO)) * tanf(std::min(thetaI, thetaO));
		return albedo->sample(shadingData.tu, shadingData.tv) * term / M_PI;
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

		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		if (wolocal.z <= 0.0f) {
			pdf = 0;
			reflectedColour = Colour(0.0f, 0.0f, 0.0f);
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		float eta = intIOR / extIOR;
		float F0 = powf((1.0f - eta) / (1.0f + eta),2);
		float Fav = ShadingHelper::schlick_avy(F0);

		if (sampler->next() < Fav) {
			// Specular part
			Vec3 wrlocal = Vec3(-wolocal.x, -wolocal.y, wolocal.z);
			Frame coatFrame; 
			coatFrame.fromVector(wrlocal);

			float e = alphaToPhongExponent();
			float s1 = sampler->next();
			float s2 = sampler->next();
			float cosTheta = powf(s1, 1.0f / (e + 1.0f));
			float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
			float phi = s2 * 2.0f * M_PI;

			Vec3 lobelocal = Vec3(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
			Vec3 wi = coatFrame.toWorld(lobelocal).normalize();
			if (wi.z <= 0.0f) {
				pdf = 0;
				reflectedColour = Colour(0.0f, 0.0f, 0.0f);
				return Vec3(0.0f, 0.0f, 0.0f);
			}

			float IRDot = powf(std::max(0.0f, Dot(wi,wrlocal)),e);
			pdf = (e + 1.0f) * IRDot / (2.0f * M_PI) * Fav + (1.0f - Fav) * SamplingDistributions::cosineHemispherePDF(wi);
			float col=(e + 2.0f) * IRDot / (2.0f * M_PI)*Fav;
			reflectedColour = Colour(col, col, col);
			return shadingData.frame.toWorld(wi).normalize();
		}
		else {
			// Diffuse part
			Vec3 wilocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			pdf = (1 - Fav) * wilocal.z / M_PI;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1-Fav) / M_PI;
			return shadingData.frame.toWorld(wilocal).normalize();
		}
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = shadingData.frame.toLocal(wi).normalize();
		if (wilocal.z <= 0.0f || wolocal.z <= 0.0f) return Colour(0.0f, 0.0f, 0.0f);
		Vec3 wrlocal = Vec3(-wolocal.x, -wolocal.y, wolocal.z).normalize();
		
		float eta = intIOR / extIOR;
		float F0 = powf((1.0f - eta) / (1.0f + eta), 2);
		float Fav = ShadingHelper::schlick_avy(F0);
		
		float e = alphaToPhongExponent();
		float IRDot = powf(std::max(0.0f, Dot(wilocal, wrlocal)), e);
		float specular = (e + 2.0f) * IRDot / (2.0f * M_PI);
		float diffuse = (1 - Fav) / M_PI;
		return albedo->sample(shadingData.tu, shadingData.tv) * diffuse + Colour(specular, specular, specular) * Fav;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wolocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wilocal = shadingData.frame.toLocal(wi).normalize();
		if (wilocal.z <= 0.0f || wolocal.z <= 0.0f) return 0.0f;
		Vec3 wrlocal = reflect(wolocal, Vec3(0, 0, 1)).normalize();

		float eta = intIOR / extIOR;
		float F0 = powf((1.0f - eta) / (1.0f + eta), 2);
		float Fav = ShadingHelper::schlick_avy(F0);

		float e = alphaToPhongExponent();
		float IRDot = powf(std::max(0.0f, Dot(wilocal, wrlocal)), e);
		float specularpdf = (e + 1.0f) * IRDot / (2.0f * M_PI);
		float diffusepdf = wilocal.z / M_PI;
		return (1 - Fav) * diffusepdf + Fav * specularpdf;
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