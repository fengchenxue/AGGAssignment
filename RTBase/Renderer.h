#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <mutex>
#include <queue>
#define MAX_DEPTH 5
#define TILE_SIZE 16
inline float MISWeight(float pdf1, float pdf2)
{
	float a2 = pdf1 * pdf1;
	float b2 = pdf2 * pdf2;
	return a2 / (a2 + b2 + 1e-12f);
}

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;

	std::mutex tileQueueMutex;
	std::queue<std::pair<int, int>> tileQueue;
	bool stopThreads = false;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}

	void generateTiles() {
		while (!tileQueue.empty()) {
			tileQueue.pop();
		}
		for (int y = 0; y < film->height; y += TILE_SIZE) {
			for (int x = 0; x < film->width; x += TILE_SIZE) {
				tileQueue.push(std::make_pair(x, y));
			}
		}
	}
	void renderTile(int threadID) {
		while (!stopThreads) {
			std::pair<int, int> tile;
			{
				std::lock_guard<std::mutex> lock(tileQueueMutex);
				if (tileQueue.empty()) return;
				tile = std::move(tileQueue.front());
				tileQueue.pop();
			}

			int startX = tile.first;
			int startY = tile.second;

			for (int y = startY; y < startY+TILE_SIZE && y < film->height; y++) {
				for (int x = startX; x < startX + TILE_SIZE && x < film->width; x++ ) {
					float px = x + samplers[threadID].next();
					float py = y + samplers[threadID].next();
					
					Ray ray = scene->camera.generateRay(px, py);

					Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
					Colour col = pathTrace(ray, pathThroughput, 0, &samplers[0]);

					film->splat(px, py, col);
					unsigned char r = (unsigned char)(col.r * 255);
					unsigned char g = (unsigned char)(col.g * 255);
					unsigned char b = (unsigned char)(col.b * 255);
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		}
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		Colour L_light(0.0f, 0.0f, 0.0f);
		Colour L_bsdf(0.0f, 0.0f, 0.0f);
		
		//-------light sampling-------
		// Pure specular materials and refractive materials only have bsdf sampling, NO direct lighting sampling
		if (shadingData.bsdf->isPureSpecular() == false)
		{
			// Sample a light
			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);

			// Sample a point on the light
			float pdf;
			float pdf_light;
			float pdf_bsdf;
			Colour emitted;

			//For area light, p is a point on the light. For environment light, p is the direction to the light.
			//For area light, pdf is based on area. For environment light, pdf is based on solid angle.
			Vec3 p = light->sample(sampler, emitted, pdf,shadingData.x);

			if (pdf > 1e-12) {
				Vec3 wi;
				float GTerm=0.0f;
				bool visible = false;

				if (light->isArea()){
					wi = p - shadingData.x;
					float l = wi.lengthSq();
					wi = wi.normalize();
					GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(Dot(-wi, light->normal(wi)), 0.0f)) / l;
					visible = (GTerm > 0 && scene->visible(shadingData.x+wi*EPSILON, p));
					pdf_light = pdf * pmf;
				}
				else{
					wi = p;
					GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
					visible = (GTerm > 0 && scene->visible(shadingData.x+wi*EPSILON, shadingData.x + (wi * 10000.0f)));
					pdf_light = pdf * pmf;
				}

				if (visible){
					pdf_bsdf = shadingData.bsdf->PDF(shadingData, wi);
					float W = MISWeight(pdf_light, pdf_bsdf);
					L_light = shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm * W / pdf_light;
				}
			}

			//------bsdf sampling------
			pdf = 0.0f;
			pdf_light = 0.0f;
			pdf_bsdf = 0.0f;
			Colour bsdf(0.0f, 0.0f, 0.0f);
			emitted = Colour(0.0f, 0.0f, 0.0f);
			float W = 0.0f;

			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf_bsdf);
			if (pdf_bsdf > 1e-12f) {
				Ray shadowRay(shadingData.x + (wi * EPSILON), wi);
				IntersectionData shadowIntersection = scene->traverse(shadowRay);
				ShadingData shadowShadingData = scene->calculateShadingData(shadowIntersection, shadowRay);

				if (shadowShadingData.t < FLT_MAX) {
					if (shadowShadingData.bsdf->isLight()) {
						emitted = shadowShadingData.bsdf->emit(shadowShadingData, -wi);
						Light* hitlight = scene->getLightFromTriangleID(shadowIntersection.ID);

						if (hitlight) {
							pdf = hitlight->PDF(-wi,shadingData.x,shadowShadingData.x);
							pdf_light = pdf* pmf;
							W = MISWeight(pdf_bsdf, pdf_light);
							float l = ( shadowShadingData.x - shadingData.x).lengthSq();
							float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(Dot(-wi, hitlight->normal(wi)), 0.0f)) / l;
							L_bsdf = bsdf * emitted *GTerm* W / pdf_bsdf;
						}
					}

				}
				else {
					// Environment light
					emitted = scene->background->evaluate(wi);
					pdf = scene->background->PDF(-wi, shadingData.x, shadowShadingData.x);
					pdf_light = pdf * pmf;
					W = MISWeight(pdf_bsdf, pdf_light);
					float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
					L_bsdf = bsdf * emitted * GTerm * W / pdf_bsdf;

				}
			}
		}
		//For specular material, only bsdf sampling
		else {
			float pdf_bsdf = 0.0f;
			Colour bsdf(0.0f, 0.0f, 0.0f);
			Colour emitted(0.0f, 0.0f, 0.0f);
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf_bsdf);
			if (pdf_bsdf > 1e-12f) {
				Ray shadowRay(shadingData.x + (wi * EPSILON), wi);
				IntersectionData shadowIntersection = scene->traverse(shadowRay);
				ShadingData shadowShadingData = scene->calculateShadingData(shadowIntersection, shadowRay);

				if (shadowShadingData.t < FLT_MAX) {
					if (shadowShadingData.bsdf->isLight()) {
						emitted = shadowShadingData.bsdf->emit(shadowShadingData, -wi);
						Light* hitlight = scene->getLightFromTriangleID(shadowIntersection.ID);
						if (hitlight) {
							float l = (shadowShadingData.x - shadingData.x).lengthSq();
							float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(Dot(-wi, hitlight->normal(wi)), 0.0f)) / l;
							L_bsdf = bsdf * emitted * GTerm  / pdf_bsdf;
						}
					}

				}
				else {
					// Environment light
					emitted = scene->background->evaluate(wi);
					float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
					L_bsdf = bsdf * emitted * GTerm / pdf_bsdf;

				}
			}
		}
		return  L_light + L_bsdf;
	}
	Colour pathTrace(Ray& r, Colour pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			Colour Lo(0.0f, 0.0f, 0.0f);
			//------Emissive Materials------
			if (shadingData.bsdf->isEmissive())
			{
				Lo = Lo + pathThroughput* shadingData.bsdf->emit(shadingData, shadingData.wo);
			}

			//------Direct Lighting------
			Lo = Lo + pathThroughput * computeDirect(shadingData, sampler);
			
			//Max Depth
			if (depth >= MAX_DEPTH)
			{
				return Lo;
			}
			//Russian Roulette
			float russianRouletteProbability = max (0.0f, min(pathThroughput.Lum(), 0.9f));
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return Lo;
			}

			//------Indirect Lighting------
			//need to solve glass material
			Colour bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);

			float cosTheta = Dot(wi, shadingData.sNormal);
			if (pdf > 1e-12f && cosTheta > 0.0f) {
				pathThroughput = pathThroughput * bsdf * cosTheta / pdf;
				Ray nextRay;
				nextRay.init(shadingData.x + (wi * EPSILON), wi);
				return Lo + pathTrace(nextRay, pathThroughput, depth + 1, sampler);
			}
			return Lo;
		}
		return pathThroughput * scene->background->evaluate(r.dir);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void render()
	{
		film->incrementSPP();
		//std::cout << film->SPP << std::endl;
		generateTiles();
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::renderTile, this, i);
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}

		//for (unsigned int y = 0; y < film->height; y++)
		//{
		//	for (unsigned int x = 0; x < film->width; x++)
		//	{
		//		float px = x + samplers[0].next();
		//		float py = y + samplers[0].next();	
		//		Ray ray = scene->camera.generateRay(px, py);
		//		//Colour col = viewNormals(ray);
		//		//Colour col = albedo(ray);

		//		Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
		//		Colour col = pathTrace(ray, pathThroughput, 0, &samplers[0]);
		//		
		//		film->splat(px, py, col);
		//		unsigned char r = (unsigned char)(col.r * 255);
		//		unsigned char g = (unsigned char)(col.g * 255);
		//		unsigned char b = (unsigned char)(col.b * 255);
		//		film->tonemap(x, y, r, g, b);
		//		canvas->draw(x, y, r, g, b);
		//	}
		//}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}

};