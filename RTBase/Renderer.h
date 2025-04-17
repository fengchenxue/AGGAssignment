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
#include<atomic>

#define MAX_DEPTH 5
#define TILE_SIZE 16

//this value is used to for adaptive sampling,
//if the spp of a tile is more than this value, it will be considered whether converged
#define MIN_TILE_SPP 8
#define ADAPTIVE_SAMPLING_THRESHOLD 0.00001f

//these values are used for PPM
#define PPM_MAX_RAY 4
// PPM_MAX_BOUNCE=3 means a ray from the carmera can bounce 3 times
#define PPM_MAX_BOUNCE 3
#define PPM_photonsPerIteration 1000
#define PPM_INITIAL_RADIUS 1.0f
#define PPM_alpha 0.7f
#define PPM_MAX_DEPTH 5

inline float MISWeight(float pdf1, float pdf2)
{
	float a2 = pdf1 * pdf1;
	float b2 = pdf2 * pdf2;
	return a2 / (a2 + b2 + 1e-12f);
}
struct TileInfo
{
	int x, y;
	//the following values are used for adaptive sampling
	int SPP = 0;
	Vec3 mean = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 meanSquared = Vec3(0.0f, 0.0f, 0.0f);
	int convergedCount = 0;
	bool stop = false;
};
//used for PPM to generate hitpoints from screen
struct HitPoint{
	Vec3 position;
	Vec3 normal;
	//temporary flux
	Colour flux = Colour(0.0f, 0.0f, 0.0f);
	//accumulated flux
	Colour tau = Colour(0.0f, 0.0f, 0.0f);
	Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
	float radius2 = PPM_INITIAL_RADIUS * PPM_INITIAL_RADIUS;
	int photonCount = 0;
	int totalPhotonCount = 0;
	int depth = 0;
};

struct KDNode {
	HitPoint* hitpoint;
	KDNode* left = nullptr;
	KDNode* right = nullptr;
	int axis = -1;
	KDNode(HitPoint* hp, int ax): hitpoint(hp), axis(ax) {}
};

class KDTree {
public:
	KDNode* root = nullptr;
	void buildTree(std::vector<HitPoint*>& hitPoints) {
		if (hitPoints.empty()) return;
		root = build(hitPoints, 0);
	}

	void traverse(KDNode* node, const Vec3& pos, const Vec3& RayDir, std::vector<std::pair<HitPoint*, Colour>>& res, const Colour & col) {
		if (!node) return;
		// Check if the current node is within the radius of the hitpoint
		float dist2 = (node->hitpoint->position - pos).lengthSq();
		if (dist2 <= node->hitpoint->radius2 && node->hitpoint->normal.dot(RayDir) < 0.0f) {

			res.push_back(std::make_pair(node->hitpoint, col));
			//node->hitpoint->photonCount++;
			//node->hitpoint->flux = node->hitpoint->flux + col;
		}

		// Check which side of the current node to traverse
		float diff = pos.coords[node->axis] - node->hitpoint->position.coords[node->axis];
		float diff2 = diff * diff;
		if (diff < 0) {
			traverse(node->left, pos, RayDir, res,col);
			if (diff2 <= node->hitpoint->radius2) {
				traverse(node->right, pos, RayDir, res,col);
			}
		}
		else {
			traverse(node->right, pos, RayDir, res,col);
			if (diff2 <= node->hitpoint->radius2) {
				traverse(node->left, pos, RayDir, res,col);
			}
		}
	}

	~KDTree() {
		destroyTree(root);
	}

private:
	KDNode* build(std::vector<HitPoint*>& hitPoints, int depth) {
		if (hitPoints.empty()) return nullptr;
		int axis = depth % 3;
		std::sort(hitPoints.begin(), hitPoints.end(), [axis](HitPoint* a, HitPoint* b) {
			return a->position.coords[axis] < b->position.coords[axis];
			});

		size_t medianIndex = hitPoints.size() / 2;
		KDNode* node = new KDNode(hitPoints[medianIndex], axis);
		std::vector<HitPoint*> leftPoints(hitPoints.begin(), hitPoints.begin() + medianIndex);
		std::vector<HitPoint*> rightPoints(hitPoints.begin() + medianIndex + 1, hitPoints.end());

		node->left = build(leftPoints, depth + 1);
		node->right = build(rightPoints, depth + 1);
		return node;
	}

	void destroyTree(KDNode* node) {
		if (node) {
			destroyTree(node->left);
			destroyTree(node->right);
			delete node;
		}
	}
};

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	//multithreading
	std::mutex tileQueueMutex;
	std::queue<TileInfo*> tileQueue;
	bool stopThreads = false;
	std::vector<TileInfo> tiles;
	//PPM
	std::vector<std::vector<HitPoint>> hitPoints;
	std::unique_ptr<KDTree> kdTree = std::make_unique<KDTree>();
	std::atomic<int> photonCount = 0;
	Colour* DirectLight;
	//store results of PPM photon pass, then it will be calculated in the main thread
	std::vector<std::pair<HitPoint*, Colour>> cacheHP[64];

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		clear();

		//initialize tiles
		for (int y = 0; y < film->height; y += TILE_SIZE) {
			for (int x = 0; x < film->width; x += TILE_SIZE) {
				TileInfo tile;
				tile.x = x;
				tile.y = y;
				tiles.push_back(tile);
			}
		}
	}
	void clear()
	{
		film->clear();
	}

	void generateTiles() {
		while (!tileQueue.empty()) {
			tileQueue.pop();
		}
		for (auto& tile : tiles) {
			if (tile.stop) continue;
			TileInfo* newTile = &tile;
			tileQueue.push(newTile);

		}
	}
	void renderTile(int threadID) {
		while (!stopThreads) {
			TileInfo* tile;
			{
				std::lock_guard<std::mutex> lock(tileQueueMutex);
				if (tileQueue.empty()) return;
				tile = std::move(tileQueue.front());
				tileQueue.pop();
			}
			int startX = tile->x;
			int startY = tile->y;

			Vec3 tileSum = Vec3(0.0f, 0.0f, 0.0f);
			Vec3 tileSumSquared = Vec3(0.0f, 0.0f, 0.0f);
			int sampleCount = 0;

			for (int y = startY; y < startY + TILE_SIZE && y < film->height; y++) {
				for (int x = startX; x < startX + TILE_SIZE && x < film->width; x++) {
					float px = x + samplers[threadID].next();
					float py = y + samplers[threadID].next();

					Ray ray = scene->camera.generateRay(px, py);

					Colour pathThroughput = Colour(1.0f, 1.0f, 1.0f);
					Colour col = pathTraceMIS(ray, pathThroughput, 0, &samplers[threadID]);
					//Colour col =pathTrace(ray, pathThroughput, 0, &samplers[threadID]);
					film->vecSPP[y * film->width + x]++;
					film->splat(px, py, col);

					//adaptive sampling
					Vec3 pixel(col.r, col.g, col.b);
					tileSum = tileSum + pixel;
					tileSumSquared = tileSumSquared + pixel * pixel;
					sampleCount++;

				}
			}
			//save adaptive sampling information
			tile->SPP += 1;
			tile->mean = tile->mean + tileSum / sampleCount;
			tile->meanSquared = tile->meanSquared + tileSumSquared / sampleCount;

			//converge check
			if (tile->SPP >= MIN_TILE_SPP) {
				Vec3 mean = tile->mean / tile->SPP;
				Vec3 meanSquared = tile->meanSquared / tile->SPP;
				Vec3 variance = meanSquared - mean * mean;
				float maxvariance = max(variance.x, max(variance.y, variance.z));

				if (maxvariance < ADAPTIVE_SAMPLING_THRESHOLD) {
					tile->convergedCount++;
				}
				else {
					tile->convergedCount = 0;
				}
				if (tile->convergedCount >= 3) {
					tile->stop = true;
				}
			}

		}
	}
	Colour computeDirectMIS(ShadingData shadingData, Sampler* sampler)
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
			Vec3 p = light->sample(sampler, emitted, pdf, shadingData.x);

			if (pdf > 1e-12) {
				Vec3 wi;
				float GTerm = 0.0f;
				bool visible = false;

				if (light->isArea()) {
					wi = p - shadingData.x;
					float l = wi.lengthSq();
					wi = wi.normalize();
					GTerm = fabs(Dot(wi, shadingData.sNormal)) * max(Dot(-wi, light->normal(wi)), 0.0f) / l;
					visible = (GTerm > 0 && scene->visible(shadingData.x + wi * EPSILON, p));
					pdf_light = pdf * pmf;
				}
				else {
					wi = p;
					GTerm = fabs(Dot(wi, shadingData.sNormal));
					visible = (GTerm > 0 && scene->visible(shadingData.x + wi * EPSILON, shadingData.x + (wi * 10000.0f)));
					pdf_light = pdf * pmf;
				}

				if (visible) {
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
							pdf = hitlight->PDF(wi, shadingData.x, shadowShadingData.x);
							pdf_light = pdf * pmf;
							W = MISWeight(pdf_bsdf, pdf_light);
							float l = (shadowShadingData.x - shadingData.x).lengthSq();
							float GTerm = fabs(Dot(wi, shadingData.sNormal)) * max(Dot(-wi, hitlight->normal(wi)), 0.0f) / l;
							L_bsdf = bsdf * emitted * GTerm * W / pdf_bsdf;
						}
					}

				}
				else {
					// Environment light
					emitted = scene->background->evaluate(wi);
					pdf = scene->background->PDF(wi, shadingData.x, shadowShadingData.x);
					pdf_light = pdf * pmf;
					W = MISWeight(pdf_bsdf, pdf_light);
					float GTerm = fabs(Dot(wi, shadingData.sNormal));
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
							float l2 = (shadowShadingData.x - shadingData.x).lengthSq();
							float GTerm = fabs(Dot(wi, shadingData.sNormal)) * max(Dot(-wi, hitlight->normal(wi)), 0.0f) / l2;
							L_bsdf = bsdf * emitted * GTerm / pdf_bsdf;
						}
					}

				}
				else {
					// Environment light
					emitted = scene->background->evaluate(wi);
					float GTerm = fabs(Dot(wi, shadingData.sNormal));
					L_bsdf = bsdf * emitted * GTerm / pdf_bsdf;

				}
			}
		}
		return  L_light + L_bsdf;
	}
	Colour pathTraceMIS(Ray& r, Colour pathThroughput, int depth, Sampler* sampler)
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
				Lo = Lo + pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
			}

			//------Direct Lighting------
			Lo = Lo + pathThroughput * computeDirectMIS(shadingData, sampler);

			//Max Depth
			if (depth >= MAX_DEPTH)
			{
				return Lo;
			}
			//Russian Roulette
			float russianRouletteProbability = max(0.0f, min(pathThroughput.Lum(), 0.9f));
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

			float cosTheta = fabs(Dot(wi, shadingData.sNormal));
			if (pdf > 1e-12f) {
				pathThroughput = pathThroughput * bsdf * cosTheta / pdf;
				Ray nextRay;
				nextRay.init(shadingData.x + (wi * EPSILON), wi);
				return Lo + pathTraceMIS(nextRay, pathThroughput, depth + 1, sampler);
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
			return computeDirectMIS(shadingData, sampler);
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
	void render(bool denoising)
	{
		film->incrementSPP();
		generateTiles();
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::renderTile, this, i);
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}
		if (denoising) film->denoise();
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b, false);
				canvas->draw(x, y, r, g, b);
			}
		}
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

	//--------PPM--------
	void renderPPM(bool denoising) {
		photonCount = 0;
		generateTiles();
		film->incrementSPP();
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::PPM_GeneratePhoton_MultiThread, this, i);
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}
		//calculate hitpoint results from PPM_GeneratePhoton_MultiThread
		for (auto& cache : cacheHP) {
			for (auto& pair : cache) {
				HitPoint* hp = pair.first;
				Colour col = pair.second;
				hp->flux = hp->flux + col;
				hp->photonCount++;
			}
			cache.clear();
		}
		
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				int index = y * film->width + x;
				Colour result(0.0f, 0.0f, 0.0f);
				for (HitPoint& hp : hitPoints[index]) {
					if (hp.photonCount > 0) {

						float oldradius2 = hp.radius2;
						hp.radius2 = hp.radius2 * (hp.totalPhotonCount + PPM_alpha * hp.photonCount) / (hp.totalPhotonCount + hp.photonCount);
						hp.tau = (hp.flux + hp.tau) * hp.radius2 / oldradius2;
						result = result + hp.tau * hp.pathThroughput / (hp.radius2 * M_PI);

						hp.totalPhotonCount += hp.photonCount;
						hp.flux = Colour(0, 0, 0);
						hp.photonCount = 0;

					}
				}
				result = (result + DirectLight[index]) / PPM_MAX_RAY;
				//film->film[index] = film->film[index] * (film->SPP - 1) / film->SPP + result / film->SPP;
				film->film[index] = result;
			}
		}

		if (denoising) film->denoise();
		for (int y = 0; y < film->height; y++) {
			for (int x = 0; x < film->width; x++) {
				unsigned char r;
				unsigned char g;
				unsigned char b;
				film->tonemap(x, y, r, g, b, true);
				canvas->draw(x, y, r, g, b);
			}
		}

	}
	void PPM_GeneratePhoton_MultiThread(int threadID) {

		//photon pass
		while (photonCount < PPM_photonsPerIteration) {
			float pmf, pdfPos, pdfDir;
			Light* light = scene->sampleLight(&samplers[threadID], pmf);
			Vec3 lightPos = light->samplePositionFromLight(&samplers[threadID], pdfPos);
			Vec3 lightDir = light->sampleDirectionFromLight(&samplers[threadID], pdfDir);
			if (pdfPos < EPSILON || pdfDir < EPSILON) {
				continue;
			}
			photonCount++;
			Ray photon(lightPos + lightDir * EPSILON, lightDir);

			Colour Le = light->getTotalPhotonFlux(-lightDir);
			Colour flux = Le / (pmf * pdfPos * pdfDir * PPM_photonsPerIteration);


			for (int depth = 0; depth < PPM_MAX_DEPTH; depth++)
			{
				IntersectionData intersection = scene->traverse(photon);
				ShadingData shadingData = scene->calculateShadingData(intersection, photon);
				if (shadingData.t >= FLT_MAX) break;

				if (!shadingData.bsdf->isPureSpecular()) {
					//std::vector<HitPoint*> resHP;
					Vec3 wi = -photon.dir;
					float cosTheta = Dot(wi, shadingData.sNormal);
					Colour bsdf = shadingData.bsdf->evaluate(shadingData, wi);
					Colour col = flux * bsdf * cosTheta;

					kdTree->traverse(kdTree->root, shadingData.x, photon.dir, cacheHP[threadID],col);


					//Russian Roulette
					float russianRouletteProbability = max(0.0f, min(flux.Lum(), 0.9f));
					if (samplers[threadID].next() < russianRouletteProbability)
					{
						flux = flux / russianRouletteProbability;
					}
					else
					{
						break;
					}
				}

				//next ray
				float pdf;
				Colour bsdf;
				Vec3 newDir = shadingData.bsdf->sample(shadingData, &samplers[threadID], bsdf, pdf);
				if (pdf < EPSILON) break;

				float cosTheta = fabs(Dot(newDir, shadingData.sNormal));

				flux = flux * bsdf * cosTheta / pdf;
				photon.init(shadingData.x + (newDir * EPSILON), newDir);

			}
		}
	}

	//generate hitpoints
	//add simple multithreading
	void PPM_init() {
		DirectLight = new Colour[film->width * film->height];

		if (hitPoints.empty()) {
			hitPoints.resize(film->width * film->height);
		}
		generateTiles();
		for (int i = 0; i < numProcs; i++) {
			threads[i] = new std::thread(&RayTracer::PPM_GenerateHP_Multithread, this, i);
		}
		for (int i = 0; i < numProcs; i++) {
			threads[i]->join();
			delete threads[i];
		}

		std::vector<HitPoint*> flatHitPoints;
		//initial KDTree
		for (auto& hps : hitPoints) {
			for (HitPoint& hp : hps) {
				HitPoint* ptr = &hp;
				flatHitPoints.push_back(ptr);
			}
		}
		kdTree->buildTree(flatHitPoints);
	}
	void PPM_GenerateHP_Multithread(int threadID) {

		while (!stopThreads) {
			TileInfo* tile;
			{
				std::lock_guard<std::mutex> lock(tileQueueMutex);
				if (tileQueue.empty()) return;
				tile = std::move(tileQueue.front());
				tileQueue.pop();
			}
			int xStart = tile->x;
			int yStart = tile->y;

			int xEnd = min(xStart + TILE_SIZE, film->width);
			int yEnd = min(yStart + TILE_SIZE, film->height);

			for (int y = yStart; y < yEnd; y++) {
				for (int x = xStart; x < xEnd; x++) {
					int index = y * film->width + x;
					for (int i = 0; i < PPM_MAX_RAY; i++) {
						float px = x + samplers[threadID].next();
						float py = y + samplers[threadID].next();
						Ray ray = scene->camera.generateRay(px, py);
						PPM_GenerateHP_Bounce(ray, 0, &samplers[threadID], index, Colour(1.0f, 1.0f, 1.0f));

					}
				}
			}
		}
	}
	//One ray can generate multiple hitpoints by bouncing
	//Hitpoints can only be placed on the DIFFUSE surface
	void PPM_GenerateHP_Bounce(Ray& ray, int depth, Sampler* sampler, int index, Colour throughput) {

		IntersectionData intersection = scene->traverse(ray);
		ShadingData shadingData = scene->calculateShadingData(intersection, ray);

		if (shadingData.t < FLT_MAX) {
			//1.if ray intersects an emissive surface
			if (shadingData.bsdf->isEmissive()) {
				//1.1 if the ray is a light
				if (shadingData.bsdf->isLight()) {
					DirectLight[index] = DirectLight[index] + shadingData.bsdf->emit(shadingData, shadingData.wo);
					// discard the ray that hits the light
					return;
				}
				//1.2 if the ray is not a light, store (emission * pathThroughput), then regard this point as non-emissive material
				DirectLight[index] = DirectLight[index] + shadingData.bsdf->emit(shadingData, shadingData.wo) * throughput;
			}
			//2. if the ray hits a non-pure specular surface, store hit point and return
			if (!shadingData.bsdf->isPureSpecular()) {
				HitPoint hp;
				hp.position = shadingData.x;
				hp.normal = shadingData.sNormal;
				hp.depth = depth;
				hp.pathThroughput = throughput;
				hitPoints[index].push_back(hp);
				return;
			}
			//3.For pure specular, we don't store the hitpoint, only check next bounce
			if (depth < PPM_MAX_BOUNCE) {
				float pdf;
				Colour bsdf;
				Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
				if (pdf < EPSILON) return;
				// it is pure specular, so cosTheta = 1
				Colour NextThroughput = throughput * bsdf / pdf;

				Ray nextRay;
				nextRay.init(shadingData.x + (wi * EPSILON), wi);
				PPM_GenerateHP_Bounce(nextRay, depth + 1, sampler, index, NextThroughput);
			}
		}
		// 4. if the ray misses the scene, add background colour
		//but background color is (0,0,0), so this step is not needed

	}
};