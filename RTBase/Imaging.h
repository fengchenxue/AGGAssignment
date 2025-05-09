#pragma once
#include"oidn.hpp"
#pragma comment(lib, "OpenImageDenoise_core.lib")
#pragma comment(lib, "OpenImageDenoise.lib")

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

//used for adaptive sampling, this is the minimum number of samples before we start checking for convergence
#define ADAPTIVE_SAMPLING_MINIMUM_SAMPLES 8.0f
#define ADAPTIVE_SAMPLING_EPS 0.05f

struct PixelStat {
	Vec3 mean = Vec3(0.0f, 0.0f, 0.0f);
	Vec3 meanSquared = Vec3(0.0f, 0.0f, 0.0f);
	//int numSamples = 0;
	float weight = 0.0f;
	bool converged = false;

	//returns true if converged. 
	//The return value helps check if a tile is converged.
	bool addSample(const Vec3& sample) {
		//numSamples++;
		Vec3 delta = sample - mean;
		mean = mean + delta / weight;
		meanSquared = meanSquared + delta * (sample - mean);

		if (weight > ADAPTIVE_SAMPLING_MINIMUM_SAMPLES) {
			Vec3 variance = meanSquared / (weight - 1.0f);
			float invnumSamples = 1.0f / weight;
			//95.4% confidence interval
			Vec3 error = Vec3(sqrtf(variance.x * invnumSamples), sqrtf(variance.y * invnumSamples), sqrtf(variance.z * invnumSamples)) * 2.0f;
			//I want the 95% CI to be smaller than 10% of my current average pixel color.
			Vec3 tolerance = Vec3(abs(mean.x), abs(mean.y), abs(mean.z)) * ADAPTIVE_SAMPLING_EPS;
			converged = (error.x <= tolerance.x && error.y <= tolerance.y && error.z <= tolerance.z);
		}
		return converged;
	}
};

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete[] alpha;
			alpha = NULL;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) < 0.5f && fabsf(y) < 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class GaussianFilter : public ImageFilter
{
public:
	GaussianFilter(float _alpha=2.0f, float _radius=2.0f)
		: alpha(_alpha), radius(_radius){}

	float filter(const float x, const float y) const override
	{
		return gaussian1D(x) * gaussian1D(y);
	}
	int size() const override
	{
		return static_cast<int>(std::ceil(radius));
	}
private:
	float alpha;
	float radius;
	float gaussian1D(float d) const
	{
		d = std::fabs(d);
		if (d >= radius)
			return 0.0f;

		return std::exp(-alpha * d * d) - std::exp(-alpha * radius * radius);
	}
};

class MitchellFilter : public ImageFilter
{
public:
	MitchellFilter(float _B=1.0f/3.0f, float _C=1.0f/3.0f)
		: B(_B), C(_C){}

	float filter(const float x, const float y) const override
	{
		return mitchell1D(x) * mitchell1D(y);
	}
	int size() const override
	{
		return 2;
	}
private:
	float B;
	float C;
	float mitchell1D(float x) const
	{
		x = std::fabs(x);
		if (x > 2.0f)
			return 0.0f;

		float x2 = x * x;
		float x3 = x2 * x;

		if (x < 1.0f) return ((12.0f - 9.0f * B - 6.0f * C) * x3 + (-18.0f + 12.0f * B + 6.0f * C) * x2 + (6.0f - 2.0f * B)) / 6.0f;
		else return ((-B - 6.0f * C) * x3 + (6.0f * B + 30.0f * C) * x2 + (-12.0f * B - 48.0f * C) * x + (8.0f * B + 24.0f * C)) / 6.0f;
	}
};
class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;

	//used for adaptive sampling
	std::vector<PixelStat> pixelStats;

	//denoise
	//std::vector<float> inputBuffer;
	//std::vector<float> outputBuffer;

	oidn::DeviceRef device;
	oidn::FilterRef denoiseFilter;
	oidn::BufferRef	inputBuffer;
	oidn::BufferRef	outputBuffer;
	void splat(const float x, const float y, const Colour& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		
		int r = filter->size();
		int x_start = std::max(0,static_cast<int>(floorf(x - r)));
		int x_end = std::min(static_cast<int>(width-1),static_cast<int>(floorf(x + r)));
		int y_start = std::max(0, static_cast<int>(floorf(y - r)));
		int y_end = std::min(static_cast<int>(height-1), static_cast<int>(floorf(y + r)));
		for (int j = y_start; j <= y_end; j++)
		{
			for (int i = x_start; i <= x_end; i++)
			{
				int index = j * width + i;
				if (pixelStats[index].converged) continue;

				float dx = i + 0.5f - x;
				float dy = j + 0.5f - y;
				float weight = filter->filter(dx, dy);
				film[index] = film[index] + (L * weight);
				pixelStats[index].weight += weight;
			}
		}
	}
	//I added this function
	void denoise(bool PPM) {

		float* input = (float*)inputBuffer.getData();
		float* output = (float*)outputBuffer.getData();
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				float sp = PPM ? 1.0f : pixelStats[y * width + x].weight;
				int index = (y * width + x) * 3;

				if (sp > 0) {
					input[index] = film[y * width + x].r / sp;
					input[index + 1] = film[y * width + x].g / sp;
					input[index + 2] = film[y * width + x].b / sp;
					/*inputBuffer[index] = film[y * width + x].r / sp;
					inputBuffer[index + 1] = film[y * width + x].g / sp;
					inputBuffer[index + 2] = film[y * width + x].b / sp;*/
				}
				else {
					input[index] = 0.0f;
					input[index + 1] = 0.0f;
					input[index + 2] = 0.0f;
					/*inputBuffer[index] = 0.0f;
					inputBuffer[index + 1] = 0.0f;
					inputBuffer[index + 2] = 0.0f;*/
				}
			}
		}
		denoiseFilter.execute();
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, bool PPM, bool denoising, float exposure = 1.0f)
	{
		// Return a tonemapped pixel at coordinates x, y
		
		Colour color;
		if (!denoising) {
			color = (PPM) ? (film[y * width + x] * exposure) : (film[y * width + x] * exposure / pixelStats[y * width + x].weight);
		}
		else {
			float* output = (float*)outputBuffer.getData();
			int index = (y * width + x) * 3;
			color.r = output[index];
			color.g = output[index + 1];
			color.b = output[index + 2];
			
			/*int index = (y * width + x) * 3;
			color.r = outputBuffer[index];
			color.g = outputBuffer[index + 1];
			color.b = outputBuffer[index + 2];*/
		}

		// ACES Filmic Approximation (Narkowicz 2015)
		const float A = 2.51f;
		const float B = 0.03f;
		const float C = 2.43f;
		const float D = 0.59f;
		const float E = 0.14f;

		color.r = (color.r * (A * color.r + B)) / (color.r * (C * color.r + D) + E);
		color.g = (color.g * (A * color.g + B)) / (color.g * (C * color.g + D) + E);
		color.b = (color.b * (A * color.b + B)) / (color.b * (C * color.b + D) + E);

		//Gamma correction
		float invGamma = 1.0f / 2.2f;
		color.r = powf(color.r, invGamma);
		color.g = powf(color.g, invGamma);
		color.b = powf(color.b, invGamma);

		// Clamp to 0-1
		color.r = std::min(1.0f, std::max(0.0f, color.r));
		color.g = std::min(1.0f, std::max(0.0f, color.g));
		color.b = std::min(1.0f, std::max(0.0f, color.b));

		// Convert to 0-255
		r = (unsigned char)(color.r * 255);
		g = (unsigned char)(color.g * 255);
		b = (unsigned char)(color.b * 255);
		
	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;

		//I need to change code here
		// adaptive sampling
		pixelStats.resize(width * height);
		
		//denoise
		device = oidn::newDevice(OIDN_DEVICE_TYPE_CPU);
		device.commit();

		inputBuffer = device.newBuffer(width * height * 3 * sizeof(float));
		outputBuffer = device.newBuffer(width * height * 3 * sizeof(float));

		//inputBuffer.resize(width * height * 3);
		//outputBuffer.resize(width * height * 3);

		denoiseFilter = device.newFilter("RT");
		denoiseFilter.set("hdr", true);
		denoiseFilter.setImage("color", inputBuffer, oidn::Format::Float3, width, height);
		denoiseFilter.setImage("output", outputBuffer, oidn::Format::Float3, width, height);
		denoiseFilter.commit();
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
		for (auto &p: pixelStats)
		{
			p.mean = Vec3(0.0f, 0.0f, 0.0f);
			p.meanSquared = Vec3(0.0f, 0.0f, 0.0f);
			p.weight = 0.0f;
			p.converged = false;
		}

	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / pixelStats[i].weight;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};