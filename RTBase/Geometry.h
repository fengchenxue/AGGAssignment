#pragma once

#include "Core.h"
#include "Sampling.h"
#define EPSILON 0.0001f

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	//important comment: 
	//d= -n*p0, where p0 is a point on the plane
	//t= -(n*o+d) / (dir*n) or t= (p0-o)*n / (dir*n)
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		float nd = Dot(n, r.dir);
		if (std::abs(nd) < EPSILON) return false;
		
		t = -(Dot(n, r.o) + d) / nd;
		return t >= 0;
	}
};

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float invarea; // Inverse area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		//The original code seemed wrong, so I rewrote e1 and e2
		e1 = vertices[1].p - vertices[0].p;
		e2 = vertices[2].p - vertices[0].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		invarea = 1.0f / area;
		//The orignal code was d = Dot(n, vertices[0].p), the standard formula is d = -n*p0
		d = -Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
	// Add code here
	// Möller-Trumbore
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		Vec3 s = r.o - vertices[0].p;
		Vec3 s1 = r.dir.cross(e2);
		Vec3 s2 = s.cross(e1);
		float invDet = s1.dot(e1);
		if (std::abs(invDet) < EPSILON) return false;
		invDet = 1.0f / invDet;
		
		t = s2.dot(e2) * invDet;
		u = s1.dot(s) * invDet;
		v = s2.dot(r.dir) * invDet;
		if (t < 0.0f|| u < 0.0f || v < 0.0f || u + v > 1.0f) return false;

		return true;

	}
	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{
		Vec3 t_min(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		Vec3 t_max(FLT_MAX, FLT_MAX, FLT_MAX);

		for (int i = 0; i <= 2; i++) {
			if (std::abs(r.dir.coords[i]) < EPSILON) {
				if (r.o.coords[i] < min.coords[i] || r.o.coords[i] > max.coords[i]) return false;
			}
			else {
				float invD = 1.0f / r.dir.coords[i];
				t_min.coords[i] = (min.coords[i] - r.o.coords[i]) * invD;
				t_max.coords[i] = (max.coords[i] - r.o.coords[i]) * invD;
				if (r.dir.coords[i] < 0.0f) std::swap(t_min.coords[i], t_max.coords[i]);
			}
		}
		float t_enter = std::max(t_min.x, std::max(t_min.y, t_min.z));
		float t_exit = std::min(t_max.x, std::min(t_max.y, t_max.z));

		if (t_enter > t_exit || t_exit < 0.0f) return false;

		t = (t_enter < 0.0f) ? 0.0f : t_enter;

		return true;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float t;
		return rayAABB(r, t);
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;
		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		Vec3 l = r.o - centre;
		float b = 2*Dot(l, r.dir);
		float c = Dot(l, l) - radius * radius;

		float delta = b * b - 4 * c;
		if (delta < 0) return false;
		
		float sqrtDelta = sqrt(delta);
		float t0 = (-b - sqrtDelta) / 2;
		float t1 = (-b + sqrtDelta) / 2;

		if (t0 >= 0.0f) {
			t = t0;
			return true;
		}else if (t1>=0.0f) {
			t = t1;
			return true;
		}

		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
