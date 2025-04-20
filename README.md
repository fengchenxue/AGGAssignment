# Assignment instructions

## 1.Overview
This project implements 2 ways of ray tracing. One is MIS, the other is progressive photon mapping (PPM).

Here are two setting options here. 
```
bool PPMMode = false; 
bool denoising = false;
```
For "PPMMode=false", you can see MIS renderer. For "PPMMode=true", you can see PPM renderer.

"denoising" option is used to enable OIDN-based denoiser.


## 2.Material
I implemented the following materials:
```
1. Diffuse 
#Line 154-195, class DiffuseBSDF, Materials.h
2. Mirror
#Line 197-251, class MirrorBSDF, Materials.h
3. Conductor
#Line 254-352, class ConductorBSDF, Materials.h
4. Glass
#Line 354-524, class GlassBSDF, Materials.h
5. Oren-Nayar
#Line 626-704, class OrenNayarBSDF, Materials.h
6. Plastic
#Line 706-823, class PlasticBSDF, Materials.h

Helper Function
#Line 15-23, Line 48-119, Materials.h

I didn't implement Dielectric and Layered materials.
```
![Diffuse Material without denoiser](/pic/Diffuse_NoDenoiser.png "Magic Gardens")










## Scenes

Scenes can be found on the Moodle page. Please download them and place them in the working directory.

## Tasks for Students

Several functions and algorithms are left incomplete and require implementation. Look for comments such as:

```cpp
// Add code here
```

## Notes

- Ensure your implementations are efficient and well-commented.
- Test incremental changes using appropriate scenes.
  
