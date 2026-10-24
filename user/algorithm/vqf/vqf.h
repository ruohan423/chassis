//
// Created by mot on 2026/10/5.
//

#ifndef VQF_VQF_H
#define VQF_VQF_H

#define VQF_SINGLE_PRECISION

#ifdef __cplusplus
extern "C" {
#endif

#ifdef VQF_SINGLE_PRECISION
typedef float vqf_real_t;
typedef float vqf_double_t;
#elif defined(VQF_MIXED_PRECISION)
typedef float vqf_real_t;
typedef double vqf_double_t;
#else   // double precision
typedef double vqf_real_t;
typedef double vqf_double_t;
#endif

void updateGyr(const vqf_real_t gyr[3]);
void updateAcc(const vqf_real_t acc[3]);
void updateMag(const vqf_real_t mag[3]);
void getQuat3D(vqf_real_t out[4]);
void getQuat6D(vqf_real_t out[4]);
void getQuat9D(vqf_real_t out[4]);
vqf_real_t getDelta();
vqf_real_t getBiasEstimate(vqf_real_t out[3]);
void setBiasEstimate(vqf_real_t bias[3], vqf_real_t sigma);
void initVqf(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

#ifdef __cplusplus
}
#endif

#endif //VQF_VQF_H
