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

/* 三轴输入均为调用方持有的只读数组，仅在调用期间借用；VQF 不保存输入指针。 */
void updateGyr(const vqf_real_t *gyr);
void updateAcc(const vqf_real_t *acc);
void updateMag(const vqf_real_t *mag);
/* 四元数输出由调用方提供，函数仅在调用期间写入四个元素，不保存指针。 */
void getQuat3D(vqf_real_t *out);
void getQuat6D(vqf_real_t *out);
void getQuat9D(vqf_real_t *out);
vqf_real_t getDelta();
/* out 可为 NULL；非 NULL 时写入三个偏置分量，函数不保存指针。 */
vqf_real_t getBiasEstimate(vqf_real_t *out);
/* bias 为调用方持有的三个只读偏置分量，仅在调用期间借用并复制。 */
void setBiasEstimate(const vqf_real_t *bias, vqf_real_t sigma);
void initVqf(vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

#ifdef __cplusplus
}
#endif

#endif //VQF_VQF_H
