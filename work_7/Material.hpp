//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"

enum MaterialType { DIFFUSE, Microfacet };

class Material{
private:

    // Compute reflection direction
    Vector3f reflect(const Vector3f &I, const Vector3f &N) const
    {
        return I - 2 * dotProduct(I, N) * N;
    }

    // Compute refraction direction using Snell's law
    //
    // We need to handle with care the two possible situations:
    //
    //    - When the ray is inside the object
    //
    //    - When the ray is outside.
    //
    // If the ray is outside, you need to make cosi positive cosi = -N.I
    //
    // If the ray is inside, you need to invert the refractive indices and negate the normal N
    Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        Vector3f n = N;
        if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }

    // Compute Fresnel equation
    //
    // \param I is the incident view direction,入射光线的方向向量
    //
    // \param N is the normal at the intersection point,法线
    //
    // \param ior is the material refractive index,光线从空气进入时的相对折射率
    //
    // \param[out] kr is the amount of light reflected，反射系数（Reflectance）,表示反射光线的能量占比
    void fresnel(const Vector3f &I, const Vector3f &N, const float &ior, float &kr) const
    {
        float cosi = clamp(-1, 1, dotProduct(I, N));
        float etai = 1, etat = ior;
        if (cosi > 0) {  std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        // As a consequence of the conservation of energy, transmittance is given by:
        // kt = 1 - kr;
    }

    /// <summary>
    /// GGX法线分布
    /// </summary>
    /// <param name="N">微表面宏观法线</param>
    /// <param name="H">半程向量</param>
    /// <param name="roughness">微表面粗糙度</param>
    /// <returns>多少能量反射到出射方向</returns>
    float DistributionGGX(const Vector3f& N, const Vector3f& H, const float& roughness)
    {
        float a = roughness * roughness;
        float a2 = a * a;
        float cosnh = std::max(0.0f, dotProduct(N, H));
        float cosnh2 = cosnh * cosnh;
        float x2 = (cosnh2 * (a2 - 1) + 1) * (cosnh2 * (a2 - 1) + 1);

        return a2 / (M_PI * x2);
    }
  
   
    /// <summary>
    /// 计算Smith几何遮蔽子项G_sub
    /// </summary>
    /// <param name="NdotV">微表面宏观法线与观测方向或光源方向的点乘结果</param>
    /// <param name="roughness">微表面粗糙度</param>
    /// <returns></returns>
    float GeometrySub(const float& NdotV, const float& roughness)
    {
        float r = (roughness + 1.0);
        float k = (r * r) / 8.0;

        float nom = NdotV;
        float denom = NdotV * (1.0 - k) + k;
        return nom / denom;
    }
    /// <summary>
    /// Smith几何遮蔽项
    /// </summary>
    /// <param name="N">微表面宏观法线</param>
    /// <param name="V">观测方向</param>
    /// <param name="L">光源方向</param>
    /// <param name="roughness">微表面粗糙度</param>
    /// <returns></returns>
    float GeometrySmith(const Vector3f& N, const Vector3f& V, const Vector3f& L, const float& roughness)
    {
        float NdotV = std::max(dotProduct(N, V), 0.0f);
        float NdotL = std::max(dotProduct(N, L), 0.0f);
        return GeometrySub(NdotV, roughness) * GeometrySub(NdotL, roughness);
    }

     Vector3f toWorld(const Vector3f &a, const Vector3f &N){
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)){
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
     }

public:
    MaterialType m_type;
    //Vector3f m_color;
    Vector3f m_emission;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0,0,0));
    inline MaterialType getType();
    //inline Vector3f getColor();
    inline Vector3f getColorAt(double u, double v);
    inline Vector3f getEmission();
    inline bool hasEmission();

    // sample a ray by Material properties
    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // given a ray, calculate the PdF of this ray
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray
    inline Vector3f eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);

};

Material::Material(MaterialType t, Vector3f e){
    m_type = t;
    //m_color = c;
    m_emission = e;
}

MaterialType Material::getType(){return m_type;}
///Vector3f Material::getColor(){return m_color;}
Vector3f Material::getEmission() {return m_emission;}
bool Material::hasEmission() {
    if (m_emission.norm() > EPSILON) return true;
    else return false;
}

Vector3f Material::getColorAt(double u, double v) {
    return Vector3f();
}


Vector3f Material::sample(const Vector3f &wi, const Vector3f &N){
    switch(m_type){
     case Microfacet:
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
     case Microfacet:
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
    }
}

Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse   model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case Microfacet:
        {
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                float roughness = 0.35; //调整微表面粗糙度

                Vector3f V = -wi;
                Vector3f L = wo;
                Vector3f H = normalize(V + L);

                // calculate distribution of normals: D
                float D = DistributionGGX(N, H, roughness);

                // calculate shadowing masking term: G
                float G = GeometrySmith(N, V, L, roughness);

                // calculate fresnel coefficient: F
                float F;
                float etat = 1.85; //调整当前物体折射率
                fresnel(wo, N, etat, F);

                Vector3f nominator = D * G * F;
                float denominator = 4 * std::max(dotProduct(N, V), 0.0f) * std::max(dotProduct(N, L), 0.0f);
                Vector3f specular = nominator / std::max(denominator, 0.001f);

                //保证能量守恒 ks + kd = 1 即：F + Kd = 1
                Vector3f diffuse = Kd / M_PI; //diffuse = ρ/pi
                return (Vector3f(1.0f) - F) * diffuse + specular;
            }
            else
                return Vector3f(0.0f);
            break;
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
