//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_MATERIAL_H
#define RAYTRACING_MATERIAL_H

#include "Vector.hpp"
#include "global.hpp"
#include <cmath>

enum MaterialType {
    DIFFUSE,
    MICROFAFET
};

class Material{
private:
    //for Microfacet material
    float ggxAlpha() const
    {
        const float r = clamp(0.0f, 1.0f,  roughness);
        //actually a = r * r is just a usual parameter mapping . You can change the formula as you like.
        return std::max(0.02f , r*r);
        //avoid tiny roughness causing numerical problems
        //so that microfacet != specular material
    }

    float cal_ggxD(float NdotH) const
    //how many tiny specular planes face to the wanted direction(H)
    {
        if(NdotH <= 0.0f)
            return 0.0f;
        //only take the planes with the same orientation as main plane into consideration
        const float a = ggxAlpha(), a2 = a * a;

        const float d = NdotH * NdotH * (a2 - 1.0f) + 1.0f;
        return a2 / (M_PI * d * d);
    }

    float smithG1(float NdotH) const
    //can this tiny specular planes be seen in given direction(H)? --  it can be covered by the other convex planes nearby.
    //G1 gives a propertional of the planes can be seen.
    {
        if(NdotH <= 0.0f)
            return 0.0f;
        const float a = ggxAlpha(), a2 = a * a;
        return 2.0f * NdotH /
                    (NdotH + std::sqrt(a2 + (1.0f -a2) * NdotH * NdotH));
    }



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
    // \param I is the incident view direction
    //
    // \param N is the normal at the intersection point
    //
    // \param ior is the material refractive index
    //
    // \param[out] kr is the amount of light reflected
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

    //a is in the coordinate of(N == z-axis)
    //this function is to change a into the coordinate of real world(N is might not z-axis)
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

    float roughness = 0.5f;//0->specular , 1->very rough,

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
        case DIFFUSE:
        {
            // uniform sample on the hemisphere
            float x_1 = get_random_float(), x_2 = get_random_float();
            float z = std::fabs(1.0f - 2.0f * x_1);//same as 1-x_1,but it maps on [-1,1],then 2*x_1
            float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
            Vector3f localRay(r*std::cos(phi), r*std::sin(phi), z);
            return toWorld(localRay, N);
            
            break;
        }
        case MICROFAFET:
        {
            const Vector3f V = -wi;//wi orint towards the plane, but in formula wi should face to the light source
            if(dotProduct(V, N) <= 0.0f)
                return Vector3f();
            float u1 = get_random_float() , u2  = get_random_float();
            float a = ggxAlpha(), a2 = a*a;
            float phi = 2*M_PI * u1;//included angle(H) in horizon plane
            //how far it deviate from N
            //and cosTheta is actually N dot H (the z length)
            //theta is the vertical angle of H
            float cosTheta = std::sqrt((1.0f - u2) / (1.0f + (a2-1.0f) * u2));//ggx sample formula
            float sinTheta = std::sqrt(std::max(0.0f , 1.0f - cosTheta*cosTheta));//the sqrt(x*x+y*y) length

            //got H
            Vector3f H = toWorld(
                Vector3f(sinTheta * std::cos(phi), sinTheta * std::sin(phi),cosTheta)
                , N);

            //calculate wo
            float VdotH = dotProduct(V, H);
            if(VdotH < 0.0f)
                return Vector3f();
            Vector3f W_O = 2.0f * VdotH * H - V;//L
            return normalize(W_O);
        }
    }
}

float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // uniform sample probability 1 / (2 * PI)
            if (dotProduct(wo, N) > 0.0f)
                return 0.5f / M_PI;
            else
                return 0.0f;
            break;
        }
        case MICROFAFET:
        {
            float NdotV = dotProduct(-wi, N);
            float NdotL = dotProduct(wo, N);
            if(NdotV <= 0.0f || NdotL <= 0.0f)
                return 0.0f;
            Vector3f H = normalize(wo + (-wi));//L + V

            float NdotH = dotProduct(N, H);
            float VdotH = dotProduct(-wi, H);
            if(NdotH <= 0.0f || VdotH <= 0.0f)
                return 0.0f;

            //probablity density of (H) = ggxD(NoH)*NoH , change to  pdf of (w_o)= p(H)/4*HoV;
            return (cal_ggxD(NdotH) * NdotH) / (4.0f * VdotH);
        }
    }
}

//BRDF
Vector3f Material::eval(const Vector3f &wi, const Vector3f &wo, const Vector3f &N){
    switch(m_type){
        case DIFFUSE:
        {
            // calculate the contribution of diffuse model
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / M_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFAFET:
        {
            float NdotV = dotProduct(-wi, N);
            float NdotL = dotProduct(wo, N);
            if(NdotV <= 0.0f || NdotL <= 0.0f)
                return 0.0f;
            Vector3f H = normalize(wo + (-wi));//L + V

            float NdotH = dotProduct(N, H);
            float VdotH = dotProduct(-wi, H);
            if (NdotH <= 0.0f || VdotH <= 0.0f) return Vector3f();

            //cal BRDF:returned propertional
            //for metal
            float x = 1.0f - clamp(0.0f, 1.0f, VdotH);//x = 1-cosTheta
            float x2 = x*x , x5 = x2*x2*x;

            Vector3f F = Ks + (Vector3f(1.0f)-Ks)*x5;
            //for glass,plastic,air...（non-metal）
            //Vector3f F = fresnel(wi, H,its ior, Ks)
            float G = smithG1(NdotV) * smithG1(NdotL);
            float D = cal_ggxD(NdotH);
            return F*G*D / (4.0f * NdotV * NdotL);
        }
    }
}

#endif //RAYTRACING_MATERIAL_H
