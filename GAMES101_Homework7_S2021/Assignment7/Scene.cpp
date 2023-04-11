//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}
#define my
// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
#ifdef my
{
    // TO DO Implement Path Tracing Algorithm here
    auto inter_p = intersect(ray);
    if(!inter_p.happened){
        return {100}; //return for background color
    }
    if(inter_p.m->hasEmission()){
        return inter_p.m->getEmission(); //对于光源，我们有光源发出的光Lo(p,wo) = Le(p,wo)
    }

    Intersection inter_light;
    float pdf_light;
    sampleLight(inter_light, pdf_light); //按道理，我们应该对所有的光源计算总的pdf，不过这里只有一个光源，应该没有问题。
    Vector3f L_dir;


    auto &x = inter_p.coords;
    auto wo = (ray.origin - x).normalized();
    auto &xx = inter_light.coords;
    auto &n = inter_p.normal;
    auto wi = (xx - x).normalized();

//    Vector3f p_deviation = (dotProduct(ray.direction, inter_p.normal) < 0) ?
//                           x + inter_p.normal * EPSILON :
//                           x - inter_p.normal * EPSILON ;
    Ray wi_ray(x,wi);
    Intersection x2light = intersect(wi_ray);

    float d = (xx - x).norm();

//    if(x2light.distance - d <= -0.01 && (x2light.happened && (x2light.coords - xx).norm() < 5e-3)){
//        assert(false);
//    }//没有交打到无穷远处使光线能量出现了负值出现了问题,还会出现交到更远处的情况-》灯光平面和所在平面平行的情况？
    if(x2light.happened && abs(x2light.distance - d) <= 0.001){ //这里注意使用inter_light.normal 而非 x2light.normal 两者在交接点上很有可能不是同一个物体
    //if(x2light.happened && (x2light.coords - xx).norm() < 5e-3){
        L_dir = inter_light.emit * inter_p.m->eval(wo,wi,n) * dotProduct(n,wi) * dotProduct(inter_light.normal,-wi) //没有东西在光线的上面，因此后一项一定大于0
                / (d * d * pdf_light);
//        if(L_dir.x <=-1){
//            bool fuck = true;
//        }
        //assert(L_dir.x >= 0);
    }
    Vector3f L_indir;
    //对material的sample是随机sample，即按照漫反射的方式sample，而对于object的sample则是对面积的均匀采样。
    float rr = get_random_float();
    if(rr <= RussianRoulette){
        wi = inter_p.m->sample(wo,n);
        //assert(wi.norm()<=1.01&&wi.norm()>=0.99);
        Ray indir_ray(x,wi);
        Intersection inter_indir = intersect(indir_ray);

        if(inter_indir.happened && !inter_indir.m->hasEmission()) //发光部分已经计算过了，这里不再计算
        {
            float pdf = inter_p.m->pdf(wo,wi,n);
            if(pdf > EPSILON) {
                L_indir = castRay(indir_ray, depth + 1) * inter_p.m->eval(wo, wi, n) * dotProduct(wi, n)
                          / (pdf * RussianRoulette); //我说能量怎么越算越大
            }
        }


    }
    return L_dir + L_indir;
}
// Implementation of Path Tracing
#endif
#ifndef my
{
    //计算ray的交点
    Intersection  inter = Scene::intersect(ray);
    Vector3f L_dir(0, 0, 0);
    Vector3f L_indir(0, 0, 0);
    //如果没有交点返回空
    if (!inter.happened) {
        return Vector3f(0,0,0);
    }
    //如果交点打到光源
    if (inter.m->hasEmission()) {
        if(depth==0){
            return inter.m->getEmission();
        }
        else return Vector3f(0,0,0);
    }
    //对光源进行采样
    Intersection lightInter;
    float lightPdf = 0.0f;
    sampleLight(lightInter, lightPdf);
    //物体表面的信息 n x
    Vector3f n = inter.normal;
    Vector3f x = inter.coords;
    //灯光表面信息 n' x'
    Vector3f nn = lightInter.normal;
    Vector3f xx = lightInter.coords;
    //求物体表面与灯光的连线 x->x'
    Vector3f w = xx - x;
    Vector3f lightDir = w.normalized();
    float lightDistance = w.x * w.x + w.y * w.y + w.z * w.z;
    //看x->x'过程中是否有遮挡，lightObjtoL为物体表面的交点向光源的射线
    Ray lightObjtoL(x, lightDir);
    Intersection objtoLInter = intersect(lightObjtoL);
    //击中的是光源的情况，利用交点与光源之差小于某个小值判断
    if (objtoLInter.happened && (objtoLInter.coords - xx).norm() < 5e-3) {
        L_dir = lightInter.emit * inter.m->eval(ray.direction, lightDir, n) * dotProduct(lightDir, n) * dotProduct(-lightDir, nn) / lightDistance / lightPdf;
    }
    //轮盘赌
    if (get_random_float() < RussianRoulette) {
        //随机在物体表面取反射方向
        Vector3f nextDir = inter.m->sample(ray.direction, n).normalized();
        Ray nextRay(x,nextDir);
        Intersection nextInter = intersect(nextRay);
        //下一条光线击中的不是光源时才加入
        if (nextInter.happened && !nextInter.m->hasEmission()) {
            float pdf = inter.m->pdf(ray.direction, nextDir, n);
            Vector3f f_r = inter.m->eval(ray.direction, nextDir, n);
            L_indir = castRay(nextRay, depth + 1) * f_r * dotProduct(nextDir, n) / pdf / RussianRoulette;
        }
    }
    return L_dir + L_indir;
    // TO DO Implement Path Tracing Algorithm here
}
#endif