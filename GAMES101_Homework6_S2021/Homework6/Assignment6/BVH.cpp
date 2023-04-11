#include <algorithm>
#include <cassert>
#include "BVH.hpp"
const int bucket_size = 32;
BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    double secs = diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %lf secs\n\n",
        hrs, mins, secs);
}
template<class T>
void BVHAccel::getBucketCost(std::vector<float> &bucket_cost, std::vector<int> &bucket_last_obj, T begin, T end,
                             const int &axis, const float &minn, const float &maxx) {

    auto getBucket = [minn,maxx](float a) {
        return  std::clamp((int)((a - minn) / (maxx - minn) * (bucket_size)),0,bucket_size-1);
    };
    Bounds3 bound;
    int cur_bucket = 0;
    for(auto i = begin; i < end; i++){
        int bucket_i;
        switch(axis){
            case 0:
                bucket_i = getBucket((*i)->getBounds().Centroid().x);
                break;
            case 1:
                bucket_i = getBucket((*i)->getBounds().Centroid().y);
                break;
            case 2:
                bucket_i = getBucket((*i)->getBounds().Centroid().z);
                break;
        }

        if(bucket_i != cur_bucket){
            for(int x = cur_bucket; x < bucket_i; x++){
                bucket_last_obj[x] = i - begin;
                bucket_cost[x] = bound.SurfaceArea();
            }
            cur_bucket = bucket_i;
        }
        bound = Union(bound,(*i)->getBounds());
    }
}
BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuildSAH(std::vector{objects[0]});
        node->right = recursiveBuildSAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                    Union(centroidBounds, objects[i]->getBounds().Centroid());
        float best_cost = kInfinity;
        std::vector<Object*> leftshapes,rightshapes;
        for(int i=0; i<3; i++){
            std::vector<float> bucket_cost_l(bucket_size);
            std::vector<float> bucket_cost_r(bucket_size);
            std::vector<int> bucket_last_obj_l(bucket_size);
            std::vector<int> bucket_last_obj_r(bucket_size);
            float minn,maxx;
            //minn = centroidBounds.Centroid()[1];
            switch (i) {
                case 0:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().x <
                               f2->getBounds().Centroid().x;
                    });
                    minn = centroidBounds.pMin.x;
                    maxx = centroidBounds.pMax.x;
                    break;
                case 1:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().y <
                               f2->getBounds().Centroid().y;
                    });
                    minn = centroidBounds.pMin.y;
                    maxx = centroidBounds.pMax.y;
                    break;
                case 2:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBounds().Centroid().z <
                               f2->getBounds().Centroid().z;
                    });
                    minn = centroidBounds.pMin.z;
                    maxx = centroidBounds.pMax.z;
                    break;
            }

//            std::sort(objects.begin(), objects.end(), [i](auto f1, auto f2) {
//                return f1->getBounds().Centroid()[i] <
//                       f2->getBounds().Centroid()[i];
//            });
            getBucketCost(bucket_cost_r, bucket_last_obj_r, objects.rbegin(), objects.rend(), i, maxx, minn);
            getBucketCost(bucket_cost_l, bucket_last_obj_l, objects.begin(), objects.end(), i, minn, maxx);
            float min_cost = kInfinity;
            int best_div = 0;
            for(int l=0;l<bucket_size - 1;l++){
                int r = bucket_size - (l + 2);
                float cost = bucket_cost_l[l] * bucket_last_obj_l[l] + bucket_cost_r[r] * (objects.size() - bucket_last_obj_l[l]);
                // a float accuracy error will occur here, seems have little affect on whole algorithm;
                // assert(bucket_last_obj_l[l] + bucket_last_obj_r[r] == objects.size());
                if(cost < min_cost){
                    min_cost = cost;
                    best_div = bucket_last_obj_l[l];
                }
            }
            if(min_cost < best_cost){
                leftshapes = std::vector<Object*>(objects.begin(),objects.begin() + best_div);
                rightshapes = std::vector<Object*>(objects.begin() + best_div,objects.end());
                //forget this sentence! fuck!
                best_cost = min_cost;
            }
        }
        //std::cout<<"Current objects.size() = "<<objects.size()<<std::endl;

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        assert(leftshapes.size() != 0);
        assert(rightshapes.size() != 0);

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsNeg{};
    for(int i=0; i<3; i++){
        dirIsNeg[i] = ray.direction[i] < 0;
    };

    bool intersectBound = node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg);
    Intersection inter;
    if(!intersectBound)return inter;
    if(node->left == nullptr && node->right == nullptr){
        inter = node->object->getIntersection(ray);
        return inter;
    }
    Intersection l = getIntersection(node->left,ray);
    Intersection r = getIntersection(node->right,ray);
    inter = l.distance < r.distance ? l : r;
    return inter;
}
