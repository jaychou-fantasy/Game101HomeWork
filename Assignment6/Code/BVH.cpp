#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <vector>
#include "BVH.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Object.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,SplitMethod splitMethod)
    : primitives(std::move(p)),maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod)
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if(splitMethod == SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else
        root = recursiveBuild_SAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
    if(splitMethod == SplitMethod::SAH)
        printf("Use SAH!!!");
    else
        printf("Use BVH!!!");
        
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    //the exit conition 
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
                        Union(centroidBounds, objects[i]->getBounds().Centroid());//this is not the bounding box of all objects (it just bound all centroid)
                        //by list the max/min xyz of "the centroid of all objects' bounding box"
                        //used for getting the largest dimension extension
        int dim = centroidBounds.maxExtent();
        //sort by order of the axis(dimension) that has max extension
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });//sort by order of smaller to larger
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

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));//check again to ensure there no object leap

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }//the first time run this function , get the root node.
    //so if you : Node* root = recursivebuild(...),then get the root nodex

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild_SAH(std::vector<Object*> objects)
{
    assert(!objects.empty());

    BVHBuildNode* node = new BVHBuildNode();

    //compute bounds of all primitives
    Bounds3 bounds;
    for(int i=0;i<objects.size();i++)
        bounds = Union(bounds,objects[i]->getBounds());
    //exit condition
    if(objects.size() == 1)
    {
        //the leaf node
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if(objects.size() == 2)
    {
        //directly divided into two parts
        node->left = recursiveBuild_SAH(std::vector{objects[0]});
        node->right = recursiveBuild_SAH(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else 
    {
        struct Bucket{
            int cnt = 0;
            Bounds3 bounds;
            std::vector<Object*> obj_buc; 
        };
        //you can promote 12 as a parameter,  constptr or a function parameter
        std::array<Bucket, 12> buckets{};
        Bounds3 centroidBound;//the bounding box covering the centroid of object in it
        //simply just divided by axis that has max extension
        for(Object* object : objects)
        {
            centroidBound = Union(centroidBound,object->getBounds().Centroid());
        }

        int dim = centroidBound.maxExtent();
        float  min_center = centroidBound.pMin[dim];
        float  extention = centroidBound.pMax[dim] - min_center;
        //divide into bucket
        for(Object* object : objects)
        {
            Bounds3 objectBounds = object->getBounds();
            float center = objectBounds.Centroid()[dim];
            int idx = std::clamp(
                static_cast<int>(12 * (center - min_center) / extention),
                0,11);//the index of bucket
                //as the last object in [11],but idx =12,so clamp it
            buckets[idx].cnt++;
            buckets[idx].bounds = Union(buckets[idx].bounds , objectBounds);
            buckets[idx].obj_buc.push_back(object);
        }
        //Now compare the score of this 11 cut options
        std::array<int,12> leftCount{},rightCount{};
        std::array<Bounds3,12> leftBounds{},rightBounds{};

        //leftCount[i] : the accumulated cnt from i (0 to i)
        //same as leftBound[i]
        Bounds3 accumulateBounds;
        int accumulateCount = 0;
        for(int i = 0;i < 12;i++)
        {
            if(buckets[i].cnt > 0)
            {   //union calculate harder than integer
                accumulateBounds = Union(accumulateBounds,buckets[i].bounds);
            }
            accumulateCount += buckets[i].cnt;
            leftCount[i] = accumulateCount;
            leftBounds[i] = accumulateBounds;
        }
        //right (cut_i to 11)
        accumulateBounds = Bounds3();
        accumulateCount = 0;
        for(int i = 11;i >= 0 ;i--)
        {
            if(buckets[i].cnt > 0)
            {
                accumulateBounds = Union(accumulateBounds,buckets[i].bounds);
            }
            accumulateCount += buckets[i].cnt;
            rightCount[i] = accumulateCount;
            rightBounds[i] = accumulateBounds;
        }

        //calculate the score
        float bestscore = std::numeric_limits<float>::infinity();
        int bestcut = -1;

        for(int cut = 0 ; cut < 11; cut++)
        {
            int count_l = leftCount[cut]; // bucket 0~cut
            int count_r = rightCount[cut+1]; // bucket cut+1~11
            if(count_l == 0 || count_r == 0)
            {
                continue;//meaingless cut
            }
            float score = leftBounds[cut].SurfaceArea() * count_l 
                        + rightBounds[cut+1].SurfaceArea() * count_r;
            if(score < bestscore)
            {
                bestscore = score;
                bestcut = cut;
            }
        }
        //now execute the true cut logic
        // auto middle = objects.begin() + objects.size() / 2;
        // if (bestCut >= 0){
        //     middle = std::partition(objects.begin(), objects.end(),
        //         [&](Object* object) ---------> this:get the BVH instance, &: get the temporary varible in this function(like dim...)
        //         {
        //             Bounds3 objectBounds = object->getBounds();
        //             float center = objectBounds.Centroid()[dim];
        //             int idx = std::clamp(static_cast<int>(12 * (center - min_center) / extention),0, 11);
        //             return idx <= bestCut;
        //         });
        std::vector<Object*> leftObjects{},rightObjects{};
        for(int i = 0;i < 12 ; i++)
        {
            if(i <= bestcut)
            {
                leftObjects.insert(leftObjects.end(),
                                    buckets[i].obj_buc.begin() , 
                                    buckets[i].obj_buc.end());
            }
            else {
                rightObjects.insert(rightObjects.end(),
                                    buckets[i].obj_buc.begin() , 
                                    buckets[i].obj_buc.end());
            }
        }
        assert(objects.size() == leftObjects.size() + rightObjects.size());
        node->left = recursiveBuild_SAH(leftObjects);
        node->right = recursiveBuild_SAH(rightObjects);
        node->bounds = Union(node->left->bounds,node->right->bounds);
    }//object.size() > 2, can't directly exit, only exit when size() == 1 || 2
    return node;
}


//the launcher of intersect's recursive process,return the result of the recursive process
Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

//true recursive logic
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    //for this ray 
    Intersection inter;
    std::array<int,3> dirIsPostive = {(ray.direction.x>0),(ray.direction.y >0),ray.direction.z >0};    
    
    //iterate all node to find the exact(&smallest) bounding box that happened to intersect with the ray
    //only confirm truly has intersection can calculate the exact intersection
    if(node->bounds.IntersectP(ray,ray.direction_inv,dirIsPostive)) 
    {
        //exiting condition : the child node have no intersection with ray (and node itself had)
        if(node->left == nullptr && node->right == nullptr)
        {
            //only find the smallest bounding box , can you call the exact "Intersection Function between Ray and the Triangle"
            return node->object->getIntersection(ray);
            //the first time when the object is Bunny rabbit,object == one whole meshtriangle
            //then call the bvh instance in this object,call this function again(now in leaf node), this time , the object ==. each triangle in Bunny
        }
        Intersection left_inter = getIntersection(node->left, ray);
        Intersection right_inter = getIntersection(node->right, ray);
        if(!left_inter.happened)
            return right_inter;
        if(!right_inter.happened)
            return left_inter;
        //return the closer Intersection
        return left_inter.distance < right_inter.distance ? left_inter : right_inter;
    }
    return inter;
}