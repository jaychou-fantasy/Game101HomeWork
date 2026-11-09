#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,SplitMethod splitMethod)
    : primitives(std::move(p)),maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod)
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
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
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