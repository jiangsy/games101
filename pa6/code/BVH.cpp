#include <algorithm>
#include <cassert>
#include "BVH.hpp"

constexpr int nBuckets = 12;
struct BucketInfo {
    int count = 0;
    Bounds3 bounds;
};

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuildWithSAH(primitives);

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
    for (auto & object : objects)
        bounds = Union(bounds, object->getBounds());
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
        std::sort(objects.begin(), objects.end(), [=](auto f1, auto f2) {
            return f1->getBounds().Centroid()[dim] <
                   f2->getBounds().Centroid()[dim];
        });

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

BVHBuildNode* BVHAccel::recursiveBuildWithSAH(std::vector<Object*> objects)
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
        for (auto & object : objects)
            centroidBounds =
                    Union(centroidBounds, object->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();

        std::sort(objects.begin(), objects.end(), [=](auto f1, auto f2) {
            return f1->getBounds().Centroid()[dim] <
                   f2->getBounds().Centroid()[dim];
        });

        BucketInfo buckets[nBuckets];
        for (auto & object: objects) {
            int b = nBuckets *
                    centroidBounds.Offset(object->getBounds().Centroid())[dim];
            if (b == nBuckets) b = nBuckets - 1;
            buckets[b].count++;
            buckets[b].bounds = Union(buckets[b].bounds, object->getBounds());
        }

        float min_cost = std::numeric_limits<float>::max(), tmp_cost;
        int min_cost_bucket_index = -1;
        for (int i = 0; i < nBuckets - 1; ++i) {
            Bounds3 b0, b1;
            int count0 = 0, count1 = 0;
            for (int j = 0; j <= i; ++j) {
                b0 = Union(b0, buckets[j].bounds);
                count0 += buckets[j].count;
            }
            for (int j = i+1; j < nBuckets; ++j) {
                b1 = Union(b1, buckets[j].bounds);
                count1 += buckets[j].count;
            }
            // the constant is useless since we always split until single object is left
            tmp_cost = 0.125f +  (count0 * b0.SurfaceArea() +
                                  count1 * b1.SurfaceArea()) / bounds.SurfaceArea();
            if (tmp_cost < min_cost) {
                min_cost = tmp_cost;
                min_cost_bucket_index = i;
            }
        }

        auto pmid = std::partition(objects.begin(), objects.end(),
                                   [=](auto object) {
            int b = nBuckets * centroidBounds.Offset(object->getBounds().Centroid())[dim];
            if (b == nBuckets) b = nBuckets - 1;
            return b <= min_cost_bucket_index;
        });

        auto leftshapes = std::vector<Object*>(objects.begin(), pmid);
        auto rightshapes = std::vector<Object*>(pmid, objects.end());

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildWithSAH(leftshapes);
        node->right = recursiveBuildWithSAH(rightshapes);

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
    Intersection isect;
    auto dirIsNeg = std::array<int, 3> {int(ray.direction.x>0),
                                        int(ray.direction.y>0),
                                        int(ray.direction.z>0),
                                        };

    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        return isect;
    }

    if (node->left == nullptr && node->right == nullptr) {
        return node->object->getIntersection(ray);
    }

    Intersection left_isect, right_isect;
    left_isect = getIntersection(node -> left, ray);
    right_isect = getIntersection(node -> right, ray);

    if (left_isect.distance < right_isect.distance) {
        return left_isect;
    } else {
        return right_isect;
    }

}