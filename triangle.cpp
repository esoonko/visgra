/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Tuesday, October 17, 2017 - 10:24:30
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labraytracer/triangle.h>
#include <labraytracer/util.h>
#include <memory>

namespace inviwo {

Triangle::Triangle() {}

Triangle::Triangle(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& uvw0,
                   const vec3& uvw1, const vec3& uvw2) {
    mVertices[0] = v0;
    mVertices[1] = v1;
    mVertices[2] = v2;
    mUVW[0] = uvw0;
    mUVW[1] = uvw1;
    mUVW[2] = uvw2;
}

bool Triangle::closestIntersection(const Ray& ray, double maxLambda,
                                   RayIntersection& intersection) const {
    // Programming TASK 1: Implement this method
    // Your code should compute the intersection between a ray and a triangle.
    //
    // If you detect an intersection, the return type should look similar to this:
    // if(rayIntersectsTriangle)
    // {
    //   intersection = RayIntersection(ray,shared_from_this(),lambda,ray.normal,uvw);
    //   return true;
    // }
    //
    // Hints :
    // Ray origin p_r : ray.getOrigin()
    // Ray direction t_r : ray.getDirection()
    // Compute the intersection point using ray.pointOnRay(lambda)

    const vec3 r0 = ray.getOrigin();
    const vec3 rd = ray.getDirection();

    // Convert to barycentric coordinates (a, b, c), where (1, 0, 0), (0, 1, 0)
    // and (0, 0, 1) each represent a vertex of the triangle. Anything with an
    // a, b or c value lower than 0 will be outside the triangle.

    // Vectors spanning the plane which supports the triangle.
    const vec3 E1 = mVertices[1] - mVertices[0];
    const vec3 E2 = mVertices[2] - mVertices[0];

    // Normal of the triangle.
    const vec3 N = cross(E1, E2);

    // Determinant.
    const float det = -dot(rd, N);
    const float invdet = 1.0 / det;

    const vec3 A0 = r0 - mVertices[0];
    const vec3 DA0 = cross(A0, rd);

    float u =  dot(E2, DA0) * invdet;
    float v = -dot(E1, DA0) * invdet;
    float t =  dot(A0, N)   * invdet;

    if (u < 0.0 || v < 0.0 || (u+v) > 1 || det < 0.000001)  {
      std::cout << "False i första";
      return false;
    }

    if (t < 0.0 || t + Util::epsilon > maxLambda) {
      std::cout << "False i andra";
      return false;
    }

    const vec3 uvw(0, 0, 0);
    std::cout << "True hela";

    // intersection = RayIntersection(ray, shared_from_this(), lambda1, normal_, uvw);
    // intersection = RayIntersection(ray, shared_from_this(), dist, N, uvw);
    intersection = RayIntersection(ray, shared_from_this(), t, N, uvw);

    return true;
}

bool Triangle::anyIntersection(const Ray& ray, double maxLambda) const {
    RayIntersection temp;
    return closestIntersection(ray, maxLambda, temp);
}

void Triangle::drawGeometry(std::shared_ptr<BasicMesh> mesh,
                            std::vector<BasicMesh::Vertex>& vertices) const {
    auto indexBuffer = mesh->addIndexBuffer(DrawType::Lines, ConnectivityType::None);

    Util::drawLineSegment(mVertices[0], mVertices[1], vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(),
                          vertices);
    Util::drawLineSegment(mVertices[1], mVertices[2], vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(),
                          vertices);
    Util::drawLineSegment(mVertices[2], mVertices[0], vec4(0.2, 0.2, 0.2, 1), indexBuffer.get(),
                          vertices);
}

}  // namespace inviwo
