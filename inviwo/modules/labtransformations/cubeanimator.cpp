/*********************************************************************
 *  Author  : Himangshu Saikia
 *  Init    : Thursday, October 12, 2017 - 11:11:30
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <labtransformations/cubeanimator.h>

namespace inviwo
{

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CubeAnimator::processorInfo_
{
    "org.inviwo.CubeAnimator",      // Class identifier
    "Cube Animator",                // Display name
    "KTH Labs",                 // Category
    CodeState::Experimental,  // Code state
    Tags::None,               // Tags
};

const ProcessorInfo CubeAnimator::getProcessorInfo() const
{
    return processorInfo_;
}


CubeAnimator::CubeAnimator()
    :Processor()
    // Ports
    , meshIn_("meshIn")
    , meshOut_("meshOut")
    // Properties 
    // For a FloatProperty 
    // variablename(identifier, display name, init value, minvalue, maxvalue)
    , radius_("radius", "Radius", 6, 1, 8)
    , theta_("theta", "Angle", 0.01, 0, 3.14159265358979323846) {
    // Add ports
    addPort(meshIn_);
    addPort(meshOut_);
    
    // Add properties
    addProperty(radius_);
    addProperty(theta_);
}


void CubeAnimator::process()
{
    // Clone the input mesh
    if (!meshIn_.getData()) return;
    auto mesh = meshIn_.getData()->clone();

    // Get the matrix that defines where the mesh is currently
    auto matrix = mesh->getWorldMatrix();

    // Transform the mesh (TODO)
    matrix *= glm::translate(vec3(radius_.get(), 0, 0));
    matrix *= glm::translate(vec3(0, radius_.get()-1, sin(2*radius_.get())));
    matrix *= glm::rotate(vec3(theta_.get(), 0, 0));

    // Update
    mesh->setWorldMatrix(matrix);
	
    // Set output
    meshOut_.setData(mesh);
}

} // namespace

