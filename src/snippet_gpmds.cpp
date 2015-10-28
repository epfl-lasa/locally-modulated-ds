//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "snippet_gpmds.h"

SnippetGPMDS::SnippetGPMDS()
        : SnippetGPMDS(nullptr) {
}

SnippetGPMDS::SnippetGPMDS(
        GaussianProcessModulatedDS<double>::DynamicalSystem original_dynamics)
        : original_dynamics_(original_dynamics) {

}

SnippetGPMDS::~SnippetGPMDS() {

}
