//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "snippet_gpmds/snippet_gpmds.h"

SnippetGPMDS::SnippetGPMDS()
        : SnippetGPMDS(nullptr) {
}

SnippetGPMDS::SnippetGPMDS(
        GaussianProcessModulatedDS<double>::DynamicalSystem original_dynamics) {
  gp_mds_.reset(new GaussianProcessModulatedDS<double>(original_dynamics));

}

SnippetGPMDS::~SnippetGPMDS() {

}
