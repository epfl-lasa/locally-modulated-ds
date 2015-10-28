//

// The snippet-based GPMDS uses
// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#ifndef PROJECT_SNIPPET_GPMDS_H
#define PROJECT_SNIPPET_GPMDS_H

#include <locally_modulated_ds/gp_modulated_ds.h>
#include <memory>

class SnippetGPMDS {

 public:

    SnippetGPMDS();

    SnippetGPMDS(GaussianProcessModulatedDS<double>::DynamicalSystem original_dynamics);

    virtual ~SnippetGPMDS();

 protected:

    std::unique_ptr<GaussianProcessModulatedDS<double>> gp_mds_;
    GaussianProcessModulatedDS<double>::DynamicalSystem original_dynamics_;

};


#endif //PROJECT_SNIPPET_GPMDS_H
