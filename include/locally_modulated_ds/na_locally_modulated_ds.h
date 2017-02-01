#ifndef LOCALLY_MODULATED_DS_H
#define LOCALLY_MODULATED_DS_H



#include <functional>
#include <iostream>

template<typename Vec, typename Mat>
class NA_LocallyModulatedDS {
protected:
    std::function<Vec(Vec)> original_dynamics_;
public:
    NA_LocallyModulatedDS() { };

    NA_LocallyModulatedDS(std::function<Vec(Vec)> original_dynamics) {
        set_original_dynamics(original_dynamics);
    };

    void set_original_dynamics(std::function<Vec(Vec)> original_dynamics) {
        original_dynamics_ = original_dynamics;
    }

    std::function<Vec(Vec)> get_original_dynamics() const {
        return original_dynamics_;
    }

    // pure virtual methods
    virtual Mat ExternalModulationFunction2(const Vec&, double) = 0;

    virtual Vec GetOutput(const Vec &in, double ext) {
        /// Version 1 as in the paper
        return ExternalModulationFunction2(in, ext) * original_dynamics_(in);
    }


    Vec operator()(const Vec& in){
        return this->GetOutput(in, 1.0);
    }
};

#endif /* LOCALLY_MODULATED_DS_H */
