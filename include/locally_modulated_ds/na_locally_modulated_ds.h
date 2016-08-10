#ifndef LOCALLY_MODULATED_DS_H
#define LOCALLY_MODULATED_DS_H



#include <functional>


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
    virtual Mat ModulationFunction(const Vec &) = 0;
//    virtual Mat ExternalModulationFunction(const Vec &) = 0;
    virtual Mat ExternalModulationFunction(const Mat&, const Vec &) = 0;

    virtual Vec GetOutput(const Vec &in, const Vec &ext) {
//      return ExternalModulationFunction(ext) * ModulationFunction(in) * original_dynamics_(in);
      return ExternalModulationFunction(ModulationFunction(in), ext) * original_dynamics_(in);
    }
};

#endif /* LOCALLY_MODULATED_DS_H */
