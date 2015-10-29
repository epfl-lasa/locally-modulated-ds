#include <functional>


template<typename Vec, typename Mat>
class LocallyModulatedDS {
 protected:
    std::function<Vec(Vec)> original_dynamics_;
 public:
    LocallyModulatedDS() { };

    LocallyModulatedDS(std::function<Vec(Vec)> original_dynamics) {
      set_original_dynamics(original_dynamics);
    };

    void set_original_dynamics(std::function<Vec(Vec)> original_dynamics) {
      original_dynamics_ = original_dynamics;
    }

    std::function<Vec(Vec)> getOriginalDynamics() const {
      return original_dynamics_;
    }

    // pure virtual methods
    virtual Mat ModulationFunction(const Vec &) = 0;

    virtual Vec GetOutput(const Vec &in) {
      return ModulationFunction(in) * original_dynamics_(in);
    }
};

