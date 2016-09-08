#ifndef LOCALLY_MODULATED_DS_H
#define LOCALLY_MODULATED_DS_H



#include <functional>

/** 
    \brief Base class for creating Locally Modulated DS

 */

template<typename Vec, typename Mat>
class LocallyModulatedDS {
 protected:
    std::function<Vec(Vec)> original_dynamics_;
 public:
    LocallyModulatedDS() { };
  /** 
      \brief Constructor.
      @param original_dynamics function handle for evaluating original dynamics.

  */

    LocallyModulatedDS(std::function<Vec(Vec)> original_dynamics) {
      set_original_dynamics(original_dynamics);
    };
  /** 
      \brief Update the oriignal dynamics 

      @param original_dynamics function handle for evaluating original dynamics.
  */


    void set_original_dynamics(std::function<Vec(Vec)> original_dynamics) {
      original_dynamics_ = original_dynamics;
    }

    std::function<Vec(Vec)> get_original_dynamics() const {
      return original_dynamics_;
    }

    // pure virtual methods
  /** 
      \brief This is the heart of LMDS. Your implementation should implement this function to define reshaping of the DS.  
  */

  virtual Mat ModulationFunction(const Vec &) = 0;
  /** 
      \brief Get the output of the DS. 

      @param in location where you want to get the velocity from the DS
  */
    virtual Vec GetOutput(const Vec &in) {
      return ModulationFunction(in) * original_dynamics_(in);
    }
};

#endif /* LOCALLY_MODULATED_DS_H */
