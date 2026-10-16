#include "swrm_flocking_core/utilities.hpp"

#include <iostream>
#include <iomanip>

#include "swrm_flocking_core/types.hpp"

void print_hpscalar(hpscalar value) {
    std::cout << std::setprecision(HP_PRECISION) << value << std::endl;
}
