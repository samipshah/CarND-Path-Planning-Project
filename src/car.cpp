#include <src/car.hpp>
#include <src/road.hpp>

bool Car::same_lane(const Car& b) {
    return get_lane(*this) == get_lane(b);
}
