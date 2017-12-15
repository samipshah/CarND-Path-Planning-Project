#include <src/path.hpp>
#include <assert.h>

using namespace std;

unsigned int Path::remaining_points() {
    assert(m_max_points >= m_x.size());
    return m_max_points - m_x.size();
}

unsigned int Path::length() {
    return m_x.size();
}