
#include <functional>
namespace wxpiggy {
struct VoxelKey {
    short x, y, z;
    VoxelKey() = default;
    VoxelKey(short x, short y, short z) : x(x), y(y), z(z) {}
    bool operator==(const VoxelKey &other) const {
        return x==other.x && y==other.y && z==other.z;
    }
};
} // namespace wxpiggy

namespace std {
template <>
struct hash<wxpiggy::VoxelKey> {
    std::size_t operator()(const wxpiggy::VoxelKey &vox) const noexcept {
        const size_t kP1 = 73856093;
        const size_t kP2 = 19349669;
        const size_t kP3 = 83492791;
        return vox.x*kP1 + vox.y*kP2 + vox.z*kP3;
    }
};
}