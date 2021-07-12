#include "drivetrain.hpp"

using namespace drive;

constexpr size_t defaultAllocCapacity   = 500;
constexpr size_t reallocAddition        = 100;

Path::Path()
    : data {new Velocities[defaultAllocCapacity]}, length {0}, capacity {defaultAllocCapacity} {}

Path::Path(const Path& path)
    : length {path.length}, capacity {path.length}
{
    if (capacity) {

        data = new Velocities[capacity];
        for (size_t i = 0; i < length; ++i) {
            data[i] = path.data[i];
        }

    } else {
        data = nullptr;
    }
}

Path::Path(Path&& path)
    : data {path.data}, length {path.length}, capacity {path.capacity}
{
    if (capacity) {
        path.data = nullptr;
        path.length = 0;
        path.capacity = 0;
    }
}

Path::Path(Velocities* path, size_t length)
    : data {path}, length {length}, capacity {length} {}

Path::Path(Velocities* path, size_t size, size_t capacity)
    : data {path}, length {size}, capacity {capacity} {}

Path::~Path() {
    if (data) {
        delete[] data;
    }
}

void Path::add(long double leftVelocity, long double rightVelocity) {

    if (length == capacity) {

        capacity += reallocAddition;
        Velocities* newData = new Velocities[capacity];

        for (size_t i = 0; i < length; ++i) {
            newData[i] = data[i];
        }

        delete[] data;
        data = newData;

    }

    data[length] = {leftVelocity, rightVelocity};
    ++length;

}

Path::Velocities Path::operator[](size_t index) {
    return data[index];
}

size_t Path::size() {
    return length;
}

Path Path::generatePathTo(XYHPoint point) {
    return generatePath({xPos, yPos, heading}, point);
}

Path Path::generatePathFromOrigin(long double startingHeading, XYHPoint point) {
    return generatePath({0, 0, startingHeading}, point);
}

Path Path::generatePath(XYHPoint start, XYHPoint end) {
    return {};
}