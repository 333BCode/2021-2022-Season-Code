#ifdef DRIVETRAIN_HPP
#ifndef PATH_HPP
#define PATH_HPP

#define PATH_POLYNOMIAL_ARGS(start, end)                \
    start * -6 + vx1 * -3 + end * 6 + vx2 * -3, 5,      \
    start * 15 + vx1 * 8 + end * -15 + vx2 * 7, 4,      \
    start * -10 + vx1 * -6 + end * 10 + vx2 * -4, 3,    \
    vx1, 1,                                             \
    start, 0

class Drivetrain::Path final {
public:

    struct Velocities {
        long double leftVelocity;
        long double rightVelocity;
    };

    Path();
    Path(const Path& path);
    Path(Path&& path);
    Path(Velocities* path, size_t length);
    Path(Velocities* path, size_t size, size_t capacity);

    ~Path();

    void operator=(const Path&) = delete;

    static Path generatePathTo(XYHPoint point);
    static Path generatePathFromOrigin(long double startingHeading, XYHPoint point);
    static Path generatePath(XYHPoint start, XYHPoint end);

    Velocities operator[](size_t index);

    size_t size();

private:

    Velocities* data;

    size_t length;
    size_t capacity;

    void add(long double leftVelocity, long double rightVelocity);

};

#endif
#endif