#ifdef DRIVETRAIN_HPP
#ifndef PATH_HPP
#define PATH_HPP

#define PATH_POLYNOMIAL_ARGS(start, end, v1, v2)        \
    start * -6 + v1 * -3 + end * 6 + v2 * -3, 5,        \
    start * 15 + v1 * 8 + end * -15 + v2 * 7, 4,        \
    start * -10 + v1 * -6 + end * 10 + v2 * -4, 3,      \
    v1, 1,                                              \
    start, 0

class Drivetrain::Path final {
public:

    struct Velocities {
        long double leftVelocity;
        long double rightVelocity;
        float distanceAlongPath;
    };

    Path();
    Path(const Path& path);
    Path(Path&& path);
    Path(Velocities* path, size_t length);
    Path(Velocities* path, size_t size, size_t capacity);

    ~Path();

    void operator=(const Path&) = delete;

    Path& withAction(std::function<void()>&& action, double dist, bool duringTurn = false);

    static Path generatePathTo(XYHPoint point);
    static Path generatePathFromOrigin(long double startingHeading, XYHPoint point);
    static Path generatePath(XYHPoint start, XYHPoint end);

    Velocities operator[](size_t index) const;

    const Velocities* begin() const;
    const Velocities* end() const;

    size_t size() const;

    friend class Drivetrain;

private:

    Velocities* data;

    size_t length;
    size_t capacity;

    std::vector<Action> actions;

    void add(long double leftVelocity, long double rightVelocity, float distanceAlongPath);

};

#endif
#endif