#ifdef DRIVETRAIN_HPP
#ifndef PATH_HPP
#define PATH_HPP

class Drivetrain::Path final {
public:

    struct Velocities {
        long double leftVelocity;
        long double rightVelocity;
        float distanceAlongPath;
    };

    Path(const Path& path);
    Path(Path&& path);
    Path(Velocities* path, size_t length, long double targetX, long double targetY, float totalDist);

    ~Path();

    void operator=(const Path&) = delete;

    Path& withAction(std::function<void()>&& action, double dist);

    static Path generatePathTo(Point point);
    static Path generatePath(Point start, Point end);

    Velocities operator[](size_t index) const;

    const Velocities* begin() const;
    const Velocities* end() const;

    size_t size() const;

    friend class Drivetrain;

private:

    Path(long double targetX, long double targetY, float totalDist);

    long double targetX;
    long double targetY;

    Velocities* data;

    size_t length;
    size_t capacity;

    const float totalDist;

    void add(long double leftVelocity, long double rightVelocity, float distanceAlongPath);

};

#endif
#endif