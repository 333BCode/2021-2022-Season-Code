#ifdef DRIVETRAIN_HPP
#ifndef PATH_HPP
#define PATH_HPP

class Drivetrain::Path final {
public:

    struct Velocities {

        long double linearVoltage;
        long double rotVoltage;
        
        long double xExtension;
        long double yExtension;
    
    };

    Path(const Path& path);
    Path(Path&& path);
    Path(Velocities* path, size_t length, Point target, long double lookAheadDist);

    ~Path();

    void operator=(const Path&) = delete;

    Path& withAction(std::function<void()>&& action, double dist);

    static Path generatePathTo(Point point);
    static Path generatePath(Point start, Point end);
    static Path generatePathTo(Point point, long double lookAheadDist = defaultLookAheadDistance);
    static Path generatePath(Point start, Point end, long double lookAheadDist = defaultLookAheadDistance);

    Velocities operator[](size_t index) const;

    const Velocities* begin() const;
    const Velocities* end() const;

    size_t size() const;

    friend class Drivetrain;

private:

    Path(Point target, long double lookAheadDist);

    Point target;

    Velocities* data;

    size_t length;
    size_t capacity;

    const long double lookAheadDistance;

    void add(long double linearVoltage, long double rotVoltage, long double xExtension, long double yExtension);

};

#endif
#endif