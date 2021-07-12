#ifdef DRIVETRAIN_HPP
#ifndef POINT_HPP
#define POINT_HPP

class Drivetrain::Point final {
public:

    Point(long double x, long double y, long double heading = NAN);

    Point& withAction(std::function<void()>&& action, double dist);
    Point& withLookAhead(long double newLookAhead);
    Point& withExitConditions(const ExitConditions& conditions);

    friend class Drivetrain;

private:

    struct Action final {

        Action(std::function<void()>&& newAction, double dist);

        std::function<void()> action;
        double distance;

    };

    long double x;
    long double y;
    long double heading;
    long double lookAheadDistance {defaultLookAheadDistance};
    ExitConditions exitConditions {defaultExitConditions};
    std::vector<Action> actions;

};

struct Drivetrain::XYPoint {
    long double x;
    long double y;
};

struct Drivetrain::XYHPoint {
    long double x;
    long double y;
    long double heading;
};

#endif
#endif