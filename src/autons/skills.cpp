#include "autonomous.hpp"

#define releaseMogoMotion()                     \
    lift.release(); pros::delay(200);           \
    lift.setSubposition(Subposition::neutral);  \
    base.moveForward(-0.75_ft)

#define transfer()                      \
    lift.lower();                       \
    base.turnTo(180_deg);               \
    holder.release();                   \
    base.endEarly(0.25_ft);             \
    base.moveForward(1_ft, false);      \
    base.endTurnEarly(5_deg);           \
    base.turnTo(0_deg);                 \
    base.endEarly(0.25_ft);             \
    base.addAction(lift.clamp, 0.5_ft); \
    base.moveForward(1.25_ft);           \
    lift.raise();

void skills() {

    // get alliance on platform, leftmost neutral mogo
    pros::delay(500);

    base.setPosition(28.5_in, 1_ft, 180_deg);

    base.limitSpeed(20);
    base.moveForward(-6_in);
    
    holder.grab();
    lift.release();
    pros::delay(250);

    base.setFollowDirection(Direction::forward);

    base.limitSpeed(40);
    base.addAction(lift.clamp, 3.75_ft);
    base.addAction(base.unboundTurnSpeed, 1_ft);
    base << Waypoint {3.25_ft, 9_ft}.withAction(intake.intake, 5_ft);

    // drop alliance mogo, put neutral on platform
    // base.endEarly(0.25_ft);
    // base.endTurnEarly(10_deg);
    base.endTurnEarly(10_deg);
    base.endEarly(2_in);
    base.moveTo(3_ft, 9_ft, 0_deg);
    intake.stop();
    base.setFollowDirection(Direction::autoDetermine);

    base.limitLinearSpeed(20);
    lift.raise();
    lift.setSubposition(Subposition::high);
    base.endEarly(0.5_ft);
    base.endTurnEarly(10_deg);
    // base.addAction(holder.release, 2_ft);
    base.moveTo(5.5_ft, 9_ft, 90_deg);
    base.moveForward(0.65_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(1000);
    releaseMogoMotion();

    // get dropped alliance mogo, place on platform
    /*base.limitLinearSpeed(40);
    base.endTurnEarly(10_deg);
    base.turnTo(180_deg);
    lift.lower();
    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.75_ft);
    base.moveTo(3_ft, 9.5_ft);*/
    transfer();
    base.limitLinearSpeed(20);
    base.endTurnEarly(10_deg);
    base.turnTo(90_deg);
    base.moveForward(0.75_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    releaseMogoMotion();

    base.unboundLinearSpeed();

    lift.lower();
    base.endTurnEarly(10_deg);
    base.turnTo(0_deg);

////////////////////////////////////////////////////////////////////////

    // pick up alliance mogo, tall netural mogo, place on platform
    base.moveTo(1_ft, 9_ft);
    holder.grab();
    pros::delay(500);

    base.limitLinearSpeed(40);
    intake.intake();
    base.endEarly(0.25_ft);
    base << Waypoint {3.5_ft, 9_ft} << Waypoint {7.5_ft, 4.75_ft}.withAction(lift.clamp, 3_ft);
    lift.raise();
    base.turnTo(-95_deg);

    base.limitLinearSpeed(20);
    base.endEarly(0.25_ft);
    base.moveForward(base.getPosition().y - 2.65_ft, false);
    intake.stop();
    lift.setSubposition(Subposition::low);
    pros::delay(1500);
    lift.release();
    lift.setSubposition(Subposition::neutral);
    base.moveForward(-1_ft);

    base.limitLinearSpeed(40);

    // drop alliance mogo, put on platform
    lift.lower();
    base.turnTo(180_deg);
    holder.release();
    base.endEarly(0.25_ft);
    base.moveForward(1_ft, false);
    base.endTurnEarly(5_deg);
    base.turnTo(0_deg);

    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.5_ft);
    base.moveForward(1.25_ft);

    lift.raise();
    pros::delay(500);
    base.limitLinearSpeed(20);
    base.turnTo(-90_deg);

    base.moveForward(0.75_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(250);
    releaseMogoMotion();

    // get alliance mogo, rightmost neutral mogo
    base.unboundLinearSpeed();

    lift.lower();
    base.endTurnEarly(10_deg);
    base.turnTo(180_deg);

    base << Waypoint {11.75_ft, 3_ft};
    base.moveTo(11.75_ft, 3_ft, Drivetrain::defaultLinearExit<1000, 50000, 10000, 200000, 150, 200>);
    holder.grab();
    pros::delay(500);

    base.endTurnEarly(10_deg);
    base.moveTo(9_ft, 3_ft, 90_deg);

    base.limitLinearSpeed(40);
    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 2.25_ft);
    base.moveForward(5.25_ft);
    lift.raise();
    base.turnTo({6_ft, 9.5_ft});
    intake.intake();
    base.limitLinearSpeed(20);
    base.moveTo(6.5_ft, 9.5_ft);

    lift.release();
    base.moveForward(-0.75_ft);

    // drop alliance mogo, place on platform
    lift.lower();
    base.turnTo(180_deg);
    holder.release();
    base.endEarly(0.25_ft);
    base.moveForward(1_ft, false);
    base.endTurnEarly(5_deg);
    base.turnTo(0_deg);

    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.5_ft);
    base.moveForward(1.5_ft);

    lift.raise();
    base.limitLinearSpeed(20);
    base.turnTo(-90_deg);

    base.moveForward(0.75_ft, false);
    base.limitLinearSpeed(40);
    lift.release();
    base.endEarly(0.25_ft);
    base.moveForward(-0.75_ft);

    // get final alliance mogo
    base.unboundLinearSpeed();
    lift.lower();
    base << Waypoint {12_ft, 9_ft};
    pros::delay(500);
    base.turnTo({9_ft, 11_ft});
    base.limitLinearSpeed(40);
    base.endEarly(0.5_ft);
    base.addAction(lift.clamp, 0.75_ft);
    base.moveTo(9_ft, 11_ft);

    // put final mogo on platform

}