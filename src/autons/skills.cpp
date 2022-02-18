#include "autonomous.hpp"

#define releaseMogoMotion()                     \
    lift.release(); pros::delay(200);           \
    lift.setSubposition(Subposition::neutral);  \
    base.endEarly(1_in);                        \
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

    LinearExitConditions quickLinearExit = Drivetrain::defaultLinearExit<1000, 1000, 1000, 2000, 0, 1500>;

    base.setPosition(28.5_in, 1_ft, 180_deg);

    base.limitSpeed(20);
    do {
        base.moveForward(-6_in);
    } while (30.5 - base.getPosition().x > 0.25);

    holder.grab();
    lift.release();
    pros::delay(250);

    base.setFollowDirection(Direction::forward);

    base.limitSpeed(40);
    base.addAction(lift.clamp, 3.65_ft);
    base.addAction(base.unboundTurnSpeed, 1_ft);
    base << Waypoint {3_ft, 8.5_ft}.withAction(intake.intake, 5_ft);

    // drop alliance mogo, put neutral on platform
    // base.endEarly(0.25_ft);
    // base.endTurnEarly(10_deg);
/*    base.endTurnEarly(10_deg);
    base.endEarly(2_in);
    base.moveTo(3_ft, 9_ft, 0_deg);
    intake.stop();
*/
    base.limitLinearSpeed(20);
    lift.raise();
    lift.setSubposition(Subposition::high);
    // base << Waypoint {5.25_ft, 9_ft};
    base.endEarly(0.5_ft);
    base.endTurnEarly(10_deg);
    // base.addAction(holder.release, 2_ft);
    base.moveTo(5.25_ft, 9_ft, 90_deg);
    base.setFollowDirection(Direction::autoDetermine);
    base.moveForward(0.75_ft, false, quickLinearExit);
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
    base.moveForward(0.75_ft, false, quickLinearExit);
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
    lift.setSubposition(Subposition::low);
    pros::delay(1500);
    lift.release();
    intake.reverse();
    pros::delay(200);
    base.supply(-40, 0);
    pros::delay(200);
    lift.setSubposition(Subposition::neutral);
    base.moveForward(-1_ft, true, quickLinearExit);

    intake.stop();

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
    base.moveTo(6.5_ft, 3_ft, -90_deg);

    base.moveForward(0.75_ft, false, quickLinearExit);
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
    base.moveTo(9_ft, 3.5_ft, 90_deg, quickLinearExit);

///////////////////////////////////////////////////////////////

    base.setFollowDirection(Direction::forward);

    base.limitSpeed(40);
    base.addAction(lift.clamp, 0.75_ft);
    base.endEarly(0.5_ft);
    base.moveTo(9_ft, 7_ft);

    lift.raise();
    lift.setSubposition(Subposition::high);
    base.limitLinearSpeed(20);
    // base << Waypoint {6_ft, 9_ft};
    base.endEarly(0.5_ft);
    base.endTurnEarly(10_deg);
    base.moveTo(6_ft, 9_ft, 90_deg);
    base.setFollowDirection(Direction::autoDetermine);
    base.moveForward(0.5_ft, false, quickLinearExit);


/////////////////////////////////////////////////

    /*base.limitLinearSpeed(40);
    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 2.25_ft);
    base.moveForward(4.75_ft);
    lift.raise();
    base.turnTo(145_deg);
    intake.intake();
    base.limitLinearSpeed(20);
    base.moveForward(2.5_ft, false);*/

    /*lift.release();
    base.moveForward(-0.75_ft);*/
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    releaseMogoMotion();

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
    pros::delay(500);
    base.limitLinearSpeed(20);
    base.turnTo(90_deg);

    base.moveForward(0.75_ft, false);
    base.limitLinearSpeed(40);
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    releaseMogoMotion();

    // get final alliance mogo
    base.setFollowDirection(Direction::reverse);
    base.unboundLinearSpeed();
    lift.lower();
    base << Waypoint {12_ft, 9_ft};
    base.setFollowDirection(Direction::autoDetermine);
    base.turnTo({8.8_ft, 11_ft});
    base.limitLinearSpeed(40);
    base.endEarly(0.5_ft);
    base.addAction(lift.clamp, 0.75_ft);
    base.moveTo(8.8_ft, 11_ft);

    // put final mogo on platform
    base.unboundLinearSpeed();
    base << Waypoint {9_ft, 6_ft} << endAt(6_ft, 3_ft);

    base.limitLinearSpeed(20);
    lift.raise();
    base.turnTo(-90_deg);

    base.moveForward(0.75_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(250);
    
    lift.release(); pros::delay(200);
    lift.setSubposition(Subposition::neutral);
    base.moveForward(-1.75_ft);

}