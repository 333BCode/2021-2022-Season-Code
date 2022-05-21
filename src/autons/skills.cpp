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
    // pros::delay(250);

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
    base.limitLinearSpeed(30);
    lift.raise();
    lift.setSubposition(Subposition::high);
    // base << Waypoint {5.25_ft, 9_ft};
    base.endEarly(0.5_ft);
    base.endTurnEarly(5_deg);
    // base.addAction(holder.release, 2_ft);
    base.moveTo(5.5_ft, 9_ft, 90_deg);
    base.setFollowDirection(Direction::autoDetermine);
    base.endEarly(0.25_ft);
    base.moveForward(0.75_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(1200);
    intake.reverse();
    releaseMogoMotion();

    // get dropped alliance mogo, place on platform
    /*base.limitLinearSpeed(40);
    base.endTurnEarly(10_deg);
    base.turnTo(180_deg);
    lift.lower();
    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.75_ft);
    base.moveTo(3_ft, 9.5_ft);*/
    intake.stop();
    transfer();
    pros::delay(200);
    base.limitLinearSpeed(30);
    base.endTurnEarly(10_deg);
    base.turnTo(90_deg);
    base.moveForward(1_ft, false, quickLinearExit);
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    releaseMogoMotion();

    base.unboundLinearSpeed();

    lift.lower();
    base.endTurnEarly(10_deg);
    base.turnTo(0_deg);

////////////////////////////////////////////////////////////////////////

    // pick up alliance mogo, tall netural mogo, place on platform
    
    base.addAction(holder.grab, 1.5_in);
    base.moveTo(1.25_ft, 9_ft);
    holder.grab();
    pros::delay(500);

    base.limitLinearSpeed(40);
    intake.intake();
    base.endEarly(0.25_ft);
    base << Waypoint {3.5_ft, 9_ft} << Waypoint {7.5_ft, 4.75_ft}.withAction(lift.clamp, 3_ft);
    lift.raise();
    base.endTurnEarly(1_deg);
    base.turnTo(-95_deg);

    base.limitLinearSpeed(30);
    base.endEarly(0.25_ft);
    base.moveForward(base.getPosition().y - 2.65_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(1000);
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
    base.endTurnEarly(5_deg);
    base.turnTo(-95_deg);
    base.limitLinearSpeed(30);

    base.moveForward(1_ft, false, quickLinearExit);
    lift.setSubposition(Subposition::low);
    pros::delay(250);
    releaseMogoMotion();

    // get alliance mogo, rightmost neutral mogo
    base.unboundLinearSpeed();

    lift.lower();
    base.endTurnEarly(10_deg);
    base.turnTo(180_deg);

    base << Waypoint {11.25_ft, 3_ft};
    base.moveTo(11.25_ft, 3_ft, Drivetrain::defaultLinearExit<1000, 50000, 10000, 200000, 150, 200>);
    holder.grab();
    pros::delay(500);

    base.endTurnEarly(10_deg);
    base.endEarly(1_in);
    base.moveTo(9_ft, 3.5_ft, 90_deg, quickLinearExit);

///////////////////////////////////////////////////////////////

    base.setFollowDirection(Direction::forward);

    base.limitSpeed(40);
    base.addAction(lift.clamp, 0.75_ft);
    base.endEarly(0.5_ft);
    base.moveTo(9_ft, 7_ft);

    intake.intake();

    lift.raise();
    base.limitLinearSpeed(30);
    // base << Waypoint {6_ft, 9_ft};
    base.endEarly(0.5_ft);
    base.endTurnEarly(10_deg);
    base.moveTo(5.75_ft, 9_ft, 90_deg);
    base.setFollowDirection(Direction::autoDetermine);
    base.endEarly(0.25_ft);
    base.addAction(bundle(lift.setSubposition(Subposition::low)), 0.4_ft);
    base.moveForward(0.75_ft, false, quickLinearExit);


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
    // lift.setSubposition(Subposition::low);
    // pros::delay(200);
    intake.reverse();
    releaseMogoMotion();

    // drop alliance mogo, place on platform
    lift.lower();
    base.turnTo(180_deg);
    intake.stop();
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
    base.limitLinearSpeed(30);
    base.endTurnEarly(1_deg);
    base.turnTo(100_deg);

    base.moveForward(1.15_ft, false, quickLinearExit);
    lift.setSubposition(Subposition::low);
    base.limitLinearSpeed(40);
    releaseMogoMotion();

    // get final alliance mogo
    base.setFollowDirection(Direction::reverse);
    base.unboundLinearSpeed();
    lift.lower();
    base << Waypoint {12_ft, 9_ft};
    base.setFollowDirection(Direction::autoDetermine);
    base.endTurnEarly(10_deg);
    base.turnTo({8.60_ft, 11_ft}, true);
    base.limitLinearSpeed(40);
    base.endEarly(0.75_ft);
    base.addAction(lift.clamp, 0.8_ft);
    base.moveTo(8.60_ft, 11_ft);
    base.setFollowDirection(Direction::reverse);

    // put final mogo on platform
    base.unboundLinearSpeed();
    base.endEarly(0.5_ft);
    base.moveTo(9_ft, 5_ft);
    base.setFollowDirection(Direction::forward);

    base.limitLinearSpeed(30);
    lift.raise();

    base.endEarly(0.5_ft);
    base.endTurnEarly(1_deg);
    base.moveTo(6_ft, 3.15_ft, -90_deg);

    base.endEarly(0.25_ft);
    base.moveForward(0.75_ft, false);
    // pros::delay(250);
    
    base.setFollowDirection(Direction::reverse);

    lift.release(); // pros::delay(200);
    // lift.setSubposition(Subposition::neutral);
    base.moveForward(-1.75_ft);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef _COMMENT_
void skills2() {

    LinearExitConditions quickLinearExit = Drivetrain::defaultLinearExit<1000, 1000, 1000, 2000, 0, 1500>;

    base.setPosition(28.5_in, 1_ft, 180_deg);

    base.limitSpeed(20);
    do {
        base.moveForward(-6_in);
    } while (30.5 - base.getPosition().x > 0.25);

    holder.grab();
    lift.release();
    // pros::delay(250);

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
    base.limitLinearSpeed(30);
    lift.raise();
    lift.setSubposition(Subposition::high);
    // base << Waypoint {5.25_ft, 9_ft};
    base.endEarly(0.5_ft);
    base.endTurnEarly(5_deg);
    // base.addAction(holder.release, 2_ft);
    base.moveTo(5.5_ft, 9_ft, 90_deg);
    base.setFollowDirection(Direction::autoDetermine);
    base.endEarly(0.25_ft);
    base.moveForward(0.75_ft, false);
    lift.setSubposition(Subposition::low);
    pros::delay(1200);
    intake.reverse();
    releaseMogoMotion();

    // get dropped alliance mogo, place on platform
    /*base.limitLinearSpeed(40);
    base.endTurnEarly(10_deg);
    base.turnTo(180_deg);
    lift.lower();
    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.75_ft);
    base.moveTo(3_ft, 9.5_ft);*/
    intake.stop();
    transfer();
    pros::delay(200);
    base.limitLinearSpeed(30);
    base.endTurnEarly(10_deg);
    base.turnTo(90_deg);
    base.moveForward(0.75_ft, false, quickLinearExit);
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    releaseMogoMotion();


    base.limitLinearSpeed(40);

    // base.unboundLinearSpeed();

    lift.lower();
    base.endTurnEarly(10_deg);
    base.turnTo(-90_deg);

    // get tall mogo

    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 1_ft);
    base.moveTo(6_ft, 6_ft);
    base.limitTurnSpeed(4000);
    lift.setSubposition(Subposition::high);
    // lift.raise();
    base.endTurnEarly(1_deg);
    base.turnTo(90_deg);
    base.unboundTurnSpeed();
    lift.raise();

    base.limitLinearSpeed(30);

    base.endEarly(1_in);
    base.moveTo(5.95_ft, 9.5_ft);
    
    lift.setSubposition(Subposition::low);
    pros::delay(1500);
    lift.release();
    intake.reverse();
    pros::delay(200);
    base.supply(-40, 0);
    pros::delay(200);
    lift.setSubposition(Subposition::neutral);
    base.moveForward(-1.75_ft, true, quickLinearExit);

    base.limitLinearSpeed(40);

    // get last neutral mogo

    intake.intake();
    lift.lower();
    
    base.endEarly(1_in);
    base.endTurnEarly(5_deg);
    base.moveTo(8_ft, 7.75_ft, {9_ft, 6.5_ft});
    
    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.5_ft);
    base.moveTo(9_ft, 6.5_ft);

    // base.limitLinearSpeed(30);

    lift.raise();
    pros::delay(500);
    base.endTurnEarly(5_deg);
    base.turnTo({6.5_ft, 9_ft}, false);

    base.endEarly(1_in);
    base.moveTo(6.5_ft, 9_ft);
    lift.release();
    base.endEarly(0.25_ft);
    base.moveForward(-0.75_ft, false);

//////////////////////////////////////////////////////////////////////////////////////////////////////////

    // get two alliance mogos
    base.unboundLinearSpeed();
    lift.lower();
    base.endTurnEarly(10_deg);
    base.turnTo(0_deg);
    base.moveTo(1_ft, 9_ft);
    holder.grab();
    pros::delay(500);
    lift.setSubposition(Subposition::high);

    base.unboundLinearSpeed();
    intake.intake();

    base << Waypoint {3_ft, 3.5_ft} /*5_ft} << Waypoint {6_ft, 3_ft}*/ << Waypoint {9_ft, 3.25_ft};
    lift.setSubposition(Subposition::low);

    base.limitLinearSpeed(40);

    base.endEarly(1_in);
    base.endTurnEarly(5_deg);
    base.moveTo(9_ft, 3.25_ft, 0_deg);

    base.endEarly(0.25_ft);
    base.addAction(lift.clamp, 0.5_ft);
    base.moveForward(1.35_ft);

    base.limitLinearSpeed(30);
    lift.setSubposition(Subposition::high);
    intake.reverse();
    base.endEarly(5_deg);
    base.turnTo(95_deg);
    intake.intake();
    base.endEarly(0.25_ft);
    base.moveForward(4_ft);

    base.endTurnEarly(5_deg);
    base.turnTo({7.75_ft, 9.25_ft});
    lift.raise();
    base.endEarly(0.25_ft);
    base.moveForward(4_ft);
    lift.release();
    base.endEarly(0.25_ft);
    base.moveForward(-0.75_ft);
    lift.lower();

    // get final mogo
    base.unboundLinearSpeed();
    lift.lower();
    base << Waypoint {12_ft, 9_ft};
    base.setFollowDirection(Direction::autoDetermine);
    base.endTurnEarly(10_deg);
    base.turnTo({8.60_ft, 11_ft}, true);
    base.limitLinearSpeed(40);
    base.endEarly(0.75_ft);
    base.addAction(lift.clamp, 0.8_ft);
    base.moveTo(8.60_ft, 11_ft);
    base.setFollowDirection(Direction::reverse);

    base.unboundLinearSpeed();
    base.endEarly(0.5_ft);
    base.moveTo(9.25_ft, 3_ft);
    base.setFollowDirection(Direction::autoDetermine);
    lift.raise();
    base.endTurnEarly(45_deg);
    base.turnTo(0);
    base.endTurnEarly(10_deg);
    base.turnTo(-90_deg);

    base.limitLinearSpeed(40);
    base.moveTo(9.25_ft, 1.45_ft, 180_deg, quickLinearExit);
    base.turnTo(180_deg);
    lift.lower();
    pros::delay(1000);

    base.unboundSpeed();
    base.moveForward(4.25_ft);

    // base.supply(127, 0);

}
#endif