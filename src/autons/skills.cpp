#include "autonomous.hpp"

void skills() {

    /**
     * If you need assistance, look at the header files
     *
     * include/drivetrain.hpp
     * include/systems/
     */

    // start right side, 2nd to last tile, aligned with field perimeter
    base.setPosition(9_ft, 1_ft, 90_deg);
    intake.intake();

    // open claw
    lift.release();

    // limit speed to grab goal
    base.limitSpeed(20); // max 60

    // grab far right neutral mogo after slightly pushing it
    base.addAction(bundle(lift.clamp()), 5_in);
    base.moveTo(9_ft, 6.25_ft);

    // slightly raise lift to prevent dragging
    lift.setSubposition(Subposition::high);
    // give lift time to raise
    pros::delay(200);

    // move back, turn left, align with platform
    base.limitSpeed(40); // max 60
    base.moveTo(9_ft, 3_ft);
    base.turnTo(180_deg);

    base.moveTo(6_ft, 3_ft);
    
    // put mogo on platform
    lift.raise();
    // give lift time to raise before turning
    pros::delay(1000);

    // provide stability
    base.limitSpeed(20);

    // drive into platform
    base.turnTo(270_deg);
    base.supply(20, 0);
    pros::delay(200);
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    lift.release();
    pros::delay(200);

    // move back from platform, raise lift over edge of platform
    base.limitSpeed(40);
    base.addAction(bundle(lift.setSubposition(Subposition::neutral)), 4_in); // if lift does not lift before getting stuck on platform, increase distance from target at which lift raises
    base.moveTo(6_ft, 3_ft);

    // face alliance mogo on win point tape line
    base.turnTo(0_deg);
    lift.lower();
    pros::delay(200);

    // grab far right neutral mogo after slightly pushing it
    base.limitSpeed(20);
    base.addAction(bundle(lift.clamp()), 3_in);
    base.moveTo(10.25_ft, 3_ft);

    // slightly raise lift to prevent dragging
    lift.setSubposition(Subposition::high);
    base.limitSpeed(40);
    
    // back up to align with where neutral mogo was, go to opposite side platform
    base.moveTo(9_ft, 3_ft);
    base.turnTo(90_deg);
    base.moveTo(9_ft, 9_ft);
    base.turnTo(0_deg);
    base.moveTo(6_ft, 9_ft);

    // put second mogo on platform
    lift.raise();
    // give lift time to raise before turning
    pros::delay(1000);

    // provide stability
    base.limitSpeed(20);

    // drive into platform
    base.turnTo(90_deg);
    base.supply(20, 0);
    pros::delay(200);
    lift.setSubposition(Subposition::low);
    pros::delay(200);
    lift.release();
    pros::delay(200);

    // move back from platform, raise lift over edge of platform
    base.limitSpeed(40);
    base.addAction(bundle(lift.setSubposition(Subposition::neutral)), 4_in); // if lift does not lift before getting stuck on platform, increase distance from target at which lift raises
    base.moveTo(6_ft, 9_ft);

    // turn holder towards alliance mogo
    base.turnTo(0_deg);
    lift.lower();
    pros::delay(500);

    /**
    * Additional holder grab, start of comment section out if this is too much
    */

    // get alliance mogo in holder
    base.limitSpeed(20);
    base.moveTo(1.5_ft, 9_ft);
    holder.grab();

    // wait to allow holder to get a good grab
    pros::delay(500);

    base.limitSpeed(60);
    
    /**
    * Additional holder grab, end of comment section out if this is too much
    */

    // align with far left neutral mogo
    base.moveTo(3_ft, 9_ft);
    base.turnTo(270_deg);

    // grab neutral mogo after slightly pushing it
    base.limitSpeed(40);
    base.addAction(bundle(lift.clamp()), 5_in);
    base.moveTo(3_ft, 5.75_ft);

    lift.setSubposition(Subposition::high);
    base.limitSpeed(60);

    // drive into corner, avoid alliance mogo that was holding down the (now balanced) platform
    base.moveTo(1_ft, 1_ft);

}