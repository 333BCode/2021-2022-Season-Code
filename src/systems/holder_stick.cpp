#include "systems/holder_stick.hpp"

namespace motor_control {

    namespace holder {

        constexpr long double highAngle = 1250;

        static long double offset = -2450;

        bool usingManualControl = false;

        enum class State {
            notDeployed,
            up,
            down
        };

        static State state = State::notDeployed;

        static bool stateUpdated = false;

        static pros::Mutex mutex {};

        void raise() {
        mutex.take(TIMEOUT_MAX);
            state = State::up;
            stateUpdated = true;
        mutex.give();
        }

        void lower() {
        mutex.take(TIMEOUT_MAX);
            if (state == State::notDeployed) {
                stick::setNeutral();
            }
            state = State::down;
        mutex.give();
        }

        void toggleHolder() {
        mutex.take(TIMEOUT_MAX);
            if (state == State::notDeployed) {
                stateUpdated = true;
            }
            if (state == State::down) {
                state = State::up;
                stateUpdated = true;
            } else {
                state = State::down;
            }
        mutex.give();
        }

        void setManualControl(bool manualControl) {
        mutex.take(TIMEOUT_MAX);
            stick::setNeutral();
            usingManualControl = manualControl;
            state = State::down;
        mutex.give();
        }

        void reset() {
        mutex.take(TIMEOUT_MAX);
            motor.tare_position();
            offset = 0;
        mutex.give();
        }

    } // namespace holder

    namespace stick {

        constexpr long double highAngle = 100;
        constexpr long double neutralAngle = 45;

        enum class State {
            down,
            neutral,
            up
        };

        static State state = State::down;

        static pros::Mutex mutex {};

        void recieve() {
        mutex.take(TIMEOUT_MAX);
            state = State::down;
        mutex.give();
        }

        void deposit() {
        mutex.take(TIMEOUT_MAX);
            state = State::up;
        mutex.give();
        }

        void toggleStick() {
        mutex.take(TIMEOUT_MAX);
            state = (state == State::down ? State::up : State::down);
        mutex.give();
        }

        void setNeutral() {
        mutex.take(TIMEOUT_MAX);
            state = State::neutral;
        mutex.give();
        }

    } // namespace stick

    void powerHolderAndStick() {

    holder::mutex.take(TIMEOUT_MAX);
    stick::mutex.take(TIMEOUT_MAX);

        if (!holder::usingManualControl) {

            if (holder::stateUpdated) {
                if (stick::state == stick::State::down) {
                    stick::state = stick::State::neutral;
                }
                holder::stateUpdated = false;
            }

            if (stick::state == stick::State::down && holder::state == holder::State::up) {
                holder::state = holder::State::down;
            }

            if (holder::state == holder::State::down) {
                holder::motor.move_absolute(holder::offset, 100);
            } else if (holder::state == holder::State::up) {
                holder::motor.move_absolute(holder::highAngle + holder::offset, 100);
            }

            switch (stick::state) {

                case stick::State::down:
                
                    if (stick::motor.get_position() > 5) {
                        stick::motor.move(-127);
                    }
                
                break;
                case stick::State::neutral:
                
                    stick::motor.move_absolute(stick::neutralAngle, 100);
                
                break;
                case stick::State::up:
                
                    if (stick::motor.get_position() < stick::highAngle - 5) {
                        stick::motor.move(127);
                    }
                
                break;

            }

        } else {

            stick::motor.move_absolute(stick::neutralAngle, 100);

        }

    stick::mutex.give();
    holder::mutex.give();

    }

} // namespace motor_control