#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// Controller and Motors
#define REVERSE_BUTTON pros::E_CONTROLLER_DIGITAL_RIGHT
bool inverted = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor groups
pros::MotorGroup leftMotors({-5, -3, -1}, pros::MotorGearset::blue); // Left motor group (ports 5, 3, 1)
pros::MotorGroup rightMotors({6, 4, 2}, pros::MotorGearset::blue);   // Right motor group (ports 6, 4, 2)

pros::Motor intakeLower(-8, pros::v5::MotorGears::green); // Intake motors
pros::Motor intakeUpper(7, pros::v5::MotorGears::green);

pros::MotorGroup lift({10, -9}, pros::v5::MotorGears::green); // Lift motors

// Pneumatics
pros::adi::DigitalOut grabL('H');
pros::adi::DigitalOut grabR('G');
pros::adi::DigitalOut intakeLift('F');
pros::adi::DigitalOut redirectToggle('E');

// Sensors
pros::Rotation liftRotation(13);
pros::Distance leftDistance(18);
pros::Distance rightDistance(19);
pros::Distance frontDistance(15);
pros::Distance backDistance(20);
pros::Distance sortDistance(16);
pros::Imu imu(12); // Inertial sensor

// Tracking wheels and odometry
pros::Rotation horizontalEnc(14);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.781);

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11, lemlib::Omniwheel::NEW_275, 450, 1);

// Motion controllers
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 20);

// Sensors for odometry
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// Chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

// Flags and constants
bool grabFlag = true, upperFlag = false, intakeFlag = true, clawFlag = true;
bool clawLiftFlag = true, liftDisabled = false, macroUP = false, macroDOWN = false, sort = false, redirectFlag = true;

double DISTANCE_OFFSET_L = 6;
double DISTANCE_OFFSET_R = 5.5;
double DISTANCE_OFFSET_F = 4;
double DISTANCE_OFFSET_B = 6.25;
double FIELD_SIZE_IN = 140;
double half_fieldSizeIN = FIELD_SIZE_IN / 2;

// Helper functions
double ConvertMM2IN(double MM) {
    return MM / 25.4;
}

/**
 * Initialization code
 */
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // Motor brake modes
    intakeLower.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    intakeUpper.set_brake_mode(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    lift.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);

    // Pneumatic initialization
    grabL.set_value(LOW);
    grabR.set_value(LOW);
    intakeLift.set_value(LOW);
    redirectToggle.set_value(LOW);

    // Task for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

/**
 * Disabled state
 */
void disabled() {}

/**
 * Code for competition initialization
 */
void competition_initialize() {}

/**
 * Autonomous routine
 */
void autonomous() {
    bool leftFlag = false, rightFlag = false, frontFlag = false, backFlag = false;

    // Conversion and offsets
    double leftDINActual = ConvertMM2IN(leftDistance.get_distance()) + DISTANCE_OFFSET_L;

    double rightHyp = ConvertMM2IN(rightDistance.get_distance());
    double rightDINActual = ((rightHyp*sin(1.51844))/sin(M_PI/2)) + DISTANCE_OFFSET_R;

    double frontHyp = ConvertMM2IN(frontDistance.get_distance());
    double frontDINActual = ((frontHyp*sin(1.31472))/sin(M_PI/2)) + DISTANCE_OFFSET_F;

    double backDINActual = ConvertMM2IN(backDistance.get_distance()) + DISTANCE_OFFSET_B;

    // Orientation logic
    if (leftDINActual < half_fieldSizeIN) leftFlag = true;
    if (rightDINActual < half_fieldSizeIN) rightFlag = true;
    if (frontDINActual < half_fieldSizeIN) frontFlag = false;       //Keep false until actually useful
    if (backDINActual < half_fieldSizeIN) backFlag = true;

    // Autonomous positioning
    if (leftFlag && frontFlag) {
        double x_offset = (-half_fieldSizeIN) + frontDINActual;
        double y_offset = (-half_fieldSizeIN) + leftDINActual;
        lemlib::Pose poseLF(x_offset, y_offset, 270);
        chassis.setPose(poseLF);

    }
    else if (rightFlag && frontFlag) {
        double x_offset = (-half_fieldSizeIN) + frontDINActual;
        double y_offset = half_fieldSizeIN - rightDINActual;
        lemlib::Pose poseRF(x_offset, y_offset, 270);
        chassis.setPose(poseRF);
    }
    else if (leftFlag && backFlag) {

        double x_offset = (-half_fieldSizeIN) + leftDINActual;
        double y_offset = (-half_fieldSizeIN) + backDINActual;
        lemlib::Pose poseLB(x_offset, y_offset, 0);
        chassis.setPose(poseLB);

        chassis.turnToHeading(340, 4000, {.maxSpeed = 50, .minSpeed = 30});
        chassis.waitUntilDone();
        chassis.moveToPose(-60, -13, 330, 4000, {.maxSpeed = 80, .minSpeed = 40});
        chassis.waitUntil(1);
        lift.move_relative(700, 200);
        redirectToggle.set_value(HIGH);
        chassis.waitUntilDone();
        chassis.moveToPose(-66, -2, 325, 4000, {.maxSpeed = 80, .minSpeed = 40});
        chassis.waitUntil(10);
        lift.move_relative(-550, 200);
        chassis.waitUntilDone();
        chassis.moveToPose(-48, -20, 315, 4000, {.forwards = false, .minSpeed = 100});
        chassis.waitUntil(1);
        lift.move_relative(-550, 200);
        chassis.waitUntilDone();
        chassis.turnToHeading(300, 4000, {.maxSpeed = 50, .minSpeed = 30});
        chassis.waitUntilDone();
        chassis.moveToPose(-30, -12, 260, 4000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50});
        chassis.waitUntil(1);
        lift.move_relative(700, 200);
        chassis.waitUntilDone();
        chassis.moveToPose(-14, -28, 330, 4000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50});
        chassis.waitUntilDone();
        chassis.turnToHeading(335, 4000, {.maxSpeed = 50, .minSpeed = 20});
        chassis.waitUntilDone();


        redirectToggle.set_value(LOW);

        /*
        chassis.turnToHeading(0, 4000, {.maxSpeed = 50, .minSpeed = 30});
        chassis.waitUntilDone();
        chassis.moveToPose(chassis.getPose().x, -22, 0, 4000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50});
        chassis.waitUntilDone();
        chassis.turnToHeading(270, 4000, {.maxSpeed = 50, .minSpeed = 30});
        chassis.waitUntilDone();
        chassis.moveToPose(-22,-24, 225, 4000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50});
        chassis.waitUntilDone();
        grabL.set_value(HIGH);
        grabR.set_value(HIGH);
        */
        
    }
    else if (rightFlag && backFlag) {
        double x_offset = (-half_fieldSizeIN) + rightDINActual;
        double y_offset = half_fieldSizeIN - backDINActual;
        lemlib::Pose poseRB(x_offset, y_offset, 180);
        chassis.setPose(poseRB);

        chassis.turnToHeading(200, 4000, {.maxSpeed = 50, .minSpeed = 30});
        chassis.waitUntilDone();
        chassis.moveToPose(-60, 13, 210, 4000, {.maxSpeed = 80, .minSpeed = 40});
        chassis.waitUntil(1);
        lift.move_relative(700, 200);
        redirectToggle.set_value(HIGH);
        chassis.waitUntilDone();
        chassis.moveToPose(-66, 2, 215, 4000, {.maxSpeed = 80, .minSpeed = 40});
        chassis.waitUntil(10);
        lift.move_relative(-550, 200);
        chassis.waitUntilDone();
        chassis.moveToPose(-48, 20, 225, 4000, {.forwards = false, .minSpeed = 100});
        chassis.waitUntil(1);
        lift.move_relative(-550, 200);
        chassis.waitUntilDone();
        chassis.turnToHeading(110, 4000, {.maxSpeed = 50, .minSpeed = 30});
        chassis.waitUntil(1);
        lift.move_relative(700, 200);
        chassis.waitUntilDone();
        chassis.moveToPose(-23, 7, 119, 4000, {.maxSpeed = 80, .minSpeed = 40});
        chassis.waitUntilDone();

        

        redirectToggle.set_value(LOW);

    }
}

/*  EXAMPLE CODE AND FUNCTIONS
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
*/

/**
 * Operator control
 */
void opcontrol() {
    pros::screen::set_pen(0x00FFFFFF);
    lemlib::Timer timePrint(20), timeThrowP1(170), timeThrowP2(400);
    timeThrowP1.pause();
    timeThrowP2.pause();

    // Controller loop
    while (true) {
        // Reverse button logic
        if (controller.get_digital_new_press(REVERSE_BUTTON)) {
            inverted = !inverted;
        }

        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        if (inverted) forward = -forward;

        chassis.arcade(forward, turn);

        // Intake control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !upperFlag) {
            intakeUpper.move_velocity(-200);
            intakeLower.move_velocity(-200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !upperFlag) {
            intakeUpper.move_velocity(200);
            intakeLower.move_velocity(200);
        } else if (!upperFlag) {
            intakeLower.move_velocity(0);
            intakeUpper.move_velocity(0);
        }

        // Lift control
        int liftStick = controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y);
        if (!liftDisabled && abs(liftStick) > 0) {
            lift.move_velocity(liftStick);
        } else if (!liftDisabled) {
            lift.brake();
        }

        // Grab and intake lift control
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            grabL.set_value(grabFlag ? HIGH : LOW);
            grabR.set_value(grabFlag ? HIGH : LOW);
            grabFlag = !grabFlag;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            intakeLift.set_value(intakeFlag ? HIGH : LOW);
            intakeFlag = !intakeFlag;
        }

        // Sorter and Macro functions...
        // (similar structure can be followed for other parts)

        //Lift Macros
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && !macroUP) {
            macroDOWN = true;
        }
        if (macroDOWN) {
            liftDisabled = true;
            redirectFlag = false;
            redirectToggle.set_value(LOW);
            lemlib::Timer time1(2000);
            if (lift.get_position() > 141 && !time1.isDone()) {
                lift.move_velocity(-180);
            }
            else {
                lift.brake();
                liftDisabled = false;
                macroDOWN = false;
            }
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) && !macroDOWN) {
            macroUP = true;
        }
        if (macroUP) {
            redirectFlag = true;
            redirectToggle.set_value(HIGH);
			liftDisabled = true;
            lemlib::Timer time2(2000);
            if (lift.get_position() < 875 && !time2.isDone()) {
                lift.move_velocity(200);
            }
            else {
                lift.brake();
                liftDisabled = false;
                macroUP = false;
            }
        }

        //Sorter
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
			!sort ? sort = true : sort = false;
		}

        if (ConvertMM2IN(sortDistance.get_distance()) < 5 && timeThrowP1.isPaused() && !timeThrowP1.isDone() && sort) {
            timeThrowP1.resume();
        }
        if (!timeThrowP1.isPaused() && !timeThrowP1.isDone()) {
            upperFlag = true;
        }
        else if (timeThrowP1.isDone() && timeThrowP2.isPaused() && !timeThrowP2.isDone()) {
            timeThrowP2.resume();
            intakeUpper.move_velocity(200);
        }
        if (timeThrowP2.isDone() && timeThrowP2.isDone() && !timeThrowP2.isPaused() && !timeThrowP1.isPaused()) {
            upperFlag = false;
            timeThrowP1.reset();
            timeThrowP1.pause();
            timeThrowP2.reset();
            timeThrowP2.pause();
        }

        //Redirect Toggle
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) 
        {
            redirectToggle.set_value(redirectFlag ? HIGH : LOW);
            redirectFlag = !redirectFlag;
        }

        if(timePrint.isDone() && !imu.is_calibrating()) {

            double leftDMM = leftDistance.get_distance();
            double leftDIN = (ConvertMM2IN(leftDMM));
            double leftDINActual = leftDIN + DISTANCE_OFFSET_L;

            double rightDMM = rightDistance.get_distance();
            double rightDIN = (ConvertMM2IN(rightDMM));
            double rightDINActual = rightDIN + DISTANCE_OFFSET_R;

            double frontDMM = frontDistance.get_distance();
            double frontDIN = (ConvertMM2IN(frontDMM));
            double frontDINActual = frontDIN + DISTANCE_OFFSET_F;

            double backDMM = backDistance.get_distance();
            double backDIN = (ConvertMM2IN(backDMM));
            double backDINActual = backDIN + DISTANCE_OFFSET_B;
            /*
            pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Left Distance: %f", leftDINActual);
            pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Confidence: %d", leftDistance.get_confidence());

            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Right Distance: %f", rightDINActual);
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Confidence: %d", rightDistance.get_confidence());

            pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Front Distance: %f", frontDINActual);
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Confidence: %d", frontDistance.get_confidence());

            pros::screen::print(pros::E_TEXT_MEDIUM, 9, "Back Distance: %f", backDINActual);
            pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Confidence: %d", backDistance.get_confidence());
            */

            //pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Rotation: %fl", lift.get_position(1));

            /*pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Saturation: %fl", ColorSensor.get_saturation());
            pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Hue: %fl", ColorSensor.get_hue());
            pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Bright: %fl", ColorSensor.get_brightness());
            pros::screen::print(pros::E_TEXT_MEDIUM, 7, "red: %fl", ColorSensor.get_rgb().red);
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "blu: %fl", ColorSensor.get_rgb().blue);
            pros::screen::print(pros::E_TEXT_MEDIUM, 9, "grn: %fl", ColorSensor.get_rgb().green);*/

            timePrint.reset();
        }
    }
}