package pedroPathing.tools;

import static pedroPathing.constants.Constants_ITD.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Manual Servo Control Test")
public class AllServoTest extends OpMode {

    // Motors
    private DcMotorEx extendo, elevatorLeft, elevatorRight;

    // Servos
    private Servo intakeElbow, intakeFlexor, intakeGrabber, intakePronator;
    private Servo outtakeElbowLeft, outtakeElbowRight, outtakeFlexor, outtakeGrabberLeft, outtakeGrabberRight, outtakePronator;

    // Increment step
    private final double INCREMENT = 0.01;

    // Debounce variables
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight;
    private boolean prevA, prevB, prevY, prevX;
    private boolean prevLeftBumper, prevRightBumper;
    private double prevLeftTrigger, prevRightTrigger;
    private boolean closePrev = false;
    private boolean openPrev = false;

    @Override
    public void init() {
        // Initialize motors
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");


        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize servos
        intakeElbow = hardwareMap.get(Servo.class, "Servo Intake Elbow");
        intakeFlexor = hardwareMap.get(Servo.class, "Servo Intake Flexor");
        intakeGrabber = hardwareMap.get(Servo.class, "Servo Intake Grabber");
        intakePronator = hardwareMap.get(Servo.class, "Servo Intake Pronator");
        outtakeElbowLeft = hardwareMap.get(Servo.class, "Servo Outtake Elbow L");
        outtakeElbowRight = hardwareMap.get(Servo.class, "Servo Outtake Elbow R");
        outtakeFlexor = hardwareMap.get(Servo.class, "Servo Outtake Flexor");
        outtakeGrabberLeft = hardwareMap.get(Servo.class, "Servo Outtake Grabber L");
        outtakeGrabberRight = hardwareMap.get(Servo.class, "Servo Outtake Grabber R");
        outtakePronator = hardwareMap.get(Servo.class, "Servo Outtake Pronator");

        // Set initial servo positions (can be adjusted as needed)
        intakeElbow.setPosition(0.5);
        intakeFlexor.setPosition(0.5);
        intakeGrabber.setPosition(intakeGrabberClosed);
        intakePronator.setPosition(0.5);
        outtakeElbowLeft.setPosition(0.5);
        outtakeElbowRight.setPosition(0.5);
        outtakeFlexor.setPosition(0.5);
        outtakeGrabberLeft.setPosition(outtakeGrabberLeftOpen);
        outtakeGrabberRight.setPosition(outtakeGrabberRightOpen);
        outtakePronator.setPosition(outtakePronatorTransfer);


        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.3 && !openPrev){
            intakeGrabber.setPosition(intakeGrabber.getPosition() + 0.01);
        }
        if (gamepad1.right_trigger > 0.3 && !closePrev) {
            intakeGrabber.setPosition(intakeGrabber.getPosition() - 0.01);
        }
        openPrev = gamepad1.left_trigger > 0.3;
        closePrev = gamepad1.right_trigger > 0.3;
        if (gamepad1.left_stick_button){
            outtakePronator.setPosition(outtakePronatorBasket);
        }
        if (gamepad1.right_stick_button) {
            outtakePronator.setPosition(outtakePronatorHighChamber);
        }

        // Manually control the intake and outtake servos
        manualServoControl(gamepad1);

        // Update debounce states
        updateDebounceStates(gamepad1);

        // Telemetry for servo positions and control mappings
        telemetry.addLine("Motor positions:");
        telemetry.addData("Extendo", extendo.getCurrentPosition());
        telemetry.addData("Elevator Left", elevatorLeft.getCurrentPosition());
        telemetry.addData("Elevator Right", elevatorRight.getCurrentPosition());
        telemetry.addLine();
        telemetry.addLine("Servo Positions and Controls:");
        telemetry.addData("Intake Elbow", "dpad_up/down | Position: %.2f", intakeElbow.getPosition());
        telemetry.addData("Intake Flexor", "dpad_right/left | Position: %.2f", intakeFlexor.getPosition());
        telemetry.addData("Intake Grabber", "left_bumper/trigger | Position: %.2f", intakeGrabber.getPosition());
        telemetry.addData("Outtake Elbow Left/Right", "a/b | Left: %.2f, Right: %.2f", outtakeElbowLeft.getPosition(), outtakeElbowRight.getPosition());
        telemetry.addData("Outtake Flexor", "y/x | Position: %.2f", outtakeFlexor.getPosition());
        telemetry.addData("Outtake Grabber Left/Right", "right_bumper/trigger | Left: %.2f, Right: %.2f", outtakeGrabberLeft.getPosition(), outtakeGrabberRight.getPosition());
        telemetry.update();
    }

    private void manualServoControl(Gamepad gamepad) {
        if (gamepad.dpad_up && !prevDpadUp) {
            intakeElbow.setPosition(intakeElbow.getPosition() + INCREMENT);
        } else if (gamepad.dpad_down && !prevDpadDown) {
            intakeElbow.setPosition(intakeElbow.getPosition() - INCREMENT);
        }

        if (gamepad.dpad_right && !prevDpadRight) {
            intakeFlexor.setPosition(intakeFlexor.getPosition() + INCREMENT);
        } else if (gamepad.dpad_left && !prevDpadLeft) {
            intakeFlexor.setPosition(intakeFlexor.getPosition() - INCREMENT);
        }

        if (gamepad.a && !prevA) {
            outtakeElbowLeft.setPosition(outtakeElbowLeft.getPosition() + INCREMENT);
            outtakeElbowRight.setPosition(outtakeElbowRight.getPosition() + INCREMENT);
        } else if (gamepad.b && !prevB) {
            outtakeElbowLeft.setPosition(outtakeElbowLeft.getPosition() - INCREMENT);
            outtakeElbowRight.setPosition(outtakeElbowRight.getPosition() - INCREMENT);
        }

        if (gamepad.y && !prevY) {
            outtakeFlexor.setPosition(outtakeFlexor.getPosition() + INCREMENT);
        } else if (gamepad.x && !prevX) {
            outtakeFlexor.setPosition(outtakeFlexor.getPosition() - INCREMENT);
        }

        if (gamepad.right_bumper && !prevRightBumper) {
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftOpen);
            outtakeGrabberRight.setPosition(outtakeGrabberRightOpen);
        } else if (gamepad.right_trigger > 0.1 && prevRightTrigger <= 0.1) {
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftClosed);
            outtakeGrabberRight.setPosition(outtakeGrabberRightClosed);
        }
    }

    private void updateDebounceStates(Gamepad gamepad) {
        prevDpadUp = gamepad.dpad_up;
        prevDpadDown = gamepad.dpad_down;
        prevDpadLeft = gamepad.dpad_left;
        prevDpadRight = gamepad.dpad_right;
        prevA = gamepad.a;
        prevB = gamepad.b;
        prevY = gamepad.y;
        prevX = gamepad.x;
        prevLeftBumper = gamepad.left_bumper;
        prevRightBumper = gamepad.right_bumper;
        prevLeftTrigger = gamepad.left_trigger;
        prevRightTrigger = gamepad.right_trigger;
    }
}
