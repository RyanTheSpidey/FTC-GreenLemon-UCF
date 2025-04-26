package pedroPathing.teleop;

import static pedroPathing.constants.Constants_ITD.elevatorBasket;
import static pedroPathing.constants.Constants_ITD.elevatorHighChamber;
import static pedroPathing.constants.Constants_ITD.elevatorLowBasket;
import static pedroPathing.constants.Constants_ITD.elevatorPower;
import static pedroPathing.constants.Constants_ITD.elevatorRiggingAboveHigh;
import static pedroPathing.constants.Constants_ITD.elevatorRiggingAboveLow;
import static pedroPathing.constants.Constants_ITD.elevatorRiggingExchange;
import static pedroPathing.constants.Constants_ITD.elevatorRiggingFinal;
import static pedroPathing.constants.Constants_ITD.elevatorTransfer;
import static pedroPathing.constants.Constants_ITD.elevatorWallSpecimen;
import static pedroPathing.constants.Constants_ITD.extendoActive;
import static pedroPathing.constants.Constants_ITD.extendoIncrement;
import static pedroPathing.constants.Constants_ITD.extendoMax;
import static pedroPathing.constants.Constants_ITD.extendoMin;
import static pedroPathing.constants.Constants_ITD.extendoMultiplier;
import static pedroPathing.constants.Constants_ITD.extendoPower;
import static pedroPathing.constants.Constants_ITD.extendoRetracted;
import static pedroPathing.constants.Constants_ITD.extendoTransfer;
import static pedroPathing.constants.Constants_ITD.extendoTransferAmount;
import static pedroPathing.constants.Constants_ITD.intakeElbowActive;
import static pedroPathing.constants.Constants_ITD.intakeElbowHover;
import static pedroPathing.constants.Constants_ITD.intakeElbowNeutral;
import static pedroPathing.constants.Constants_ITD.intakeElbowTransfer;
import static pedroPathing.constants.Constants_ITD.intakeFlexorActive;
import static pedroPathing.constants.Constants_ITD.intakeFlexorChamber;
import static pedroPathing.constants.Constants_ITD.intakeFlexorNeutral;
import static pedroPathing.constants.Constants_ITD.intakeFlexorTransfer;
import static pedroPathing.constants.Constants_ITD.intakeGrabberClosed;
import static pedroPathing.constants.Constants_ITD.intakeGrabberOpen;
import static pedroPathing.constants.Constants_ITD.intakePronatorLeft;
import static pedroPathing.constants.Constants_ITD.intakePronatorNeutral;
import static pedroPathing.constants.Constants_ITD.kickstandActive;
import static pedroPathing.constants.Constants_ITD.kickstandRetracted;
import static pedroPathing.constants.Constants_ITD.kickstandStabilize;
import static pedroPathing.constants.Constants_ITD.outtakeElbowBacksideChamber;
import static pedroPathing.constants.Constants_ITD.outtakeElbowBasket;
import static pedroPathing.constants.Constants_ITD.outtakeElbowHighChamber;
import static pedroPathing.constants.Constants_ITD.outtakeElbowNeutral;
import static pedroPathing.constants.Constants_ITD.outtakeElbowTransfer;
import static pedroPathing.constants.Constants_ITD.outtakeElbowWallSpecimen;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorBasket;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorHighChamber;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorNeutral;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorTransfer;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorWallSpecimen;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberLeftClosed;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberLeftOpen;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberRightClosed;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberRightOpen;
import static pedroPathing.constants.Constants_ITD.outtakePronatorBasket;
import static pedroPathing.constants.Constants_ITD.outtakePronatorHighChamber;
import static pedroPathing.constants.Constants_ITD.outtakePronatorTransfer;
import static pedroPathing.constants.Constants_ITD.outtakePronatorWallSpecimen;
import static pedroPathing.constants.Constants_ITD.swapModeDelay;
import static pedroPathing.constants.Constants_ITD.triggerThreshold;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * TODO: change passthrough to extendo -> observation zone, press to go to a neutral position, press again to fully extend
 */
@TeleOp(name = "TeleOp WIP")
public class TeleOpGen2WIP extends OpMode {
    List<LynxModule> allHubs;

    //  Declare motors
    private DcMotorEx extendo;
    private DcMotorEx elevatorLeft, elevatorRight;
    private DcMotorEx kickstand;

    //  Declare servos
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow;
    private Servo outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;

    //  State Machines
    private enum OuttakeState{
        NEUTRAL, TRANSFER, WALLSPECIMEN, CHAMBER, BASKET
    }
    private OuttakeState outtakeState = OuttakeState.NEUTRAL;

    private enum ElevatorState{
        RETRACTED, TRANSFER, WALLSPECIMEN, CHAMBER, BASKET, LOWBASKET
    }
    private ElevatorState elevatorState = ElevatorState.TRANSFER;

    private enum IntakeState{
        RETRACTED, ACTIVE, TRANSFER, PASSTHROUGH
    }
    private IntakeState intakeState = IntakeState.RETRACTED;

    private enum ScoringState{
        SAMPLE, SPECIMEN, RIGGING
    }
    private ScoringState scoringState = ScoringState.SAMPLE;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        //  Motor Initialization
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");
        kickstand = hardwareMap.get(DcMotorEx.class, "Motor Kickstand");

        //  Motor Reversals
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        //  Zero Power Behavior
//        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickstand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  STOP AND RESET - TODO: DELETE THIS ONCE AUTONOMOUS IS WRITTEN
//        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        kickstand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //  Servo Initialization
        intakeGrabber = hardwareMap.get(Servo.class, "Servo Intake Grabber");
        intakePronator = hardwareMap.get(Servo.class, "Servo Intake Pronator");
        intakeFlexor = hardwareMap.get(Servo.class, "Servo Intake Flexor");
        intakeElbow = hardwareMap.get(Servo.class, "Servo Intake Elbow");

        outtakePronator = hardwareMap.get(Servo.class, "Servo Outtake Pronator");
        outtakeFlexor = hardwareMap.get(Servo.class, "Servo Outtake Flexor");
        outtakeElbowLeft = hardwareMap.get(Servo.class, "Servo Outtake Elbow L");
        outtakeElbowRight = hardwareMap.get(Servo.class, "Servo Outtake Elbow R");
        outtakeGrabberLeft = hardwareMap.get(Servo.class, "Servo Outtake Grabber L");
        outtakeGrabberRight = hardwareMap.get(Servo.class, "Servo Outtake Grabber R");

        //  Servo Reversals

        //  Lynx Module
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    //  Gamepad 1 debounce variables
    private boolean g1UpPrev    = false;
    private boolean g1DownPrev  = false;
    private boolean g1LeftPrev  = false;
    private boolean g1RightPrev = false;
    private boolean g1YPrev    = false;
    private boolean g1APrev  = false;
    private boolean g1XPrev  = false;
    private boolean g1BPrev = false;
    private boolean g1TouchpadPrev    = false;
    private boolean g1LeftStickButtonPrev  = false;
    private boolean g1RightStickButtonPrev  = false;
    private boolean g1LeftBumperPrev = false;
    private boolean g1RightBumperPrev    = false;
    private boolean g1LeftTriggerPrev  = false;
    private boolean g1RightTriggerPrev  = false;

    //  Gamepad 2 debounce variables
    private boolean g2UpPrev    = false;
    private boolean g2DownPrev  = false;
    private boolean g2LeftPrev  = false;
    private boolean g2RightPrev = false;
    private boolean g2YPrev    = false;
    private boolean g2APrev  = false;
    private boolean g2XPrev  = false;
    private boolean g2BPrev = false;
    private boolean g2TouchpadPrev    = false;
    private boolean g2LeftStickButtonPrev  = false;
    private boolean g2RightStickButtonPrev  = false;
    private boolean g2LeftBumperPrev = false;
    private boolean g2RightBumperPrev    = false;
    private boolean g2LeftTriggerPrev  = false;
    private boolean g2RightTriggerPrev  = false;

    //  Variables
    private double turnPower = 0.7;
    private double speedLimit = 0.7;

    private boolean pronatorAngleToggle = false; // true = 90 degrees, false = normal

    private boolean exchanging = false;
    private double exchangeTimeStamp = 0;
    private boolean grabbing = false;
    private double grabTimeStamp = 0;
    private boolean swappingMode = false;
    private double swapModeTimeStamp = 0;

    private void updateDebounce(){
        //  Gamepad 1 debounce update
        g1UpPrev                = gamepad1.dpad_up;
        g1DownPrev              = gamepad1.dpad_down;
        g1LeftPrev              = gamepad1.dpad_left;
        g1RightPrev             = gamepad1.dpad_right;
        g1YPrev                 = gamepad1.y;
        g1APrev                 = gamepad1.a;
        g1XPrev                 = gamepad1.x;
        g1BPrev                 = gamepad1.b;
        g1TouchpadPrev          = gamepad1.touchpad;
        g1LeftStickButtonPrev   = gamepad1.left_stick_button;
        g1RightStickButtonPrev  = gamepad1.right_stick_button;
        g1LeftBumperPrev        = gamepad1.left_bumper;
        g1RightBumperPrev       = gamepad1.right_bumper;
        g1LeftTriggerPrev       = gamepad1.left_trigger > triggerThreshold;
        g1RightTriggerPrev      = gamepad1.right_trigger > triggerThreshold;

        //  Gamepad 2 debounce update
        g2UpPrev                = gamepad2.dpad_up;
        g2DownPrev              = gamepad2.dpad_down;
        g2LeftPrev              = gamepad2.dpad_left;
        g2RightPrev             = gamepad2.dpad_right;
        g2YPrev                 = gamepad2.y;
        g2APrev                 = gamepad2.a;
        g2XPrev                 = gamepad2.x;
        g2BPrev                 = gamepad2.b;
        g2TouchpadPrev          = gamepad2.touchpad;
        g2LeftStickButtonPrev   = gamepad2.left_stick_button;
        g2RightStickButtonPrev  = gamepad2.right_stick_button;
        g2LeftBumperPrev        = gamepad2.left_bumper;
        g2RightBumperPrev       = gamepad2.right_bumper;
        g2LeftTriggerPrev       = gamepad2.left_trigger > triggerThreshold;
        g2RightTriggerPrev      = gamepad2.right_trigger > triggerThreshold;
    }

    private void setElevator(int position){
        elevatorLeft.setTargetPosition(position);
        elevatorRight.setTargetPosition(position);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (scoringState == ScoringState.RIGGING){
            if (position < elevatorLeft.getCurrentPosition()){
                elevatorLeft.setPower(elevatorPower * 0.2);
                elevatorRight.setPower(elevatorPower * 0.2);
            } else {
                elevatorLeft.setPower(elevatorPower);
                elevatorRight.setPower(elevatorPower);
            }
        } else {
            if (position < elevatorLeft.getCurrentPosition()){
                elevatorLeft.setPower(elevatorPower * 0.6);
                elevatorRight.setPower(elevatorPower * 0.6);
            } else {
                elevatorLeft.setPower(elevatorPower);
                elevatorRight.setPower(elevatorPower);
            }
        }
    }

    private void setElevatorRigging(int position, double power){
        elevatorLeft.setTargetPosition(position);
        elevatorRight.setTargetPosition(position);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(power);
        elevatorRight.setPower(power);
    }

    private void setOuttakeElbow(double position){
        outtakeElbowLeft.setPosition(position);
        outtakeElbowRight.setPosition(position);
    }

    private void updateOuttake(){
        if (scoringState != ScoringState.RIGGING) {
            switch (outtakeState) {
                case NEUTRAL:
                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);
                    if (elevatorState == ElevatorState.BASKET) {
                        outtakePronator.setPosition(outtakePronatorBasket);
                    } else {
                        outtakePronator.setPosition(outtakePronatorTransfer);
                    }
                    break;
                case TRANSFER:
                    setOuttakeElbow(outtakeElbowTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                    break;
                case WALLSPECIMEN:
                    setOuttakeElbow(outtakeElbowWallSpecimen);
                    outtakeFlexor.setPosition(outtakeFlexorWallSpecimen);
                    outtakePronator.setPosition(outtakePronatorWallSpecimen);
                    break;
                case CHAMBER:
                    setOuttakeElbow(outtakeElbowHighChamber);
                    outtakeFlexor.setPosition(outtakeFlexorHighChamber);
                    outtakePronator.setPosition(outtakePronatorHighChamber);
                    break;
                case BASKET:
                    setOuttakeElbow(outtakeElbowBasket);
                    outtakeFlexor.setPosition(outtakeFlexorBasket);
                    outtakePronator.setPosition(outtakePronatorBasket);
                    break;
            }
        }
    }

    private void updateIntake(){
        switch (intakeState){
            case PASSTHROUGH:
                intakeElbow.setPosition(intakeElbowHover);
                intakeFlexor.setPosition(intakeFlexorChamber);
                break;
            case RETRACTED:
                setExtendo(extendoRetracted);
                intakeElbow.setPosition(intakeElbowNeutral);
                intakeFlexor.setPosition(intakeFlexorNeutral);
                pronatorAngleToggle = false;
//                intakePronator.setPosition(intakePronatorNeutral);
                break;
            case TRANSFER:
                intakeElbow.setPosition(intakeElbowTransfer);
                intakeFlexor.setPosition(intakeFlexorTransfer);
                pronatorAngleToggle = false;
//                intakePronator.setPosition(intakePronatorNeutral);
                break;
            case ACTIVE:
                if (!grabbing) {
                    intakeElbow.setPosition(intakeElbowHover);
                } else {
                    intakeElbow.setPosition(intakeElbowActive);
                }
                intakeFlexor.setPosition(intakeFlexorActive);
                break;
        }
    }

    private void updateElevator(){
        if (scoringState != ScoringState.RIGGING) {
            switch (elevatorState) {
                case RETRACTED:
                    setElevator(0);
                    break;
                case TRANSFER:
                    setElevator(elevatorTransfer);
                    break;
                case WALLSPECIMEN:
                    setElevator(elevatorWallSpecimen);
                    break;
                case CHAMBER:
                    setElevator(elevatorHighChamber);
                    break;
                case BASKET:
                    setElevator(elevatorBasket);
                    break;
                case LOWBASKET:
                    setElevator(elevatorLowBasket);
            }
        }
    }

    private void drive() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * speedLimit, -gamepad1.left_stick_x * speedLimit * 1.414, -gamepad1.right_stick_x * turnPower, true);
        follower.update();
    }

    private void setExtendo(int position){
        //TODO: check if this works
        //  suicide prevention
        if (extendo.getCurrent(CurrentUnit.AMPS) < 2.5){
//            extendo.setTargetPosition(Range.clip(position, (int) (250 * getRuntime() / 120), extendoMax));
            extendo.setTargetPosition(position);
            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendo.setPower(extendoPower);
        } else {
            extendo.setPower(0);
        }

    }

    public void controlIntake(boolean activate, boolean transfer, boolean open, boolean close, boolean extend, boolean retract, boolean exchange, boolean togglePronator, boolean passthrough){
        if (scoringState == ScoringState.SAMPLE) {
            if (activate) {
                elevatorState = ElevatorState.TRANSFER;
                outtakeState = OuttakeState.NEUTRAL;
                intakeState = IntakeState.ACTIVE;
                pronatorAngleToggle = false;
                intakeGrabber.setPosition(intakeGrabberOpen);
                setExtendo(extendoActive);

            }
            if (transfer) {
                if (elevatorState == ElevatorState.BASKET || elevatorState == ElevatorState.LOWBASKET){
                    elevatorState = ElevatorState.TRANSFER;
                    outtakeState = OuttakeState.NEUTRAL;
                    intakeState = IntakeState.RETRACTED;
                    setExtendo(extendoRetracted);
                } else {
                    elevatorState = ElevatorState.TRANSFER;
                    outtakeState = OuttakeState.TRANSFER;
                    intakeState = IntakeState.TRANSFER;
                    setExtendo(extendoTransfer);
                }

            }
            if (togglePronator) {
                pronatorAngleToggle = !pronatorAngleToggle;
            }
            intakePronator.setPosition(pronatorAngleToggle ? intakePronatorLeft : intakePronatorNeutral);
            if (open) {
                intakeGrabber.setPosition(intakeGrabberOpen);
            }
            if (close && !grabbing) {
                intakeGrabber.setPosition(intakeGrabberOpen);
                grabTimeStamp = getRuntime();
                grabbing = true;
            }
            if (getRuntime() - grabTimeStamp > 0.2 && grabbing){
                intakeGrabber.setPosition(intakeGrabberClosed);
            }
            if (getRuntime() - grabTimeStamp > 0.3 && grabbing){
                grabbing = false;
            }
            if (extend) {
                setExtendo(Range.clip(extendo.getCurrentPosition() + extendoIncrement * 2, extendoMin, extendoMax));
            }
            if (retract) {
                setExtendo(Range.clip(extendo.getCurrentPosition() - extendoIncrement, extendoMin, extendoMax));
            }
            if (exchange && !exchanging) {
                exchangeTimeStamp = getRuntime();
                setExtendo(extendoTransfer - extendoTransferAmount);
                setOuttakeGrabber(true);
                exchanging = true;
            }
            if (getRuntime() - exchangeTimeStamp > 0.35 && exchanging && intakeState == IntakeState.TRANSFER) { //0.4
                setOuttakeGrabber(false);
                intakeGrabber.setPosition(intakeGrabberOpen);
            }
            if (getRuntime() - exchangeTimeStamp > 0.45 && exchanging && intakeState == IntakeState.TRANSFER) { //0.45
                setExtendo(extendoTransfer);
            }
            if (getRuntime() - exchangeTimeStamp > 0.45 && extendo.getCurrentPosition() >= extendoTransfer - 80 && exchanging && intakeState == IntakeState.TRANSFER) {
                exchanging = false;
                outtakeState = OuttakeState.NEUTRAL;
            }

            if (passthrough){
                intakeState = IntakeState.PASSTHROUGH;
                outtakeState = OuttakeState.NEUTRAL;
                pronatorAngleToggle = false;
                if (extendo.getTargetPosition() != extendoRetracted){
                    setExtendo(extendoRetracted);
                } else if (extendo.getTargetPosition() == extendoRetracted){
                    //TODO: TUNE POSITION TO REMAIN WITHIN 42 INCHES
                    setExtendo((int) (extendoMax-200 * extendoMultiplier));
                }
            }
        }
    }
    //  TODO: program passthrough

    private void controlOuttake(boolean activate, boolean retract, boolean open, boolean close){
        switch (scoringState){
            case SAMPLE:
                if (activate && intakeState != IntakeState.ACTIVE){
                    if (elevatorState != ElevatorState.BASKET) {
                        elevatorState = ElevatorState.BASKET;
                    } else {
                        elevatorState = ElevatorState.LOWBASKET;
                    }
                }
                if (retract) {
                    elevatorState = ElevatorState.TRANSFER;
                    outtakeState = OuttakeState.NEUTRAL;
                    setOuttakeGrabber(true);
                    intakeState = IntakeState.RETRACTED;
                    setExtendo(extendoRetracted);
                }
                if (elevatorLeft.getCurrentPosition() > elevatorTransfer + 400 && elevatorState == ElevatorState.BASKET){
                    intakeState = IntakeState.RETRACTED;
                }
                if (elevatorLeft.getCurrentPosition() > elevatorBasket - 50 && elevatorState == ElevatorState.BASKET){
                    outtakeState = OuttakeState.BASKET;
                }
                break;
            case SPECIMEN:
                if (activate) {
                    elevatorState = ElevatorState.CHAMBER;
                    outtakeState = OuttakeState.CHAMBER;
                    intakeState = IntakeState.RETRACTED;
                }
                if (retract) {
                    elevatorState = ElevatorState.WALLSPECIMEN;
                    outtakeState = OuttakeState.WALLSPECIMEN;
                    setOuttakeGrabber(true);
                }
                break;
        }

        if (open) {
            setOuttakeGrabber(true);
        }

        if (close) {
            setOuttakeGrabber(false);
        }
    }

    private void setOuttakeGrabber(boolean open){
        if (open) {
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftOpen);
            outtakeGrabberRight.setPosition(outtakeGrabberRightOpen);
        } else{
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftClosed);
            outtakeGrabberRight.setPosition(outtakeGrabberRightClosed);
        }
    }

    private void controlScoringState(boolean sample, boolean specimen, boolean rigging){
        if (sample) {
            scoringState = ScoringState.SAMPLE;
            outtakeState = OuttakeState.NEUTRAL;
            elevatorState = ElevatorState.TRANSFER;
        } else if (specimen) {
            if (!swappingMode) {
                swappingMode = true;
                swapModeTimeStamp = getRuntime();
            }

            scoringState = ScoringState.SPECIMEN;
            outtakeState = OuttakeState.WALLSPECIMEN;
            elevatorState = ElevatorState.WALLSPECIMEN;
        } else if (rigging) {
            scoringState = ScoringState.RIGGING;
            setElevator(0);
            outtakeState = OuttakeState.NEUTRAL;
            setOuttakeElbow(outtakeElbowNeutral);
            setOuttakeGrabber(false);
            outtakeFlexor.setPosition(outtakeFlexorNeutral);
            intakeState = IntakeState.RETRACTED;
        }
        if (swappingMode && getRuntime() - swapModeTimeStamp > swapModeDelay) {
            intakeState = IntakeState.RETRACTED;
            swappingMode = false;
        }
    }

    int rigState = 0;

    private void controlRigging(boolean up, boolean down, boolean activateKickstand, boolean retractKickstand, boolean armUp, boolean armDown){
        if (scoringState == ScoringState.RIGGING) {
            if (up) {
                switch (rigState){
                    case 0:
                        rigState = 1;
                        setElevatorRigging(elevatorRiggingAboveLow, 1);
                        break;
                    case 1:
                        rigState = 2;
                        setElevatorRigging(elevatorRiggingExchange, 1);
                        break;
                    case 2:
                        rigState = 3;
                        setElevatorRigging(elevatorRiggingAboveLow, 0.1);
                        break;
                    case 3:
                        rigState = 4;
                        setElevatorRigging(elevatorRiggingAboveHigh, 1);
                        break;
                    case 4:
                        rigState = 5;
                        setElevatorRigging(elevatorRiggingAboveLow, 1);
                        setOuttakeElbow(outtakeElbowBacksideChamber);
                        break;
                    case 5:
                        rigState = 6;
                        setElevatorRigging(elevatorRiggingExchange, 1);
                        break;
                    case 6:
                        rigState = 7;
                        setElevatorRigging(elevatorRiggingFinal, 0.1);
                        break;
                }
            }
            if (down) {
                setElevator(Range.clip(elevatorLeft.getTargetPosition() - 300, 0, elevatorBasket));
            }
            if (activateKickstand) {
//                setKickstand(kickstandActive);
            }
            if (retractKickstand) {
//                setKickstand(kickstandRetracted);
            }
            if (armUp || armDown){
//                setKickstand(kickstandStabilize);
            }
        } else {
            setKickstand(0);
        }
    }

    private void setKickstand(int position){
        kickstand.setTargetPosition(position);
        kickstand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (elevatorLeft.getTargetPosition() == elevatorRiggingExchange){
            kickstand.setPower(1);
        } else {
            kickstand.setPower(position < kickstand.getCurrentPosition() ? 0.1 : 1);
        }
    }

    private void resetExtendo(boolean keybind){
//        if (keybind) {
//            if (intakeState == IntakeState.RETRACTED || intakeState == IntakeState.PASSTHROUGH){
//                extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            }
//        }

        //TODO: check if this works
        if (extendo.getTargetPosition() == extendoRetracted && extendo.getCurrentPosition() < 80 && extendo.getCurrent(CurrentUnit.AMPS) > 2.5){
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void controlSpeed(){
        switch(intakeState){
            case ACTIVE:
                speedLimit = 0.5;
                turnPower = 0.3;
                break;
            case TRANSFER:
                speedLimit = 1;
                turnPower = 0.4;
                break;
            case PASSTHROUGH:
                speedLimit = 1;
                turnPower = 1;
                break;
            case RETRACTED:
                switch (scoringState){
                    case SPECIMEN:
                        speedLimit = 1;
                        turnPower = 0.3;
                        break;
                    case SAMPLE:
                        speedLimit = 0.6;
                        turnPower = 0.5;
                        break;
                    case RIGGING:
                        speedLimit = 1;
                        turnPower = 0.7;
                        break;
                }
                break;
        }
    }

    @Override
    public void start() {
        setElevator(elevatorTransfer);
        setOuttakeElbow(outtakeElbowNeutral + 0.01);
        setOuttakeElbow(outtakeElbowNeutral);
        outtakeFlexor.setPosition(outtakeFlexorNeutral);
        outtakePronator.setPosition(outtakePronatorTransfer);
        setOuttakeGrabber(true);
        setExtendo(extendoRetracted);
        intakeElbow.setPosition(intakeElbowNeutral);
        intakeFlexor.setPosition(intakeFlexorNeutral);
        intakePronator.setPosition(intakePronatorNeutral);
        intakeGrabber.setPosition(intakeGrabberOpen);

        follower.startTeleopDrive();
        resetRuntime();
    }

    @Override
    public void loop() {
        if (getRuntime() > 60 && getRuntime() < 60.5){
            gamepad1.rumble(500);
        }
        drive();
        controlScoringState(gamepad1.left_bumper && !g1LeftBumperPrev && scoringState != ScoringState.SAMPLE,
                            gamepad1.right_bumper && !g1RightBumperPrev && scoringState == ScoringState.SAMPLE,
                            gamepad1.left_bumper && gamepad1.right_bumper && scoringState != ScoringState.RIGGING);
        controlIntake(  gamepad1.y && !g1YPrev,
                        gamepad1.a && !g1APrev,
                        gamepad1.left_trigger > 0.3 && !g1LeftTriggerPrev,
                        gamepad1.right_trigger > 0.3 && !g1RightTriggerPrev,
                        gamepad1.dpad_right && !g1RightPrev,
                        gamepad1.dpad_left && !g1LeftPrev,
                        gamepad1.touchpad && !g1TouchpadPrev,
                        gamepad1.x && !g1XPrev,
                        gamepad1.b && !g1BPrev);
//        controlOuttake( gamepad1.dpad_up && !g1UpPrev,
//                        gamepad1.dpad_down && !g1DownPrev,
//                        gamepad1.left_trigger > 0.8 && !g1LeftTriggerPrev,
//                        gamepad1.right_trigger > 0.8 && !g1RightTriggerPrev);
        controlOuttake(         scoringState == ScoringState.SPECIMEN ? gamepad1.y && !g1YPrev : gamepad1.dpad_up && !g1UpPrev,
                                scoringState == ScoringState.SPECIMEN ? gamepad1.a && !g1APrev : gamepad1.dpad_down && !g1DownPrev,
                        gamepad1.left_trigger > 0.3 && !g1LeftTriggerPrev,
                        gamepad1.right_trigger > 0.3 && !g1RightTriggerPrev);
        controlRigging( gamepad1.dpad_up && !g1UpPrev,
                        gamepad1.dpad_down && !g1DownPrev,
                        gamepad1.y && !g1YPrev,
                        gamepad1.a && !g1APrev,
                        gamepad1.dpad_right && !g1RightPrev,
                        gamepad1.dpad_left && !g1LeftPrev);
        controlSpeed();
        resetExtendo(gamepad1.right_stick_button && !g1RightStickButtonPrev);
        updateIntake();
        updateOuttake();
        updateElevator();
        updateDebounce();
//        telemetry.addLine("Elevator Left Current: " + elevatorLeft.getCurrent(CurrentUnit.AMPS));
//        telemetry.addLine("Elevator Right Current: " + elevatorRight.getCurrent(CurrentUnit.AMPS));
//        telemetry.addLine();
//        telemetry.addLine("Elevator Left Position: " + elevatorLeft.getCurrentPosition());
//        telemetry.addLine("Elevator Right Position: " + elevatorRight.getCurrentPosition());
//        telemetry.addLine();
//        telemetry.addLine("Extendo Current: " + extendo.getCurrent(CurrentUnit.AMPS));
//        telemetry.addLine("Extendo Position: " + extendo.getCurrentPosition());
//        telemetry.update();
    }


}
