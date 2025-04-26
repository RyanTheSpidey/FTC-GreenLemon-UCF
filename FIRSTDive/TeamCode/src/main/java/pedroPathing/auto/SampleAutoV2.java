package pedroPathing.auto;

import static pedroPathing.constants.AutoConstants.sampleBasketPose;
import static pedroPathing.constants.AutoConstants.sampleParkPose;
import static pedroPathing.constants.AutoConstants.samplePreloadControlPose;
import static pedroPathing.constants.AutoConstants.samplePreloadPose;
import static pedroPathing.constants.AutoConstants.sampleSpikeLeftPose;
import static pedroPathing.constants.AutoConstants.sampleSpikeMiddlePose;
import static pedroPathing.constants.AutoConstants.sampleSpikeRightPose;
import static pedroPathing.constants.AutoConstants.sampleStartPose;
import static pedroPathing.constants.Constants_ITD.elevatorBasket;
import static pedroPathing.constants.Constants_ITD.elevatorPower;
import static pedroPathing.constants.Constants_ITD.elevatorTransfer;
import static pedroPathing.constants.Constants_ITD.extendoPower;
import static pedroPathing.constants.Constants_ITD.extendoRetracted;
import static pedroPathing.constants.Constants_ITD.extendoTransfer;
import static pedroPathing.constants.Constants_ITD.extendoTransferAmount;
import static pedroPathing.constants.Constants_ITD.intakeElbowActive;
import static pedroPathing.constants.Constants_ITD.intakeElbowHover;
import static pedroPathing.constants.Constants_ITD.intakeElbowNeutral;
import static pedroPathing.constants.Constants_ITD.intakeElbowTransfer;
import static pedroPathing.constants.Constants_ITD.intakeFlexorActive;
import static pedroPathing.constants.Constants_ITD.intakeFlexorNeutral;
import static pedroPathing.constants.Constants_ITD.intakeFlexorTransfer;
import static pedroPathing.constants.Constants_ITD.intakeGrabberClosed;
import static pedroPathing.constants.Constants_ITD.intakeGrabberOpen;
import static pedroPathing.constants.Constants_ITD.intakePronatorNeutral;
import static pedroPathing.constants.Constants_ITD.outtakeElbowBasket;
import static pedroPathing.constants.Constants_ITD.outtakeElbowTransfer;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorBasket;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorTransfer;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberLeftClosed;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberLeftOpen;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberRightClosed;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberRightOpen;
import static pedroPathing.constants.Constants_ITD.outtakePronatorBasket;
import static pedroPathing.constants.Constants_ITD.outtakePronatorTransfer;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled

@Autonomous
public class SampleAutoV2 extends OpMode {
    private int actionState = 0;
    List<LynxModule> allHubs;
    private DcMotorEx extendo, elevatorLeft, elevatorRight;
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow, outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path scorePreload, park;
    private PathChain grabSampleRight, scoreSampleRight, grabSampleMiddle, scoreSampleMiddle, grabSampleLeft, scoreSampleLeft;

    public void buildPaths(){
        scorePreload = new Path(new BezierCurve(new Point(sampleStartPose), new Point(samplePreloadControlPose), new Point(samplePreloadPose)));
        scorePreload.setLinearHeadingInterpolation(sampleStartPose.getHeading(), samplePreloadPose.getHeading());

        grabSampleRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(samplePreloadPose), new Point(sampleSpikeRightPose)))
                .setLinearHeadingInterpolation(samplePreloadPose.getHeading(), sampleSpikeRightPose.getHeading())
                .build();

        scoreSampleRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleSpikeRightPose), new Point(sampleBasketPose)))
                .setLinearHeadingInterpolation(sampleSpikeRightPose.getHeading(), sampleBasketPose.getHeading())
                .build();

        grabSampleMiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleBasketPose), new Point(sampleSpikeMiddlePose)))
                .setLinearHeadingInterpolation(sampleBasketPose.getHeading(), sampleSpikeMiddlePose.getHeading())
                .build();

        scoreSampleMiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleSpikeMiddlePose), new Point(sampleBasketPose)))
                .setLinearHeadingInterpolation(sampleSpikeMiddlePose.getHeading(), sampleBasketPose.getHeading())
                .build();

        grabSampleLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleBasketPose), new Point(sampleSpikeLeftPose)))
                .setLinearHeadingInterpolation(sampleBasketPose.getHeading(), sampleSpikeLeftPose.getHeading())
                .build();

        scoreSampleLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleSpikeLeftPose), new Point(sampleBasketPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), sampleBasketPose.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(sampleBasketPose), new Point(sampleParkPose)));
        park.setLinearHeadingInterpolation(sampleBasketPose.getHeading(), sampleParkPose.getHeading(), 0.3);



    }

    private boolean actionTimerActive = false;

    public void turnTo(double degrees) { // if you want to turn right, use negative degrees
        Pose temp = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(degrees));
        follower.holdPoint(temp);
    }


    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                setElevator(elevatorBasket);
                follower.followPath(scorePreload, true);
                setPathState(1);
                actionState = 0;
                break;
            case 1:
                if (follower.getCurrentTValue() > 0.8){
                    follower.setMaxPower(0.3);
                } else if (follower.getCurrentTValue() > 0.5){
                    follower.setMaxPower(0.7);
                } else {
                    follower.setMaxPower(1);
                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {
                    if (elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 100){
                        setOuttakeElbow(outtakeElbowBasket);
                        outtakeFlexor.setPosition(outtakeFlexorBasket);
                        outtakePronator.setPosition(outtakePronatorBasket);
                        if (!actionTimerActive){
                            actionTimerActive = true;
                            actionTimer.resetTimer();
                        }
                    }
                    //  score preload + activate intake
                    if (actionTimer.getElapsedTimeSeconds() > 0.5){ //0.5
                        setOuttakeGrabber(true);
                        setExtendo(770);
                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        intakePronator.setPosition(intakePronatorNeutral);
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1){
                        setOuttakeElbow(outtakeElbowTransfer);

                        actionTimerActive = false;
                        follower.followPath(grabSampleRight, true);
                        setPathState(2);
                        actionState = 0;
                    }
                }
                break;
            case 2:
                follower.setMaxPower(1);
                //  outtake -> transfer
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && !actionTimerActive){
                    setElevator(elevatorTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    //  grab sample, transfer, raise elevator
                    if (actionTimer.getElapsedTimeSeconds() > 0.1 && actionState == 0){
                        intakeElbow.setPosition(intakeElbowActive);
                        actionState = 1;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.4 && actionState == 1){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                        actionState = 2;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.5 && actionState == 2) {
                        setExtendo(extendoTransfer);
                        intakeElbow.setPosition(intakeElbowTransfer);
                        intakeFlexor.setPosition(intakeFlexorTransfer);
                        actionState = 3;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.1 && actionState == 3){
                        setExtendo(extendoTransfer - extendoTransferAmount);
                        actionState = 4;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.5 && actionState == 4) {
                        setOuttakeGrabber(false);
                        actionState = 5;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.6 && actionState == 5) {
                        intakeGrabber.setPosition(intakeGrabberOpen);
                        setExtendo(extendoTransfer);
                        actionState = 6;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.7 && extendo.getCurrentPosition() > extendoTransfer - 50 && actionState == 6){
                        actionTimerActive = false;
                        setElevator(elevatorBasket);
                        follower.followPath(scoreSampleRight, true);
                        setPathState(3);
                        actionState = 0;
                    }
                }
                break;
            case 3:
//                if (elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 100){
//                    setOuttakeElbow(outtakeElbowBasket);
//                    outtakeFlexor.setPosition(outtakeFlexorBasket);
//                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7 && elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 100) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    setOuttakeElbow(outtakeElbowBasket);
                    outtakeFlexor.setPosition(outtakeFlexorBasket);
                    outtakePronator.setPosition(outtakePronatorBasket);
                    //  score preload + activate intake
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        setOuttakeGrabber(true);
                        setExtendo(770);
                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        intakePronator.setPosition(intakePronatorNeutral);
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        setOuttakeElbow(outtakeElbowTransfer);

                        actionTimerActive = false;
                        follower.followPath(grabSampleMiddle, true);
                        setPathState(4);
                        actionState = 0;

                    }
                }
                break;
            case 4:
                //  outtake -> transfer
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && !actionTimerActive){
                    setElevator(elevatorTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    //  grab sample, transfer, raise elevator
                    if (actionTimer.getElapsedTimeSeconds() > 0.1 && actionState == 0){
                        intakeElbow.setPosition(intakeElbowActive);
                        actionState = 1;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.4 && actionState == 1){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                        actionState = 2;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.5 && actionState == 2) {
                        setExtendo(extendoTransfer);
                        intakeElbow.setPosition(intakeElbowTransfer);
                        intakeFlexor.setPosition(intakeFlexorTransfer);
                        actionState = 3;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.1 && actionState == 3){
                        setExtendo(extendoTransfer - extendoTransferAmount);
                        actionState = 4;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.5 && actionState == 4) {
                        setOuttakeGrabber(false);
                        actionState = 5;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.6 && actionState == 5) {
                        intakeGrabber.setPosition(intakeGrabberOpen);
                        setExtendo(extendoTransfer);
                        actionState = 6;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.7 && extendo.getCurrentPosition() > extendoTransfer - 50 && actionState == 6){
                        actionTimerActive = false;
                        setElevator(elevatorBasket);
                        follower.followPath(scoreSampleMiddle, true);
                        setPathState(5);
                        actionState = 0;
                    }
                }
                break;
            case 5:
//                if (elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 100){
//                    setOuttakeElbow(outtakeElbowBasket);
//                    outtakeFlexor.setPosition(outtakeFlexorBasket);
//                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7 && elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 100) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    setOuttakeElbow(outtakeElbowBasket);
                    outtakeFlexor.setPosition(outtakeFlexorBasket);
                    outtakePronator.setPosition(outtakePronatorBasket);
                    //  score preload + activate intake
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        setOuttakeGrabber(true);
                        setExtendo(1080);
                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        intakePronator.setPosition(intakePronatorNeutral - 0.1);
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        setOuttakeElbow(outtakeElbowTransfer);

                        actionTimerActive = false;
                        follower.followPath(grabSampleLeft, true);
                        setPathState(6);
                        actionState = 0;

                    }
                }
                break;
            case 6:
                if (follower.getCurrentTValue() > 0.8){
                    follower.setMaxPower(0.3);
                } else if (follower.getCurrentTValue() > 0.5){
                    follower.setMaxPower(0.7);
                } else {
                    follower.setMaxPower(1);
                }
                //  outtake -> transfer
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && !actionTimerActive){
                    setElevator(elevatorTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    //  grab sample, transfer, raise elevator
                    if (actionTimer.getElapsedTimeSeconds() > 0.1 && actionState == 0){
                        intakeElbow.setPosition(intakeElbowActive);
                        actionState = 1;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.4 && actionState == 1){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                        actionState = 2;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.5 && actionState == 2) {
                        turnTo(180);
                        setExtendo(extendoTransfer);
                        intakeElbow.setPosition(intakeElbowTransfer);
                        intakeFlexor.setPosition(intakeFlexorTransfer);
                        intakePronator.setPosition(intakePronatorNeutral);
                        actionState = 3;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.1 & actionState == 3){
                        setExtendo(extendoTransfer - extendoTransferAmount);
                        actionState = 4;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.5 && actionState == 4) {
                        setOuttakeGrabber(false);
                        actionState = 5;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.6 && actionState == 5) {
                        intakeGrabber.setPosition(intakeGrabberOpen);
                        setExtendo(extendoTransfer);
                        actionState = 6;
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1.7 && extendo.getCurrentPosition() > extendoTransfer - 50 && actionState == 6){
                        actionTimerActive = false;
                        setElevator(elevatorBasket);
                        follower.followPath(scoreSampleLeft, true);
                        setPathState(7);
                        actionState = 0;
                    }
                }
                break;
            case 7:
                follower.setMaxPower(1);
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7 && elevatorLeft.getCurrentPosition() > elevatorLeft.getTargetPosition() - 100) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    setOuttakeElbow(outtakeElbowBasket);
                    outtakeFlexor.setPosition(outtakeFlexorBasket);
                    outtakePronator.setPosition(outtakePronatorBasket);
                    //  score preload + activate intake
                    if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                        setOuttakeGrabber(true);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 1) {
                        actionTimerActive = false;
                        follower.followPath(park, true);
                        setPathState(8);
                        actionState = 0;

                    }
                }
                break;
            case 8:
                //  0.5 seconds in, retract mechanisms
                if (pathTimer.getElapsedTimeSeconds() > 0.5){
                    setElevator(elevatorTransfer);
                    setOuttakeElbow(0.43);
                    setExtendo(extendoRetracted);
                    intakeElbow.setPosition(intakeElbowNeutral);
                    intakeFlexor.setPosition(intakeFlexorNeutral);
                }
                if (follower.atParametricEnd()){
                    setPathState(-1);
                    actionState = 0;
                }
                break;
                //  end of auto
        }
    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    private void setElevator(int position){
        elevatorLeft.setTargetPosition(position);
        elevatorRight.setTargetPosition(position);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setPower(elevatorPower);
        elevatorRight.setPower(elevatorPower);
    }

    private void setOuttakeElbow(double position){
        outtakeElbowLeft.setPosition(position);
        outtakeElbowRight.setPosition(position);
    }

    public void setOuttakeGrabber(boolean open){
        if (open) {
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftOpen);
            outtakeGrabberRight.setPosition(outtakeGrabberRightOpen);
        } else{
            outtakeGrabberLeft.setPosition(outtakeGrabberLeftClosed);
            outtakeGrabberRight.setPosition(outtakeGrabberRightClosed);
        }
    }

    private void setExtendo(int position){
        extendo.setTargetPosition(position);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(extendoPower);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(sampleStartPose);
        buildPaths();

        allHubs = hardwareMap.getAll(LynxModule.class);

        //  Motor Initialization
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");

        //  Motor Reversals
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        //  Zero Power Behavior
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  STOP AND RESET
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        //  TODO: initialize with the preload scoring in mind
        setElevator(0);
        setOuttakeElbow(0.26);
        outtakeFlexor.setPosition(0.81);
        outtakePronator.setPosition(outtakePronatorTransfer);
        setOuttakeGrabber(true);
        setExtendo(0);
        intakeElbow.setPosition(0.15);
        intakeFlexor.setPosition(0.34);
        intakePronator.setPosition(intakePronatorNeutral);
        intakeGrabber.setPosition(intakeGrabberClosed);

    }

    @Override
    public void init_loop() {
        if (gamepad1.left_bumper) {
            setOuttakeGrabber(true);
        }
        if (gamepad1.right_bumper){
            setOuttakeGrabber(false);
        }
        telemetry.addLine("Ensure the following:\nKickstand is all the way up\nExtendo is retracted\nElevator is retracted\nSample is preloaded\nGamepads are connected\n\tGood Luck!");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
