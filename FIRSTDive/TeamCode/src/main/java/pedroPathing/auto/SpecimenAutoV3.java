package pedroPathing.auto;

import static pedroPathing.constants.AutoConstants.specimenChamber1Pose;
import static pedroPathing.constants.AutoConstants.specimenChamber2Pose;
import static pedroPathing.constants.AutoConstants.specimenChamber3Pose;
import static pedroPathing.constants.AutoConstants.specimenChamber4Pose;
import static pedroPathing.constants.AutoConstants.specimenObservationMiddlePose;
import static pedroPathing.constants.AutoConstants.specimenObservationParkPose;
import static pedroPathing.constants.AutoConstants.specimenObservationRightPose;
import static pedroPathing.constants.AutoConstants.specimenPreloadPose;
import static pedroPathing.constants.AutoConstants.specimenStartPose;
import static pedroPathing.constants.AutoConstants.specimenWallSpecimenControlPose;
import static pedroPathing.constants.AutoConstants.specimenWallSpecimenPose;
import static pedroPathing.constants.AutoConstants.specimenWallSpecimenSpline1Pose;
import static pedroPathing.constants.AutoConstants.specimenWallSpecimenSpline2Pose;
import static pedroPathing.constants.Constants_ITD.elevatorBacksideChamber;
import static pedroPathing.constants.Constants_ITD.elevatorHighChamber;
import static pedroPathing.constants.Constants_ITD.elevatorPower;
import static pedroPathing.constants.Constants_ITD.elevatorWallSpecimen;
import static pedroPathing.constants.Constants_ITD.extendoMin;
import static pedroPathing.constants.Constants_ITD.extendoPower;
import static pedroPathing.constants.Constants_ITD.extendoRetracted;
import static pedroPathing.constants.Constants_ITD.intakeElbowActive;
import static pedroPathing.constants.Constants_ITD.intakeElbowHover;
import static pedroPathing.constants.Constants_ITD.intakeElbowNeutral;
import static pedroPathing.constants.Constants_ITD.intakeElbowRetracted;
import static pedroPathing.constants.Constants_ITD.intakeFlexorActive;
import static pedroPathing.constants.Constants_ITD.intakeFlexorNeutral;
import static pedroPathing.constants.Constants_ITD.intakeFlexorRetracted;
import static pedroPathing.constants.Constants_ITD.intakeGrabberClosed;
import static pedroPathing.constants.Constants_ITD.intakeGrabberOpen;
import static pedroPathing.constants.Constants_ITD.intakePronatorNeutral;
import static pedroPathing.constants.Constants_ITD.outtakeElbowBacksideChamber;
import static pedroPathing.constants.Constants_ITD.outtakeElbowHighChamber;
import static pedroPathing.constants.Constants_ITD.outtakeElbowWallSpecimen;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorBacksideChamber;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorHighChamber;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorWallSpecimen;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberLeftClosed;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberLeftOpen;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberRightClosed;
import static pedroPathing.constants.Constants_ITD.outtakeGrabberRightOpen;
import static pedroPathing.constants.Constants_ITD.outtakePronatorBacksideChamber;
import static pedroPathing.constants.Constants_ITD.outtakePronatorHighChamber;
import static pedroPathing.constants.Constants_ITD.outtakePronatorNeutral;
import static pedroPathing.constants.Constants_ITD.outtakePronatorWallSpecimen;

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
public class SpecimenAutoV3 extends OpMode {
    List<LynxModule> allHubs;
    private DcMotorEx extendo, elevatorLeft, elevatorRight;
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow, outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path scorePreload, park;
    private PathChain grabSpikeMarkLeft, grabSpikeMarkMiddle, grabSpikeMarkRight, passSpikeMarkRight, grab1, score1, grab2, score2, grab3, score3, grab4, score4;

    private Pose spikeLeftPose = new Pose(120, 119, Math.toRadians(179.9));
    private Pose spikeMiddlePose = new Pose(120, 130, Math.toRadians(180));
    private Pose spikeRightPose = new Pose(116, 122, Math.toRadians(143));
    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(specimenStartPose), new Point(specimenPreloadPose)));
        scorePreload.setLinearHeadingInterpolation(specimenStartPose.getHeading(), specimenPreloadPose.getHeading());

        grabSpikeMarkLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimenPreloadPose), new Point(spikeLeftPose)))
                .setLinearHeadingInterpolation(specimenPreloadPose.getHeading(), spikeLeftPose.getHeading())
                .build();

        grabSpikeMarkMiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeLeftPose), new Point(spikeMiddlePose)))
                .setLinearHeadingInterpolation(spikeLeftPose.getHeading(), spikeMiddlePose.getHeading())
                .build();

        grabSpikeMarkRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeMiddlePose), new Point(spikeRightPose)))
                .setLinearHeadingInterpolation(spikeMiddlePose.getHeading(), spikeRightPose.getHeading())
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(spikeRightPose), new Point(specimenWallSpecimenControlPose), new Point(specimenWallSpecimenPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), specimenWallSpecimenPose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenWallSpecimenPose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenSpline1Pose),new Point(specimenChamber1Pose)))
                .setLinearHeadingInterpolation(specimenWallSpecimenPose.getHeading(), specimenChamber1Pose.getHeading())
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenChamber1Pose), new Point(specimenWallSpecimenSpline1Pose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenPose)))
                .setLinearHeadingInterpolation(specimenChamber1Pose.getHeading(), specimenWallSpecimenPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenWallSpecimenPose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenSpline1Pose),new Point(specimenChamber2Pose)))
                .setLinearHeadingInterpolation(specimenWallSpecimenPose.getHeading(), specimenChamber2Pose.getHeading())
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenChamber2Pose), new Point(specimenWallSpecimenSpline1Pose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenPose.getX() + 2, specimenWallSpecimenPose.getY())))
                .setLinearHeadingInterpolation(specimenChamber2Pose.getHeading(), specimenWallSpecimenPose.getHeading())
                .build();

        score3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenWallSpecimenPose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenSpline1Pose),new Point(specimenChamber3Pose)))
                .setLinearHeadingInterpolation(specimenWallSpecimenPose.getHeading(), specimenChamber3Pose.getHeading())
                .build();

        grab4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenChamber3Pose), new Point(specimenWallSpecimenSpline1Pose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenPose)))
                .setLinearHeadingInterpolation(specimenChamber3Pose.getHeading(), specimenWallSpecimenPose.getHeading())
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimenWallSpecimenPose), new Point(specimenWallSpecimenSpline2Pose), new Point(specimenWallSpecimenSpline1Pose), new Point(specimenChamber4Pose)))
                .setLinearHeadingInterpolation(specimenWallSpecimenPose.getHeading(), specimenChamber4Pose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(specimenChamber3Pose), new Point(132, 68), new Point(specimenObservationParkPose)));
        park.setLinearHeadingInterpolation(specimenChamber3Pose.getHeading(), specimenObservationParkPose.getHeading());

    }

    private boolean actionTimerActive = false;

    public void turnTo(double degrees) { // if you want to turn right, use negative degrees
        Pose temp = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(degrees));
        follower.holdPoint(temp);
    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                //  Move elevator + outtake to backside chamber position
                setElevator(elevatorBacksideChamber);
                setOuttakeElbow(outtakeElbowBacksideChamber);
                outtakeFlexor.setPosition(outtakeFlexorBacksideChamber);
                outtakePronator.setPosition(outtakePronatorBacksideChamber);

                follower.followPath(scorePreload);
                setPathState(1);
                break;
            //TODO: implement logic that will skip to the next path if its been stalled for too long (e.g. overrun)
            case 1:
                follower.setMaxPower(1.0);
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.5){
                    //  Score preload
                    setOuttakeGrabber(true);
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0){
                        setExtendo(1120);
                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        intakeGrabber.setPosition(intakeGrabberOpen);

                        actionTimerActive = false;

                        follower.followPath(grabSpikeMarkLeft, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                follower.setMaxPower(1);
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {
                    //  Prepare elevator + outtake for wall specimen
                    setElevator(elevatorWallSpecimen);
                    setOuttakeElbow(outtakeElbowWallSpecimen);
                    outtakeFlexor.setPosition(outtakeFlexorWallSpecimen);
                    outtakePronator.setPosition(outtakePronatorWallSpecimen);


                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.6){
                        setExtendo(extendoRetracted);
                        intakeElbow.setPosition(intakeElbowRetracted);
                        intakeFlexor.setPosition(intakeFlexorRetracted);

                        if (extendo.getCurrentPosition() < 50){
                            intakeGrabber.setPosition(intakeGrabberOpen);

                            actionTimerActive = false;
                            follower.followPath(grabSpikeMarkMiddle, true);
                            setPathState(3);
                        }
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.3){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.1){
                        intakeElbow.setPosition(intakeElbowActive);
                    }
                }
                break;
            case 3:
                if (follower.getCurrentTValue() < 0.7){
                    setExtendo(1120);
                    if (extendo.getCurrentPosition() > extendoMin){
                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    } else {
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    }
                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {

                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.6){
                        setExtendo(extendoRetracted);
                        intakeElbow.setPosition(intakeElbowRetracted);
                        intakeFlexor.setPosition(intakeFlexorRetracted);

                        if (extendo.getCurrentPosition() < 50){
                            intakeGrabber.setPosition(intakeGrabberOpen);

                            actionTimerActive = false;
                            follower.followPath(grabSpikeMarkRight, true);
                            setPathState(4);
                        }
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.3){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.1){
                        intakeElbow.setPosition(intakeElbowActive);
                    }
                }
                break;
            case 4:
                if (!actionTimerActive){
                    setExtendo(1300);
                    if (extendo.getCurrentPosition() > extendoMin){
                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        intakePronator.setPosition(intakePronatorNeutral + 0.1);
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    } else {
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    }
                }
                if (follower.getVelocityMagnitude() < 1 && follower.getCurrentTValue() > 0.7) {

                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.6){
                        turnTo(180);
                        setExtendo(extendoRetracted);
                        intakeElbow.setPosition(intakeElbowRetracted);
                        intakeFlexor.setPosition(intakeFlexorRetracted);
                        intakePronator.setPosition(intakePronatorNeutral);

                        if (extendo.getCurrentPosition() < 50){
                            intakeGrabber.setPosition(intakeGrabberOpen);

                            actionTimerActive = false;
                            follower.followPath(grab1, true);
                            setPathState(8);
                        }
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.3){
                        intakeGrabber.setPosition(intakeGrabberClosed);
                    } else if (actionTimer.getElapsedTimeSeconds() > 0.1){
                        intakeElbow.setPosition(intakeElbowActive);
                    }
                }
                break;
            case 8:
                if (follower.getCurrentTValue() > 0.7){
                    follower.setMaxPower(0.5);
                } else if (follower.getCurrentTValue() > 0.5){
                    follower.setMaxPower(0.7);
                } else {
                    follower.setMaxPower(1);
                }
                if (follower.getVelocity().getMagnitude() < 1 && follower.getCurrentTValue() > 0.5) {
                    //  Grab wall specimen
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0){ //0.3
                        setOuttakeGrabber(false);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.15){ //0.3
                        actionTimerActive = false;
                        follower.followPath(score1, false);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                follower.setMaxPower(1);
                //  Set elevator + outtake to high chamber position while moving to high chamber
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    setElevator(elevatorHighChamber);
                    setOuttakeElbow(outtakeElbowHighChamber);
                    outtakeFlexor.setPosition(outtakeFlexorHighChamber);
                    outtakePronator.setPosition(outtakePronatorHighChamber);
                }
                if (follower.getVelocity().getMagnitude() < 1 && follower.getCurrentTValue() > 0.5) {
                    //  Release scored specimen
                    setOuttakeGrabber(true);
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0) { //0.3
                        actionTimerActive = false;
                        follower.followPath(grab2, false);
                        setPathState(10);
                    }
                }
                break;
            case 10:
                if (follower.getCurrentTValue() > 0.7){
                    follower.setMaxPower(0.5);
                } else if (follower.getCurrentTValue() > 0.5){
                    follower.setMaxPower(0.7);
                } else {
                    follower.setMaxPower(1);
                }
                //  Set elevator + outtake to wall specimen position while moving to wall specimen
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setElevator(elevatorWallSpecimen);
                    setOuttakeElbow(outtakeElbowWallSpecimen);
                    outtakeFlexor.setPosition(outtakeFlexorWallSpecimen);
                    outtakePronator.setPosition(outtakePronatorWallSpecimen);
                }
                if (follower.getVelocity().getMagnitude() < 1 && follower.getCurrentTValue() > 0.5){
                    //  Grab wall specimen
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0){ //0.3
                        setOuttakeGrabber(false);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.15){ //0.3
                        actionTimerActive = false;
                        follower.followPath(score2, false);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                follower.setMaxPower(1);
                //  Set elevator + outtake to high chamber position while moving to high chamber
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    setElevator(elevatorHighChamber);
                    setOuttakeElbow(outtakeElbowHighChamber);
                    outtakeFlexor.setPosition(outtakeFlexorHighChamber);
                    outtakePronator.setPosition(outtakePronatorHighChamber);
                }
                if (follower.getVelocity().getMagnitude() < 1 && follower.getCurrentTValue() > 0.5) {
                    //  Release scored specimen
                    setOuttakeGrabber(true);
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0) { //0.3
                        actionTimerActive = false;
                        follower.followPath(grab3, false);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                if (follower.getCurrentTValue() > 0.7){
                    follower.setMaxPower(0.5);
                } else if (follower.getCurrentTValue() > 0.5){
                    follower.setMaxPower(0.7);
                } else {
                    follower.setMaxPower(1);
                }
                //  Set elevator + outtake to wall specimen position while moving to wall specimen
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setElevator(elevatorWallSpecimen);
                    setOuttakeElbow(outtakeElbowWallSpecimen);
                    outtakeFlexor.setPosition(outtakeFlexorWallSpecimen);
                    outtakePronator.setPosition(outtakePronatorWallSpecimen);
                }
                if (follower.getVelocity().getMagnitude() < 1 && follower.getCurrentTValue() > 0.5) {
                    //  Grab wall specimen
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0){ //0.3
                        setOuttakeGrabber(false);
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0.15){ //0.3
                        actionTimerActive = false;
                        follower.followPath(score3, false);
                        setPathState(13);
                    }
                }
                break;
            case 13:
                follower.setMaxPower(1);
                //  Set elevator + outtake to high chamber position while moving to high chamber
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    setElevator(elevatorHighChamber);
                    setOuttakeElbow(outtakeElbowHighChamber);
                    outtakeFlexor.setPosition(outtakeFlexorHighChamber);
                    outtakePronator.setPosition(outtakePronatorHighChamber);
                }
                if (follower.getVelocity().getMagnitude() < 1 && follower.getCurrentTValue() > 0.5) {
                    //  Release scored specimen
                    setOuttakeGrabber(true);
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();
                    }
                    if (actionTimer.getElapsedTimeSeconds() > 0) { //0.3
                        actionTimerActive = false;
                        follower.followPath(/*grab4*/park, false);
                        setPathState(/*14*/16);
                    }
                }
                break;
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
        follower.setStartingPose(specimenStartPose);
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
        setOuttakeElbow(0.86);
        outtakeFlexor.setPosition(0.25);
        outtakePronator.setPosition(outtakePronatorNeutral);
        setOuttakeGrabber(true);
        setExtendo(0);
        intakeElbow.setPosition(intakeElbowNeutral);
        intakeFlexor.setPosition(intakeFlexorNeutral);
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
