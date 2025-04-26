package pedroPathing.auto;

import static pedroPathing.constants.AutoConstants.sampleParkPose;
import static pedroPathing.constants.Constants_ITD.elevatorBasket;
import static pedroPathing.constants.Constants_ITD.elevatorPower;
import static pedroPathing.constants.Constants_ITD.elevatorTransfer;
import static pedroPathing.constants.Constants_ITD.extendoMax;
import static pedroPathing.constants.Constants_ITD.extendoMin;
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
import static pedroPathing.constants.Constants_ITD.intakePronatorLeft;
import static pedroPathing.constants.Constants_ITD.intakePronatorNeutral;
import static pedroPathing.constants.Constants_ITD.outtakeElbowBasket;
import static pedroPathing.constants.Constants_ITD.outtakeElbowNeutral;
import static pedroPathing.constants.Constants_ITD.outtakeElbowTransfer;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorBasket;
import static pedroPathing.constants.Constants_ITD.outtakeFlexorNeutral;
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous
public class SampleAutoV5 extends OpMode {
    private int intakeState = 0;
    private int outtakeState = 0;
    private double intakeStamp = 0;
    private double outtakeStamp = 0;
    private double hue = 0;

    List<LynxModule> allHubs;
    private DcMotorEx extendo, elevatorLeft, elevatorRight;
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow, outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;
    private ColorSensor colorSensor;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private static final Pose startPose = new Pose(136, 31.5, Math.toRadians(180));
    private static final Pose preloadPose = new Pose(126, 10, Math.toRadians(168));
    private static final Pose middlePose = new Pose(124.5, 9.0, Math.toRadians(172));
    private static final Pose basketPose = new Pose(126, 10, Math.toRadians(/*175*/168));
    private static final Pose leftPose = new Pose(117.5, /*20*/20.2, Math.toRadians(220));
    private static final Pose rightPose = new Pose(122.5, /*14*/14.1, Math.toRadians(160));
    private static final Pose subPose = new Pose(82, 54, Math.toRadians(90));
    private static final Point subControlPoint = new Point(78, 28);
    private static final Pose returnPose = new Pose(124, 12, Math.toRadians(156));
    private static final Point returnControlPoint = new Point(78, 28);


    private Path scorePreload, park;
    private PathChain grabMiddle, scoreMiddle, grabLeft, scoreLeft, grabRight, scoreRight, sub1, return1;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), preloadPose.getHeading());

        grabMiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preloadPose), new Point(middlePose)))
                .setLinearHeadingInterpolation(preloadPose.getHeading(), middlePose.getHeading())
                .build();

        scoreMiddle = follower.pathBuilder()
                .addPath(new BezierLine(new Point(middlePose), new Point(basketPose)))
                .setLinearHeadingInterpolation(middlePose.getHeading(), basketPose.getHeading())
                .build();

        grabLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(leftPose)))
                .setLinearHeadingInterpolation(basketPose.getHeading(), leftPose.getHeading())
                .build();
        scoreLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leftPose), new Point(basketPose)))
                .setLinearHeadingInterpolation(leftPose.getHeading(), basketPose.getHeading())
                .build();
        grabRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(basketPose), new Point(rightPose)))
                .setLinearHeadingInterpolation(basketPose.getHeading(), rightPose.getHeading())
                .build();
        scoreRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rightPose), new Point(basketPose)))
                .setLinearHeadingInterpolation(rightPose.getHeading(), basketPose.getHeading())
                .build();
        sub1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(basketPose), subControlPoint, new Point(subPose)))
                .setLinearHeadingInterpolation(basketPose.getHeading(), subPose.getHeading())
                .build();
        return1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose), returnControlPoint, new Point(returnPose)))
                .setLinearHeadingInterpolation(subPose.getHeading(), returnPose.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(returnPose), new Point(sampleParkPose)));
        park.setLinearHeadingInterpolation(returnPose.getHeading(), sampleParkPose.getHeading());

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
                break;
            case 1:
                if (follower.getCurrentTValue() > 0.6) {
                    follower.setMaxPower(0.5);
                }
                if (follower.getCurrentTValue() > 0.7 && intakeState == 0){
                    intakeElbow.setPosition(intakeElbowHover);
                    intakeFlexor.setPosition(intakeFlexorActive);
                    intakePronator.setPosition(intakePronatorNeutral + 0.05);
                    intakeGrabber.setPosition(intakeGrabberOpen);
                }
                if (follower.atParametricEnd()) {
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        outtakeState = -1;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();

                        setOuttakeElbow(outtakeElbowBasket);
                        outtakeFlexor.setPosition(outtakeFlexorBasket);
                        outtakePronator.setPosition(outtakePronatorBasket);

                    }

                }
                if (actionTimer.getElapsedTimeSeconds() > 0.5 + outtakeStamp && outtakeState == -1){
                    outtakeState = 1;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(true);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + outtakeStamp && outtakeState == 1){
                    outtakeState = 2;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 2){
                    intakeState = 1;
                    outtakeState = 3;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setElevator(elevatorTransfer);

                    actionTimerActive = false;
                    follower.followPath(grabMiddle, true);
                    setPathState(2);
                    intakeState = 0;
                    outtakeState = 0;

                    follower.setMaxPower(1);
                }
                break;
            case 2:
                if (follower.atParametricEnd()) {
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        intakeState = 1;
                        outtakeState = 1;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();

                        setExtendo(extendoMax);
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.7 + intakeStamp && intakeState == 1 && extendo.getCurrentPosition() > extendoMax - 20){
                    intakeState = 2;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowActive);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 2){
                    intakeState = 3;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 3){
                    intakeState = 4;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer);
                    intakeElbow.setPosition(intakeElbowTransfer);
                    intakeFlexor.setPosition(intakeFlexorTransfer);
                    intakePronator.setPosition(intakePronatorNeutral);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.8 + intakeStamp && intakeState == 4) {
                    intakeState = 5;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer - extendoTransferAmount);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 5){
                    intakeState = 6;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(false);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + intakeStamp && intakeState == 6) {
                    intakeState = 7;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberOpen);
                    setExtendo(extendoTransfer);
                }
                if (extendo.getCurrentPosition() > extendoTransfer - 50 && intakeState == 7){
                    intakeState = 8;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();
                    outtakeState = 4;

                    setElevator(elevatorBasket);
                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);

                }
                if (elevatorLeft.getCurrentPosition() > elevatorBasket - 100 && outtakeState == 4) {
                    outtakeState = 5;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowBasket);
                    outtakeFlexor.setPosition(outtakeFlexorBasket);
                    outtakePronator.setPosition(outtakePronatorBasket);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 5){
                    outtakeState = 6;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(true);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + outtakeStamp && outtakeState == 6){
                    outtakeState = 7;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 7) {
                    outtakeState = 8;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    actionTimerActive = false;
                    follower.followPath(grabLeft, true);
                    setPathState(3);
                    intakeState = 0;
                    outtakeState = 0;

                    follower.setMaxPower(1);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.3 && outtakeState == 0){
                    outtakeState = 1;
                    setElevator(elevatorTransfer);
                }
                if (intakeState == 0) {
                    intakeState = 1;
                    setExtendo(extendoMax);
                    intakeElbow.setPosition(intakeElbowHover);
                    intakeFlexor.setPosition(intakeFlexorActive);
                    intakePronator.setPosition(intakePronatorNeutral - 0.1);
                }
                if (follower.atParametricEnd()){
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        intakeState = 2;
                        outtakeState = 2;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 1 + intakeStamp && intakeState == 2){
                    intakeState = 3;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowActive);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 3){
                    intakeState = 4;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 4){
                    intakeState = 5;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer);
                    intakeElbow.setPosition(intakeElbowTransfer);
                    intakeFlexor.setPosition(intakeFlexorTransfer);
                    intakePronator.setPosition(intakePronatorNeutral);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.8 + intakeStamp && intakeState == 5) {
                    intakeState = 6;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer - extendoTransferAmount);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 6){
                    intakeState = 7;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(false);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + intakeStamp && intakeState == 7) {
                    intakeState = 8;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberOpen);
                    setExtendo(extendoTransfer);
                }
                if (extendo.getCurrentPosition() > extendoTransfer - 50 && intakeState == 8){
                    intakeState = 9;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();
                    outtakeState = 4;

                    setElevator(elevatorBasket);
                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);

                    actionTimerActive = false;
                    follower.followPath(scoreLeft, true);
                    setPathState(4);
                    intakeState = 0;
                    outtakeState = 0;
                }
                break;
            case 4:
                if (follower.atParametricEnd()) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        outtakeState = 1;
                        intakeState = 1;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();

                        setOuttakeElbow(outtakeElbowBasket);
                        outtakeFlexor.setPosition(outtakeFlexorBasket);
                        outtakePronator.setPosition(outtakePronatorBasket);
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 1){
                    outtakeState = 2;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(true);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + outtakeStamp && outtakeState == 2) {
                    outtakeState = 3;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 3) {
                    outtakeState = 4;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setElevator(elevatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 4) {
                    outtakeState = 5;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoMax);
                    intakeElbow.setPosition(intakeElbowHover);
                    intakeFlexor.setPosition(intakeFlexorActive);
                    intakePronator.setPosition(intakePronatorNeutral + 0.1);

                    actionTimerActive = false;
                    follower.followPath(grabRight, true);
                    setPathState(5);
                    outtakeState = 0;
                    intakeState = 0;
                }
                break;
            case 5:
                if (follower.atParametricEnd()) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        outtakeState = 1;
                        intakeState = 1;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.8 + intakeStamp && intakeState == 1) {
                    intakeState = 2;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowActive);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 2) {
                    intakeState = 3;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 3){
                    intakeState = 4;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer);
                    intakeElbow.setPosition(intakeElbowTransfer);
                    intakeFlexor.setPosition(intakeFlexorTransfer);
                    intakePronator.setPosition(intakePronatorNeutral);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.8 + intakeStamp && intakeState == 4){
                    intakeState = 5;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer - extendoTransferAmount);

                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 5){
                    intakeState = 6;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(false);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + intakeStamp && intakeState == 6) {
                    intakeState = 7;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberOpen);
                    setExtendo(extendoTransfer);
                }
                if (extendo.getCurrentPosition() > extendoTransfer - 50 && intakeState == 7) {
                    intakeState = 8;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setElevator(elevatorBasket);
                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);

                    actionTimerActive = false;
                    follower.followPath(scoreRight, true);
                    setPathState(6);
                    intakeState = 0;
                    outtakeState = 0;
                }
                break;
            case 6:
                if (follower.atParametricEnd()) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        outtakeState = 1;
                        intakeState = 1;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();

                        setOuttakeElbow(outtakeElbowBasket);
                        outtakeFlexor.setPosition(outtakeFlexorBasket);
                        outtakePronator.setPosition(outtakePronatorBasket);
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 1){
                    outtakeState = 2;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(true);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + outtakeStamp && outtakeState == 2) {
                    outtakeState = 3;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 3) {
                    outtakeState = 4;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setElevator(elevatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 4){
                    setExtendo(150);
                    intakeElbow.setPosition(intakeElbowNeutral);
                    intakeFlexor.setPosition(intakeFlexorNeutral);

                    actionTimerActive = false;
                    follower.followPath(sub1, true);
                    setPathState(7);
                    outtakeState = 0;
                    intakeState = 0;
                }
                break;
            case 7:
                if (follower.getCurrentTValue() > 0.7) {
                    follower.setMaxPower(0.5);
                }
                if (follower.getVelocityMagnitude() < 5 && follower.getCurrentTValue() > 0.7) {
                    if (!actionTimerActive) {
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        outtakeState = 1;
                        intakeState = -100;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();

                        intakeElbow.setPosition(intakeElbowHover);
                        intakeFlexor.setPosition(intakeFlexorActive);
                        follower.breakFollowing();
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + intakeStamp && intakeState == -100){
                    intakeState = -101;
                }
                if (intakeState == -101){
                    while (!getDetectedSample().equals("yellow") && opmodeTimer.getElapsedTimeSeconds() < 27 && extendo.getCurrentPosition() < extendoMax - 50){
                        extendo.setTargetPosition(extendoMax);
                        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extendo.setPower(0.2); //0.2
                    }
                    if (extendo.getCurrentPosition() >= extendoMax - 50) {
                        setExtendo(extendoRetracted);
                        intakeElbow.setPosition(intakeElbowNeutral);
                        intakeFlexor.setPosition(intakeFlexorNeutral);
                        follower.followPath(return1, true);
                        setPathState(-1);
                        follower.setMaxPower(1);
                    } else if (extendo.getCurrentPosition() >= 250){
                        extendo.setPower(-0.07);
                        intakeState = 1;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        follower.setMaxPower(1);
                    }

                }
//                if (!getDetectedSample().equals("yellow") && extendo.getCurrentPosition() < extendoMax - 50 && intakeState == -100){
//                    extendo.setTargetPosition(extendoMax);
//                    extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    extendo.setPower(0.2);
//                } else {
//                    extendo.setPower(0.1);
//                    intakeState = 1;
//                    intakeStamp = actionTimer.getElapsedTimeSeconds();
//                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 1) {
                    intakeState = 2;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowActive);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 2) {
                    intakeState = -1;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberClosed);
                    extendo.setPower(0);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -1) {
                    intakeState = -2;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowHover);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -2) {
                    intakeState = -3;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakePronator.setPosition(0.38);
                    intakeGrabber.setPosition(intakeGrabberOpen);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -3) {
                    intakeState = -4;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowActive);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -4) {
                    intakeState = -5;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -5) {
                    intakeState = -6;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowHover);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -6) {
                    intakeState = -7;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakePronator.setPosition(intakePronatorLeft);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -7) {
                    intakeState = -8;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberOpen);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -8) {
                    intakeState = -9;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeElbow.setPosition(intakeElbowActive);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == -9) {
                    intakeState = 3;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberClosed);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 3){
                    intakeState = 4;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowTransfer);
                    outtakeFlexor.setPosition(outtakeFlexorTransfer);
                    outtakePronator.setPosition(outtakePronatorTransfer);

                    setExtendo(extendoTransfer);
                    intakeElbow.setPosition(intakeElbowTransfer);
                    intakeFlexor.setPosition(intakeFlexorTransfer);
                    intakePronator.setPosition(intakePronatorNeutral);

                }
                if (actionTimer.getElapsedTimeSeconds() > 0.8 + intakeStamp && intakeState == 4){
                    intakeState = 5;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setExtendo(extendoTransfer - extendoTransferAmount);
                    if (!getDetectedSample().equals("yellow")) {
                        intakeGrabber.setPosition(intakeGrabberOpen);
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.2 + intakeStamp && intakeState == 5){
                    intakeState = 6;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(false);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + intakeStamp && intakeState == 6) {
                    intakeState = 7;
                    intakeStamp = actionTimer.getElapsedTimeSeconds();

                    intakeGrabber.setPosition(intakeGrabberOpen);
                    setExtendo(extendoTransfer);
                    actionTimerActive = false;
                    follower.followPath(return1, true);
                    setPathState(8);
                    intakeState = 0;
                    outtakeState = 0;
                }
                break;
            case 8:
                follower.setMaxPower(1);
                if (pathTimer.getElapsedTimeSeconds() > 1 && outtakeState == 0) {
                    outtakeState = 1;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setElevator(elevatorBasket);
                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);
                    outtakePronator.setPosition(outtakePronatorBasket);
                }
                if (follower.atParametricEnd()) {
                    if (!actionTimerActive){
                        actionTimerActive = true;
                        actionTimer.resetTimer();

                        outtakeState = 2;
                        intakeStamp = actionTimer.getElapsedTimeSeconds();
                        outtakeStamp = actionTimer.getElapsedTimeSeconds();
                        setOuttakeElbow(outtakeElbowBasket);
                        outtakeFlexor.setPosition(outtakeFlexorBasket);
                        setExtendo(extendoRetracted);
                        intakeElbow.setPosition(intakeElbowNeutral);
                        intakeFlexor.setPosition(intakeFlexorNeutral);
                    }
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 2) {
                    outtakeState = 3;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeGrabber(true);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.1 + outtakeStamp && outtakeState == 3) {
                    outtakeState = 4;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setOuttakeElbow(outtakeElbowNeutral);
                    outtakeFlexor.setPosition(outtakeFlexorNeutral);
                    outtakePronator.setPosition(outtakePronatorTransfer);
                }
                if (actionTimer.getElapsedTimeSeconds() > 0.3 + outtakeStamp && outtakeState == 4) {
                    outtakeState = 5;
                    outtakeStamp = actionTimer.getElapsedTimeSeconds();

                    setElevator(0);
                }
                break;
            case -1:
                follower.setMaxPower(1);
                break;
        }
    }


    private String getDetectedSample(){
        hue = JavaUtil.rgbToHue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        if (hue < 35) {
            return "red";
        } else if (hue < 90) {
            return "yellow";
        } else if (hue < 200) {
            return "kill yourself";
        } else {
            return "blue";
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
        if (position < elevatorLeft.getCurrentPosition()){
            elevatorLeft.setPower(elevatorPower * 0.6);
            elevatorRight.setPower(elevatorPower * 0.6);
        } else {
            elevatorLeft.setPower(elevatorPower);
            elevatorRight.setPower(elevatorPower);
        }
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
        follower.setStartingPose(startPose);
        buildPaths();

        allHubs = hardwareMap.getAll(LynxModule.class);

        //  Motor Initialization
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");

        //  Color Sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        ((NormalizedColorSensor) colorSensor).setGain(2);

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
        telemetry.addLine("Ensure the following:\nKickstand is all the way up\nExtendo is retracted\nElevator is retracted\nSample is preloaded\nGamepads are connected\nColor Sensor light is on\n\tGood Luck!");
        telemetry.addLine(colorSensor.getI2cAddress().toString());
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

        telemetry.addLine("" + opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("hue", hue);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("intake state", intakeState);
        telemetry.addData("intake stamp", intakeStamp);
        telemetry.addData("outtake state", outtakeState);
        telemetry.addData("outtake stamp", outtakeStamp);
        telemetry.addData("action timer", actionTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
