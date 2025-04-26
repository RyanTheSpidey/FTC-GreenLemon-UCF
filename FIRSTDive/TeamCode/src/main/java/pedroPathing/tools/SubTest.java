package pedroPathing.tools;

import static pedroPathing.constants.Constants_ITD.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous
public class SubTest extends OpMode {
    double hue = 0;
    private DcMotorEx extendo, elevatorLeft, elevatorRight;
    private Servo intakeGrabber, intakePronator, intakeFlexor, intakeElbow, outtakePronator, outtakeFlexor, outtakeElbowLeft, outtakeElbowRight, outtakeGrabberLeft, outtakeGrabberRight;
    private ColorSensor colorSensor;

    private String getDetectedSample(){
        hue = JavaUtil.rgbToHue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        if (hue < 35) {
            return "red";
        } else if (hue < 85) {
            return "yellow";
        } else if (hue < 200) {
            return "kill yourself";
        } else {
            return "blue";
        }
    }

    @Override
    public void init() {
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
        intakeElbow.setPosition(intakeElbowHover);
        intakeFlexor.setPosition(intakeFlexorActive);
        intakePronator.setPosition(intakePronatorNeutral);

    }
    int intakeState = 0;

    @Override
    public void loop() {
        if (!getDetectedSample().equals("yellow") && getRuntime() < 27 && intakeState == 0 && extendo.getCurrentPosition() < extendoMax - 50){
            extendo.setTargetPosition(extendoMax);
            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendo.setPower(0.2);
        } else {
            extendo.setPower(0.1);
            intakeState = 1;
        }
        if (intakeState == 1){
            intakeElbow.setPosition(intakeElbowActive);
        }
    }
}
