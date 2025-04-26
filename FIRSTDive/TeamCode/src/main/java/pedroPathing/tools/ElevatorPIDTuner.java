package pedroPathing.tools;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Config
@TeleOp
public class ElevatorPIDTuner extends OpMode {

    private DcMotorEx elevatorLeft, elevatorRight;
    public static double upkp = 0.0001;
    public static double downkp = 0.0001;
    public static double upkf = 0.2;
    public static double downkf = 0.2;
    public static double period = 1;
    public static int goal = 0;

    @Override
    public void init() {
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "Motor Elevator L");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "Motor Elevator R");
        elevatorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double elevatorPIDControl(double target, double current){
        double error = target - current;

        double power = error * (error > 0 ? upkp : downkp);
        return power + (error > 0 ? upkf : downkf);
    }


    @Override
    public void loop() {
        elevatorLeft.setPower(elevatorPIDControl(goal, elevatorLeft.getCurrentPosition()));
        elevatorRight.setPower(elevatorPIDControl(goal, elevatorRight.getCurrentPosition()));
    }
}
