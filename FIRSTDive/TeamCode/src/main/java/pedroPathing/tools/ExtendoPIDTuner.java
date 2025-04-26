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
public class ExtendoPIDTuner extends OpMode {

    private DcMotorEx extendo;
    public static double kp = 0.0001;
    public static double kf = 0.2;
    public static double period = 1;
    public static int goal = 0;

    @Override
    public void init() {
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double elevatorPIDControl(double target, double current){
        double error = target - current;

        double power = error * kp;
        return power + kf;
    }

    @Override
    public void loop() {
        extendo.setPower(elevatorPIDControl(goal, extendo.getCurrentPosition()));
        extendo.setPower(elevatorPIDControl(goal, extendo.getCurrentPosition()));
    }
}
