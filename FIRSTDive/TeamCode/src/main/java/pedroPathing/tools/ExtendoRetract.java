package pedroPathing.tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class ExtendoRetract extends OpMode {
    private DcMotorEx extendo;
    @Override
    public void init() {
        extendo = hardwareMap.get(DcMotorEx.class, "Motor Extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        extendo.setPower(-gamepad1.left_stick_y * 0.2);
        telemetry.addLine("Extendo position: " + extendo.getCurrentPosition());
        telemetry.addLine("Extendo current: " + extendo.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
