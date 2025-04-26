package pedroPathing.tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
@TeleOp
public class ColorSensorTest extends OpMode {
    private ColorSensor colorSensor;
    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        ((NormalizedColorSensor) colorSensor).setGain(2);
    }

    @Override
    public void loop() {
        double hue = JavaUtil.rgbToHue(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addLine("Hue: " + hue);
        telemetry.update();
    }
}
