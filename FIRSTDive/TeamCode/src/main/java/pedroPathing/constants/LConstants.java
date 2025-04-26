package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00297609821124097095204141736677;
        ThreeWheelConstants.strafeTicksToInches = 0.002944723495443822563485822656;
        ThreeWheelConstants.turnTicksToInches = 0.0029930197629;
        ThreeWheelConstants.leftY = 6;
        ThreeWheelConstants.rightY = -6;
        ThreeWheelConstants.strafeX = -4.375;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "Motor FL";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "Motor FR";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "Motor BL";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




