package pedroPathing.constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        FollowerConstants.leftFrontMotorName = "Motor FL";
        FollowerConstants.leftRearMotorName = "Motor BL";
        FollowerConstants.rightFrontMotorName = "Motor FR";
        FollowerConstants.rightRearMotorName = "Motor BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14;

        FollowerConstants.xMovement = 76; //71
        FollowerConstants.yMovement = 52;

        FollowerConstants.forwardZeroPowerAcceleration = -35;
        FollowerConstants.lateralZeroPowerAcceleration = -73;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.4,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(/*0.06*/0.2,0.00,0.0001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.015,0.003,0,0.8,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0004;

        FollowerConstants.pathEndTimeoutConstraint = 0; //100
        FollowerConstants.pathEndTValueConstraint = 0.995; //0.995
        FollowerConstants.pathEndVelocityConstraint = 0.1; //0.07
        FollowerConstants.pathEndTranslationalConstraint = 0.3; //0.1
        FollowerConstants.pathEndHeadingConstraint = 0.02; //0.007

        FollowerConstants.useBrakeModeInTeleOp = true;
    }
}
