package pedroPathing.constants;

import com.pedropathing.localization.Pose;

public class AutoConstants {
    public static final double positionThreshold = 0.5;


    //  Specimen Poses
    public static final Pose specimenStartPose = new Pose(136, 78, Math.toRadians(0));

    public static final Pose specimenPreloadPose = new Pose(103.5, 73.5, Math.toRadians(0));
    public static final Pose specimenChamber1Pose = new Pose(103, 63, Math.toRadians(180));
    public static final Pose specimenChamber2Pose = new Pose(103, 65.5, Math.toRadians(180));
    public static final Pose specimenChamber3Pose = new Pose(103, 66.5, Math.toRadians(180));
    public static final Pose specimenChamber4Pose = new Pose(103, 70, Math.toRadians(180));

    public static final Pose specimenChamber1ControlPose = new Pose(130, 70, Math.toRadians(270));
    public static final Pose specimenChamber2ControlPose = new Pose(130, 72, Math.toRadians(270));
    public static final Pose specimenChamber3ControlPose = new Pose(130, 74, Math.toRadians(270));
    public static final Pose specimenChamber4ControlPose = new Pose(130, 76, Math.toRadians(270));


    public static final Pose specimenSpikeLeftPose = new Pose(/*84*/87, 110, Math.toRadians(180));
    public static final Pose specimenSpikeLeftControlPose = new Pose(/*132*/126, /*100*/114, Math.toRadians(180));
    public static final Pose specimenObservationLeftPose = new Pose(124, 120, Math.toRadians(180));
    public static final Pose specimenObservationLeftControlPose = new Pose(/*87*/85, /*120*/124, Math.toRadians(180));

    public static final Pose specimenSpikeMiddlePose = new Pose(/*80*/84, 118/*116*/, Math.toRadians(180));
    public static final Pose specimenObservationMiddlePose = new Pose(124, 126, Math.toRadians(180));
    public static final Pose specimenObservationMiddleControlPose = new Pose(/*80*/84, 130/*126*/, Math.toRadians(180));

    public static final Pose specimenSpikeRightPose = new Pose(/*80*/84, 126, Math.toRadians(180));
    public static final Pose specimenObservationRightPose = new Pose(124, 131.0, Math.toRadians(180));
    public static final Pose specimenObservationRightControlPose = new Pose(/*80*/84, 134, Math.toRadians(180));

    public static final Pose specimenSpikeLeftAnglePose = new Pose(120, 121, Math.toRadians(179.9));
    public static final Pose specimenSpikeMiddleAnglePose = new Pose(120, 121, Math.toRadians(150));
    public static final Pose specimenSpikeRightAnglePose = new Pose(120, 121, Math.toRadians(140));
    public static final Pose specimenSpikeNeutralAnglePose = new Pose(120, 121, Math.toRadians(180));

    public static final Pose specimenWallSpecimenPose = new Pose(/*138*/138.3, /*108*/107, Math.toRadians(180));
    public static final Pose specimenFarWallSpecimenPose = new Pose(132, 131.5, Math.toRadians(270));
    public static final Pose specimenWallSpecimenControlPose = new Pose(/*120*/100, 108, Math.toRadians(180));
    public static final Pose specimenWallSpecimenSpline1Pose = new Pose(/*132*/120, 70, Math.toRadians(180));
    public static final Pose specimenWallSpecimenSpline2Pose = new Pose(/*112*/120, /*108*/107, Math.toRadians(180));

    public static final Pose specimenObservationParkPose = new Pose(132, 108, Math.toRadians(225));



    //  Sample Poses
    public static final Pose sampleStartPose = new Pose(136, 55 - 23.5, Math.toRadians(90));

    public static final Pose samplePreloadPose = new Pose(128.5, 11.2, Math.toRadians(140));
    public static final Pose samplePreloadControlPose = new Pose(/*120*/128, 26, Math.toRadians(135));

    public static final Pose sampleBasketPose = new Pose(128, 12, Math.toRadians(140));

    public static final Pose sampleSpikeRightPose = new Pose(116, 23, Math.toRadians(180));
    public static final Pose sampleSpikeMiddlePose = new Pose(116, 12.2, Math.toRadians(180));
    public static final Pose sampleSpikeLeftPose = new Pose(116.5, 12.7, Math.toRadians(213));

    public static final Pose sampleParkPose = new Pose(128, 36, Math.toRadians(90));


}
