package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants_ITD {

    //  Gen 2
    //  Gamepad
    public static double  triggerThreshold            =   0.3;


    //  Intake Grabber
    public static double  intakeGrabberOpen           =   0.45; //0.46
    public static double  intakeGrabberClosed         =   0.205;//0.214

    //  Intake Pronator
    public static double  intakePronatorNeutral       =   0.492;
    public static double  intakePronatorLeft          =   intakePronatorNeutral - 0.28;
    public static double  intakePronatorRight         =   intakePronatorNeutral + 0.25;

    //  Intake Flexor
    public static double  intakeFlexorRetracted       =   0.01;
    public static double  intakeFlexorTransfer        =   0.06;
    public static double  intakeFlexorActive          =   0.83;
    public static double  intakeFlexorNeutral         =   0.50;
    public static double  intakeFlexorChamber         =   0.45;

    //  Intake Elbow
    public static double  intakeElbowRetracted        =   0.2;
    public static double  intakeElbowTransfer         =   0.45;
    //TODO: HOVER in teleop
    public static double  intakeElbowHover            =   0.63;// 0.65
    public static double  intakeElbowActive           =   0.675; //0.67
    public static double  intakeElbowNeutral          =   0.30; //0.33

    //  Extendo
    public static double  extendoMultiplier           =   1/1.3;
    public static double  extendoPower                =   1;
    public static int     extendoRetracted            = (int) (15 * extendoMultiplier);
    public static int     extendoTransferAmount       = 260; //230
    public static int     extendoTransfer             = 930;
    public static int     extendoActive               = (int) (700 * extendoMultiplier);
    public static int     extendoMin                  = (int) (600 * extendoMultiplier);
    public static int     extendoMax                  = (int) (1400 * extendoMultiplier);
    public static int     extendoIncrement            = (int) (200 * extendoMultiplier);

    //  Elevator
    public static double  elevatorMultiplier          =   145.1/537.7;
    public static double  elevatorPower               =   1;
    public static int     elevatorTransfer            = (int) (0 * elevatorMultiplier); //40
    public static int     elevatorWallSpecimen        = (int) (1200 * elevatorMultiplier); //1216
//    public static int     elevatorHighChamber         = (int) (1630 * elevatorMultiplier); //1690
    public static int     elevatorHighChamber         = 408;

    public static int     elevatorBacksideChamber     = (int) (1580 * elevatorMultiplier); //1615
    public static int     elevatorBasket              = (int) (3400 * elevatorMultiplier);
    public static int     elevatorLowBasket           = 390;
    public static int     elevatorRiggingAboveLow     = (int) (1800 * elevatorMultiplier);
    public static int     elevatorRiggingExchange     = (int) (500 * elevatorMultiplier);
    public static int     elevatorRiggingAboveHigh    = (int) (3300 * elevatorMultiplier);
    public static int     elevatorRiggingFinal        = (int) (900 * elevatorMultiplier);
    public static double  elevatorUpKp                  = 0.0001;
    public static double  elevatorUpKf                  = 0.2;
    public static double  elevatorDownKp                  = 0.0001;
    public static double  elevatorDownKf                  = 0.2;
    //tuning pid, start with kf = 0, minimize error between actual and target

    //  Outtake Elbow
    public static double  outtakeElbowNeutral         =   0.24 + 0.03;
    public static double  outtakeElbowTransfer        =   0.06 + 0.03; //0.04 + 0.03
    public static double  outtakeElbowWallSpecimen    =   0.97 + 0.03;
    public static double  outtakeElbowHighChamber     =   0.33; //0.3
    public static double  outtakeElbowBasket          =   0.6 + 0.03; //0.63
    public static double  outtakeElbowBacksideChamber =   0.655 + 0.03;

    //  Outtake Pronator
    public static double  outtakePronatorNeutral      =   0.685;
    public static double  outtakePronatorTransfer     =   0.685;
    public static double  outtakePronatorWallSpecimen =   0.685;
    public static double outtakePronatorHighChamber   =   0.685;
    public static double  outtakePronatorBasket       =   0.125;
    public static double outtakePronatorBacksideChamber =   0.125;

    //  Outtake Flexor
    public static double  outtakeFlexorNeutral        =   0.5;
    public static double  outtakeFlexorTransfer       =   0.24; //0.22
    public static double  outtakeFlexorWallSpecimen   =   0.40; //0.41
    public static double  outtakeFlexorHighChamber    =   0.635; //0.61
    public static double  outtakeFlexorBasket         =   0.38; //0.37
    public static double  outtakeFlexorBacksideChamber=   0.408;

    //  Outtake Grabber
    // TODO: get these numbers
    public static double  outtakeGrabberLeftOpen      =   0.6; //0.7
    public static double  outtakeGrabberRightOpen     =   0.6; //0.7
    public static double  outtakeGrabberLeftClosed    =   0.96;
    public static double  outtakeGrabberRightClosed   =   0.96;

    //  Kickstand
    public static int     kickstandRetracted          =   0;
    public static int     kickstandActive             =   650; //650
    public static int     kickstandStabilize          =   850; //800


    //  Miscellaneous
    public static double swapModeDelay =   0.3;
}
