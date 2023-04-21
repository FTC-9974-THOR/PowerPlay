package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AdrianControls.Claw;
import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;
import org.firstinspires.ftc.teamcode.AdrianControls.RotatingArm;
import org.firstinspires.ftc.teamcode.AdrianControls.TurretWithPid;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff2023;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive9974;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import java.util.concurrent.TimeUnit;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enablses one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:S
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Config
@Autonomous(group = "advanced")
@Disabled
public class AutoPowerPlayWithFiveConesAndRotatorArmAndTurretWithPID extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        // region STATES FOR LEVEL MIDDLE
        IDLE,
        raiseTheLinearSlide,
        initalForwardTowardsFirstPole,
        placeTheConeInFirstPole,
        forwardOverShootForSignal,
        forwardTowardsYConeStack,
        forwardTowardsXConeStack,
        bringDownTheSlideToCorrectLevelAndCloseClaw,
        pickupSecondCone,
        backLittleTowardsSecondPole,
        backTowardsSecondPole,
        placeTheConeInSecondPole,
        forwardTowardsParking,
        goToTheParkingPosition,
        lowerTheRotatorAndSlide,
        parkPosition
    }
    private int teamColor;//1=Left -1= Right
    private boolean slideToSide = false;
    private int counterForAutoCycle = 1;

    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String targetZone = "D";
    private static final String VUFORIA_KEY =
            "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";
    private VuforiaLocalizer vuforia;
    public VuforiaStuff2023 vuforiaStuff;
    private TFObjectDetector tfod;

    public static  double yValueForChannel = 0.0;
    public static  double yValueForChannelLeft = -20.1;//20.3
    public static  double yValueForChannelRight = -20.3;
    public static  double yValueForChannelHighPole = 0.0;
    public static  double yValueForChannelHighPoleLeft = -21.5;
    public static  double yValueForChannelHighPoleRight = -20;
    public static  double yValueForChannelForConeDropOff = 0.0;
    public static  double yValueForChannelForConeDropOffLeft = -20.25;
    //yvalueforchannelforconedropoffLeft
    // RedLeftOchoa1 - -19.25
    // Red LeftOchoa2 - 20.25


    //-20.25 //-18.75//18.25 //19.5//22
    public static  double yValueForChannelForConeDropOffRight = -17.8;//18.3 //18.35//18.35//18.25
    public static  double xValueForwardTowardsXConeStack = 0.0;
    public static  double xValueForwardTowardsXConeStackLeftClaw1 = -59.1; //-58.0 //-57.75 //57.25//Reduced, orig is -58.1 //-57.9
    public static  double xValueForwardTowardsXConeStackRightClaw1 = -59.5;  //-60.75
    public static  double xValueForwardTowardsXConeStackLeftClaw2 = -59.5;
    public static  double xValueForwardTowardsXConeStackRightClaw2 = -59.7;
    public static  double xValueForChannel = 0.0;
    public static  double xValueForChannelLeft = -36.75;
    // -36.5 Left Red Ochoa 1
    // -36.75 Left Red Ochoca 2

    // -35.5//-34.75//34 //33.5//35.5 //36.5//-32.25; 45 deg rotator arm//37  90 deg rotator arm
    public static  double xValueForChannelRight = -36; //-36.50//34.5//36.5


    public static double distanceForOneDotLeft = -57;
    public static double distanceForTwoDotLeft = -35;
    public static double distanceForThreeDotLeft = -10.25;
    public static double distanceForOneDotRight = -58;
    public static double distanceForTwoDotRight = -36;
    public static double distanceForThreeDotRight = -12;
    public static double distanceForOneDot = 0.0;
    public static double distanceForTwoDot = 0.0;
    public static double distanceForThreeDot = 0.0;



    public static double MAX_VEL_OVERRIDE = DriveConstants.MAX_VEL*0.85; //0.7
    public static  double MAX_ACCEL_OVERRIDE = DriveConstants.MAX_ACCEL*0.85; //0.7
    public static double MAX_VEL_OVERRIDE_FAST = DriveConstants.MAX_VEL*0.85; //0.7
    public static  double MAX_ACCEL_OVERRIDE_FAST = DriveConstants.MAX_ACCEL*0.85; //0.7
    public static double MAX_VEL_OVERRIDE_FOR_OVERSHOOT_CONE = DriveConstants.MAX_VEL;//*0.95; //0.7
    public static  double MAX_ACCEL_OVERRIDE_FOR_OVERSHOOT_CONE = DriveConstants.MAX_ACCEL;//*0.95; //0.7

    public static double MAX_VEL_OVERRIDE_SLOW = DriveConstants.MAX_VEL*0.75; //0.7
    public static  double MAX_ACCEL_OVERRIDE_SLOW = DriveConstants.MAX_ACCEL*0.75; //0.7
    public static double MAX_VEL_OVERRIDE_SUPER_SLOW = DriveConstants.MAX_VEL*0.55; //0.7
    public static  double MAX_ACCEL_OVERRIDE_SUPER_SLOW = DriveConstants.MAX_ACCEL*0.55; //0.7
    public AutoPowerPlayWithFiveConesAndRotatorArmAndTurretWithPID(int TeamColor, boolean SlideToSide) {
        super();
        teamColor = TeamColor;
        slideToSide = SlideToSide;

    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);
        LinearSlidePIDWithVelocity linearSlide = new LinearSlidePIDWithVelocity(hardwareMap, true);
        Claw claw = new Claw(hardwareMap, linearSlide);
        TurretWithPid turret = new TurretWithPid(hardwareMap);
        RotatingArm rotatingArm = new RotatingArm(hardwareMap, linearSlide);

        // Initialize MecanumDrive9974
        MecanumDrive9974 drive = new MecanumDrive9974(hardwareMap);
        // Define our start pose
        // This assumes we start at x: 15, y: 10, heading: 180 degrees
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //Initialize Magnetic Switch Operated Turret.
        // MagneticSwitch turret = new MagneticSwitch(hardwareMap);
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaStuff = new VuforiaStuff2023(vuforia);
        Pose2d startPose = new Pose2d(-32.5*teamColor, -72 , Math.toRadians(90)); //-36
        if(teamColor ==1)
        {
            yValueForChannel = yValueForChannelLeft;
            yValueForChannelHighPole = yValueForChannelHighPoleLeft;
            yValueForChannelForConeDropOff = yValueForChannelForConeDropOffLeft;
            xValueForChannel = xValueForChannelLeft;
            distanceForOneDot = distanceForOneDotLeft;
            distanceForTwoDot = distanceForTwoDotLeft;
            distanceForThreeDot = distanceForThreeDotLeft;


            if(claw.clawNum == 1){
                xValueForwardTowardsXConeStack = xValueForwardTowardsXConeStackLeftClaw1;
            }
            else
            {
                xValueForwardTowardsXConeStack = xValueForwardTowardsXConeStackLeftClaw2;
            }
        }
        else
        {
            yValueForChannel = yValueForChannelRight;
            yValueForChannelHighPole = yValueForChannelHighPoleRight;
            yValueForChannelForConeDropOff = yValueForChannelForConeDropOffRight;
            xValueForChannel = xValueForChannelRight;
            distanceForOneDot = distanceForOneDotRight;
            distanceForTwoDot = distanceForTwoDotRight;
            distanceForThreeDot = distanceForThreeDotRight;
            if(claw.clawNum == 1){
                xValueForwardTowardsXConeStack = xValueForwardTowardsXConeStackRightClaw1;
            }
            else
            {
                xValueForwardTowardsXConeStack = xValueForwardTowardsXConeStackRightClaw2;
            }
        }
        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        //region TRAJECTORIES
        Trajectory initialForwardTowardsFirstPole = drive.trajectoryBuilder(startPose)
                //.lineTo(new Vector2d(-36, -64 ))// this is for the low pole. //41.65
                .lineToLinearHeading(new Pose2d(-36*teamColor, -41.9 ,Math.toRadians(90)) //-41.4
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_FAST, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_FAST)
                ) // this is for the middle pole.
                .build();
        Trajectory initialForwardToLowerTheRotator = drive.trajectoryBuilder(initialForwardTowardsFirstPole.end())
                .lineToLinearHeading(new Pose2d(-36*teamColor, -39 ,Math.toRadians(90))
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE)
                )
                .build();
        Trajectory forwardToOverShootSignalCone = drive.trajectoryBuilder(initialForwardTowardsFirstPole.end())
                .lineToLinearHeading(new Pose2d(-36*teamColor, -16 ,Math.toRadians(90)) //og is 14
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_FOR_OVERSHOOT_CONE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_FOR_OVERSHOOT_CONE)
                )
                .build();
        Trajectory forwardTowardsYConeStack = drive.trajectoryBuilder(forwardToOverShootSignalCone.end())
                .lineToLinearHeading(new Pose2d(-36*teamColor, yValueForChannel,Math.toRadians(90))
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE)
                )
                .build();
        double newAngleToUse = Math.toRadians(90)+Math.toRadians(90*teamColor);
        Trajectory forwardTowardsXConeStack = drive.trajectoryBuilder(forwardTowardsYConeStack.end().plus(new Pose2d(0,0,Math.toRadians(90*teamColor))))
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_FAST, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_FAST)
                )
                .build();
//region Second Pole
        Trajectory backLittleTowardsSecondPole = drive.trajectoryBuilder(forwardTowardsXConeStack.end())
                .lineToLinearHeading(new Pose2d(-59*teamColor, yValueForChannel,newAngleToUse)
                        , MecanumDrive9974.getVelocityConstraint(MAX_VEL_OVERRIDE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE)
                )
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsSecondPole = drive.trajectoryBuilder(forwardTowardsXConeStack.end())
                .lineToLinearHeading(new Pose2d(xValueForChannel*teamColor, yValueForChannelForConeDropOff,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SLOW)
                )
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
//region Third Pole        
        Trajectory forwardTowardsXConeStackThirdPole = drive.trajectoryBuilder(backTowardsSecondPole.end())
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_FAST, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_FAST)
                )
                .build();
        Trajectory backLittleTowardsThirdPole = drive.trajectoryBuilder(forwardTowardsXConeStackThirdPole.end())
                .lineToLinearHeading(new Pose2d(-59*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE)
                )
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsThirdPole = drive.trajectoryBuilder(forwardTowardsXConeStackThirdPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannel*teamColor, yValueForChannelForConeDropOff,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SLOW)
                )
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
//region Fourth Pole        
        Trajectory forwardTowardsXConeStackFourthPole = drive.trajectoryBuilder(backTowardsThirdPole.end())
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_FAST, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_FAST)
                )
                .build();
        Trajectory backLittleTowardsFourthPole = drive.trajectoryBuilder(forwardTowardsXConeStackFourthPole.end())
                .lineToLinearHeading(new Pose2d(-59*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE)
                )
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsFourthPole = drive.trajectoryBuilder(forwardTowardsXConeStackFourthPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannel*teamColor, yValueForChannelForConeDropOff,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SLOW)
                )
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion


//region Fifth Pole
        Trajectory forwardTowardsXConeStackFifthPole = drive.trajectoryBuilder(backTowardsFourthPole.end())
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_FAST, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_FAST)
                )
                .build();
        Trajectory backLittleTowardsFifthPole = drive.trajectoryBuilder(forwardTowardsXConeStackFifthPole.end())
                .lineToLinearHeading(new Pose2d(-59*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE)
                )
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsFifthPole = drive.trajectoryBuilder(forwardTowardsXConeStackFifthPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannel*teamColor, yValueForChannelForConeDropOff,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SLOW)
                )
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
        Trajectory parkPositionOneDot = drive.trajectoryBuilder(backTowardsFifthPole.end())
                .lineToLinearHeading(new Pose2d(distanceForOneDot*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SUPER_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SUPER_SLOW)
                )
//                .lineToLinearHeading(new Pose2d(-58, -22,Math.toRadians(185) ))
                .build();
        Trajectory parkPositionTwoDot = drive.trajectoryBuilder(backTowardsFourthPole.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(distanceForTwoDot*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SUPER_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SUPER_SLOW)
                )
//                .lineToLinearHeading(new Pose2d(-38, -22,Math.toRadians(185) ))
                //.strafeTo(new Vector2d(-38, -22 ))
                .build();
        Trajectory parkPositionThreeDot = drive.trajectoryBuilder(backTowardsFourthPole.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(distanceForThreeDot*teamColor, yValueForChannel,newAngleToUse)
                        ,drive.getVelocityConstraint(MAX_VEL_OVERRIDE_SUPER_SLOW, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(MAX_ACCEL_OVERRIDE_SUPER_SLOW) //51, then 50.
                )
//                .lineToLinearHeading(new Pose2d(-13, -22,Math.toRadians(185) ))
                //.strafeTo(new Vector2d(-11, -22 ))
                .build();
        //endregion TRAJECTORIES


/** Wait for the game to begin */

/*        drive.redLED1.setMode(DigitalChannel.Mode.OUTPUT);
        drive.redLED2.setMode(DigitalChannel.Mode.OUTPUT);
        drive.greenLED1.setMode(DigitalChannel.Mode.OUTPUT);
        drive.greenLED2.setMode(DigitalChannel.Mode.OUTPUT);
        drive.redLED1.setState(true);
        drive.redLED2.setState(true);
        drive.greenLED1.setState(false);
        drive.greenLED2.setState(false);
        //drive.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/

        telemetry.addData(">", "Ready To Go Teammate. Let's Go THOR 9974!");
        telemetry.update();
/*
        drive.redLED1.setState(false);
        drive.redLED2.setState(false);
        drive.greenLED1.setState(true);
        drive.greenLED2.setState(true);
*/
        VuforiaStuff2023.sleeveSignalDetectedData posData = null;
        //drive.SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VuforiaStuff2023.sleeveSignal pos = null;
        claw.CloseClaw();
        turret.turretGoHome();
        turret.update();
        ElapsedTime timerForFullAutoCycle = new ElapsedTime();
        waitForStart();
        timerForFullAutoCycle.reset();
        //drive.ArmLifter(0,4);
        linearSlide.moveToMiddlePoleForFirstPole();
        if(teamColor <0) {
            rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
        }
        else
        {
            rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);
        }
        linearSlide.update();
        rotatingArm.update();
        currentState = State.raiseTheLinearSlide;
        posData = vuforiaStuff.vuforiascan(true, true,false,teamColor);
        double distanceToDropOffSkystone = 0;
        double distanceBackToCenterLine = 0;
        double distanceBackToSecondStone = 0;
        boolean turnOnlyOneAtIntake = false;
        pos = posData.sleeveSignalDetected;


        if (opModeIsActive()) {

            telemetry.addData("Position", pos);
            telemetry.addData("OneDotCount", posData.oneDotCount);
            telemetry.addData("TwoDotsCount", posData.twoDotsCount);
            telemetry.addData("ThreeDotsCount", posData.threeDotsCount);


            telemetry.update();
        }

        if (isStopRequested()) return;
        //sleep(200); //OG is 300
        //linearSlide.moveToLowPole();
        /*
        linearSlide.moveToMiddlePoleForFirstPole();
        if(teamColor <0) {
            rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
        }
        else
        {
            rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);

        }
        linearSlide.update();
        rotatingArm.update();
        currentState = State.raiseTheLinearSlide;
*/


        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        int levelArmShouldGoTo = 0;
        //  pos = VuforiaStuff2023.sleeveSignal.ONEDOT;

        ElapsedTime timer = new ElapsedTime();
        double timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = 425; //450 //500
        double timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForShortDistance = 200;
        double timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForLongDistance = 400;
        double timeToWaitBeforeBringingRotatingArmDownBetweenCycles = 100; // 150
        double timeToWaitBeforeBringingSlideDownToRightLevel = 210; // 565
        double timeToWaitBeforeMovingAtStart = 235; //215 //225//215 //50//75
        double timeToWaitBeforeBringingRotatingArmDownBetweenCyclesForEnd = 300;
        double timeToWaitBeforeBringingRotatingArmUpAfterPickup = 50;//350
        double timeToCloseClawForPickup = 130;//205//225
        double timeToCloseClawForPickupForLastCone = 175;//150
        boolean haveWeUsedTimerOnceAlready = false;
        boolean haveWekickedOffLinearSlideAlreadyInThisCycle = false;
        timer.reset();
        haveWeUsedTimerOnceAlready=false;
        haveWekickedOffLinearSlideAlreadyInThisCycle=false;

        while (opModeIsActive() && !isStopRequested()) {
            turret.update();
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
//region States For Auto
                case raiseTheLinearSlide:
                    if(timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeMovingAtStart)
                    {
                        drive.followTrajectoryAsync(initialForwardTowardsFirstPole);
                        currentState = State.initalForwardTowardsFirstPole;
                        timer.reset();
                        haveWeUsedTimerOnceAlready=false;
                    }
                    break;
                case initalForwardTowardsFirstPole:

                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy())// && !drive.IsArmLifterBusy())
                    {
                        currentState = State.placeTheConeInFirstPole;

                    }

                    break;

                case placeTheConeInFirstPole:
                    if(claw.clawMode == Claw.statesWithinClaw.IDLE)
                    {
                        claw.clawMode = Claw.statesWithinClaw.linearSlideDown;
                    }
                    claw.OpenClawWithLinearSlideWithEarlyDrop(linearSlide.subtractionForLowerCalcualtedHeightForTurretAutonomous);
                    if(claw.clawMode == Claw.statesWithinClaw.clawOpenDone) {
                        //sleep(300);
                        currentState = State.forwardOverShootForSignal;
                        claw.clawMode = Claw.statesWithinClaw.IDLE;
                        //turret.turretGoHome(teamColor);
                        drive.followTrajectoryAsync(forwardToOverShootSignalCone);
                        timer.reset();
                        haveWeUsedTimerOnceAlready=false;
                    }
                    break;
                case forwardOverShootForSignal:
                    if(!drive.isBusy())
                    {
                        drive.followTrajectoryAsync(forwardTowardsYConeStack);
                        currentState = State.forwardTowardsParking;
                    }
                    else
                    {
                        if((timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownInMilliSeconds)
                        && !haveWeUsedTimerOnceAlready)
                        {
                            rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                            linearSlide.moveToLevel5ConeStack();
                            haveWeUsedTimerOnceAlready=true;
                        }
                    }
                    break;
                case forwardTowardsParking:
                    if(!drive.isBusy())
                    {
                        currentState = State.forwardTowardsYConeStack;
                    }
                    break;
                case forwardTowardsYConeStack:
                    //if(!linearSlide.isBusy() )
                    //{
                        currentState = State.forwardTowardsXConeStack;
                        drive.turn(Math.toRadians(90*teamColor));
                        drive.followTrajectoryAsync(forwardTowardsXConeStack);
                        turret.turretGoHome();
                        turret.update();
                    //}
                    break;
                case forwardTowardsXConeStack:
                    switch(counterForAutoCycle) {
                        case 1:
                            if (!drive.isBusy() && !turret.isBusy()) {
                                currentState = State.pickupSecondCone;
                                claw.CloseClaw();
                                sleep((long) timeToCloseClawForPickup);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();

                            }
                             break;

                        case 2:
                            if(!rotatingArm.isBusy() && !linearSlide.isBusy() && !turret.isBusy())
                            {
                                currentState = State.bringDownTheSlideToCorrectLevelAndCloseClaw; // pickupSecondCone
                                //sleep(75);
                               // currentState = State.pickupSecondCone; // pickupSecondCone
                                //linearSlide.moveToLevel4ConeStack();

                            }
                            else
                            {
                                if((timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownBetweenCycles)
                                && !haveWeUsedTimerOnceAlready)
                                {
                                    rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                                    if(turret.isItInRangeToBringLiftDownDuringTurretAuto())
                                    {
                                        linearSlide.moveToLevel4ConeStack();
                                        haveWeUsedTimerOnceAlready=true;
                                        haveWekickedOffLinearSlideAlreadyInThisCycle=false;
                                    }
                                    else {
                                        if(!haveWekickedOffLinearSlideAlreadyInThisCycle) {
                                            linearSlide.moveToLevelToRaiseTheConeFromStack(); // move to level4 cone stack.
                                            haveWekickedOffLinearSlideAlreadyInThisCycle = true;
                                        }
                                    }
                                }

                            }
                            break;
                        case 3:
                            if(!rotatingArm.isBusy() && !linearSlide.isBusy() && !turret.isBusy())
                            {
                                currentState = State.bringDownTheSlideToCorrectLevelAndCloseClaw; //pickupSecondCone
                                //sleep(75);
                                //linearSlide.moveToLevel3ConeStack();
                            }
                            else
                            {
                                if((timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownBetweenCycles)
                                    && !haveWeUsedTimerOnceAlready)
                                {
                                    rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                                    if(turret.isItInRangeToBringLiftDownDuringTurretAuto())
                                    {
                                        linearSlide.moveToLevel3ConeStack();
                                        haveWeUsedTimerOnceAlready=true;
                                        haveWekickedOffLinearSlideAlreadyInThisCycle=false;
                                    }
                                    else {
                                        if(!haveWekickedOffLinearSlideAlreadyInThisCycle) {
                                            linearSlide.moveToLevelToRaiseTheConeFromStack(); // move to level4 cone stack.
                                            haveWekickedOffLinearSlideAlreadyInThisCycle = true;
                                        }
                                    }
                                }
                            }
                            break;
                        case 4:
                            if(!rotatingArm.isBusy() && !linearSlide.isBusy() && !turret.isBusy())
                            {
                                currentState = State.bringDownTheSlideToCorrectLevelAndCloseClaw; //pickupSecondCone
                                sleep(25);//50
                                linearSlide.moveToLevel2ConeStack();
                            }
                            else
                            {
                                if((timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownBetweenCycles)
                                    && !haveWeUsedTimerOnceAlready)
                                {
                                    haveWeUsedTimerOnceAlready=true;
                                    rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                                    //if(turret.isItInRangeToBringLiftDownDuringTurretAuto())
                                    //{
                                    //    linearSlide.moveToLevel2ConeStack();
                                    //}
                                    //else {
                                        linearSlide.moveToLevelToRaiseTheConeFromStack(); // move to level4 cone stack.
                                    //}
                                }
                            }
                            break;
                    }
                    break;
                case bringDownTheSlideToCorrectLevelAndCloseClaw:
                    switch(counterForAutoCycle) {
                        case 1:
                            if (!drive.isBusy() && !turret.isBusy()) {
                                currentState = State.pickupSecondCone;
                            }
                            break;

                        case 2:
                            if(!drive.isBusy() && !linearSlide.isBusy())
                            {
                                currentState = State.pickupSecondCone; // pickupSecondCone

                                // currentState = State.pickupSecondCone; // pickupSecondCone
                                claw.CloseClaw();
                                sleep((long) timeToCloseClawForPickup);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();

                            }
                            break;
                        case 3:
                            if(!drive.isBusy() && !linearSlide.isBusy())
                            {
                                currentState = State.pickupSecondCone; //pickupSecondCone
                                claw.CloseClaw();
                                sleep((long) timeToCloseClawForPickup);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();
                            }
                            break;
                        case 4:
                            if(!drive.isBusy() && !linearSlide.isBusy())
                            {
                                currentState = State.pickupSecondCone; //pickupSecondCone
                                claw.CloseClaw();
                                sleep((long) timeToCloseClawForPickupForLastCone);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();
                            }
                            break;
                    }
                    break;
                case pickupSecondCone:
                    if(!linearSlide.isBusy())
                    {
                        currentState = State.backLittleTowardsSecondPole;
                        timer.reset();
                        haveWeUsedTimerOnceAlready=false;
                        switch(counterForAutoCycle) {
                            case 1:
                                drive.followTrajectoryAsync(backTowardsSecondPole);
                                break;
                            case 2:
                                drive.followTrajectoryAsync(backTowardsThirdPole);
                                break;
                            case 3:
                                drive.followTrajectoryAsync(backTowardsFourthPole);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(backTowardsFifthPole);
                                break;
                        }

                    }
                    break;
                case backLittleTowardsSecondPole:
                    if(!drive.isBusy()&&!linearSlide.isBusy()&&!rotatingArm.isBusy()&&!turret.isBusy())
                    {
                        currentState = State.backTowardsSecondPole;
                    }
                    else
                    {
                        if((timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmUpAfterPickup)
                            && !haveWeUsedTimerOnceAlready)
                        {
                            haveWeUsedTimerOnceAlready=true;
                            switch(counterForAutoCycle)
                            {
                                case 1:
                                    //drive.followTrajectoryAsync(backTowardsSecondPole);
                                    linearSlide.moveToMiddlePole();
                                    if (teamColor < 0) {
                                        turret.turretGoRight();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT_75);
                                    } else {
                                        turret.turretGoLeft();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT_75);
                                    }
                                    break;
                                case 2:
                                    //drive.followTrajectoryAsync(backTowardsThirdPole);
                                    linearSlide.moveToMiddlePole();
                                    if (teamColor < 0) {
                                        turret.turretGoRight();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT_75);
                                    } else {
                                        turret.turretGoLeft();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT_75);
                                    }
                                    break;
                                case 3:
                                    //drive.followTrajectoryAsync(backTowardsFourthPole);
                                    linearSlide.moveToMiddlePole();
                                    if (teamColor < 0) {
                                        turret.turretGoRight();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT_75);
                                    } else {
                                        turret.turretGoLeft();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT_75);
                                    }
                                    break;
                                case 4:
                                    //drive.followTrajectoryAsync(backTowardsFourthPole);
                                    linearSlide.moveToMiddlePole();
                                    if (teamColor < 0) {
                                        turret.turretGoRight();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT_75);
                                    } else {
                                        turret.turretGoLeft();
                                        turret.update();
                                        rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT_75);
                                    }
                                    break;
                            }
                        }
                    }
                    break;
                case backTowardsSecondPole:
                    if(!drive.isBusy() && !linearSlide.isBusy())
                    {
                        currentState = State.placeTheConeInSecondPole;

//                        drive.followTrajectoryAsync(forwardTowardsYConeStack);

                    }
                    break;
                case placeTheConeInSecondPole:
                    if(claw.clawMode == Claw.statesWithinClaw.IDLE)
                    {
                        claw.clawMode = Claw.statesWithinClaw.linearSlideDown;
                    }
                    claw.OpenClawWithLinearSlideWithEarlyDrop(linearSlide.subtractionForLowerCalcualtedHeightForTurretAutonomous);
                    if(claw.clawMode == Claw.statesWithinClaw.clawOpenDone) {

                        //sleep(300);
                        claw.clawMode = Claw.statesWithinClaw.IDLE;
                        switch (counterForAutoCycle) {
                            case 1:
                                counterForAutoCycle += 1;
                                drive.followTrajectoryAsync(forwardTowardsXConeStackThirdPole);
                                turret.turretGoHome();
                                turret.update();
                                currentState = State.forwardTowardsXConeStack;
                                timer.reset();
                                haveWeUsedTimerOnceAlready=false;
                                //sleep here to let turret start before arm and lift move.
                                break;
                            case 2:
                                counterForAutoCycle += 1;
                                drive.followTrajectoryAsync(forwardTowardsXConeStackFourthPole);
                                turret.turretGoHome();
                                turret.update();
                                currentState = State.forwardTowardsXConeStack;
                                timer.reset();
                                haveWeUsedTimerOnceAlready=false;
                                break;
                            case 3:
                                counterForAutoCycle += 1;
                                //If we are running out of time Just go to parking.
                                if(timerForFullAutoCycle.time(TimeUnit.MILLISECONDS) > 24500)
                                {
                                    turret.turretGoHome();
                                    turret.update();
                                    currentState = State.goToTheParkingPosition;
                                    break;
                                }
                                else {
                                    currentState = State.forwardTowardsXConeStack;
                                }
                                drive.followTrajectoryAsync(forwardTowardsXConeStackFifthPole);
                                turret.turretGoHome();
                                turret.update();
                                currentState = State.forwardTowardsXConeStack;
                                timer.reset();
                                haveWeUsedTimerOnceAlready=false;
                                break;
                            case 4:
                                turret.turretGoHome();
                                turret.update();
                                currentState = State.goToTheParkingPosition;
                                break;

                        }
                    }

                    //turret.turretGoHome(teamColor);
                    break;
                case goToTheParkingPosition:
                    currentState = State.lowerTheRotatorAndSlide;
                    //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    if (pos == VuforiaStuff2023.sleeveSignal.ONEDOT) {
                        if(teamColor == 1) {
                            drive.followTrajectoryAsync(parkPositionOneDot);
                            timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForLongDistance;
                        }
                        else
                        {
                            drive.followTrajectoryAsync(parkPositionThreeDot);
                            timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForShortDistance;
                        }

                        //       drive.followTrajectoryAsync(goToBasketTowerLevelMiddle);
                        //     levelArmShouldGoTo = 2;
                        //   //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    }
                    if (pos == VuforiaStuff2023.sleeveSignal.TWODOTS) {
                        drive.followTrajectoryAsync(parkPositionTwoDot);
                        timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownBetweenCyclesForEnd;


                        //        drive.followTrajectoryAsync(goToBasketTowerLevelHigh);
                        //      levelArmShouldGoTo = 4;
                        //    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    }
                    if (pos == VuforiaStuff2023.sleeveSignal.THREEDOTS) {
                        if(teamColor == 1) {
                            drive.followTrajectoryAsync(parkPositionThreeDot);
                            timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForShortDistance;

                        }
                        else
                        {
                            drive.followTrajectoryAsync(parkPositionOneDot);
                            timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForLongDistance;

                        }

                        //     drive.followTrajectoryAsync(goToBasketTowerLevelLow);
                        //   levelArmShouldGoTo = 1;
                        // //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    }
                    timer.reset();
                    haveWeUsedTimerOnceAlready=false;
                    break;
                case lowerTheRotatorAndSlide:
                    if (!drive.isBusy()) {
                        linearSlide.moveToAboveTheCameraHeight();
                        currentState = State.parkPosition;
                        ////drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        // drive.followTrajectoryAsync(goTowardsSecondPoleFirst);
                    }
                    else
                    {
                        if((timer.time(TimeUnit.MILLISECONDS)>timeToWaitBeforeBringingRotatingArmDownInMilliSeconds)
                            && !haveWeUsedTimerOnceAlready)
                        {
                            rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                            linearSlide.moveToAboveTheCameraHeight();
                            haveWeUsedTimerOnceAlready=true;
                        }
                    }
                    break;
                case parkPosition:
                    if (!rotatingArm.isBusy() && !linearSlide.isBusy()) {
                        linearSlide.goToZero();
                        currentState = State.IDLE;
                        ////drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        // drive.followTrajectoryAsync(goTowardsSecondPoleFirst);
                    }
                    break;



//endregion States For Level Middle
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();
            linearSlide.update();
            rotatingArm.update();
            //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("Position", pos);
            telemetry.addData("OneDotCount", posData.oneDotCount);
            telemetry.addData("TwoDotCount", posData.twoDotsCount);
            telemetry.addData("ThreeDotCount", posData.threeDotsCount);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("MagSwitchStateForTurret", turret.magneticLimitSwitch.getState());
            telemetry.addData("TurretPOT", turret.getTurretPotentiometerVoltage());
            telemetry.addData("Full Cycle Timer", timerForFullAutoCycle.time(TimeUnit.MILLISECONDS));

            //telemetry.addData("ArmPosition",drive.SlideMotor.getCurrentPosition());
            telemetry.update();
            if(timerForFullAutoCycle.time(TimeUnit.MILLISECONDS) > 29950)
            {
                drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drive.setMotorPowers(0,0,0,0);
            }
        }
        drive.setMotorPowers(0,0,0,0);
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }
}
