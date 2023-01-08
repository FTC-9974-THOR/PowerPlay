package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.AdrianControls.Claw;
import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;
import org.firstinspires.ftc.teamcode.AdrianControls.RotatingArm;
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
@Autonomous(group = "advanced")
@Disabled
public class AutoPowerPlayWithFiveConesAndRotatorArm extends LinearOpMode {

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

    private double yValueForChannel = 0.0;
    private double yValueForChannelLeft = -20.3;
    private double yValueForChannelRight = -19.2;
    private double yValueForChannelHighPole = 0.0;
    private double yValueForChannelHighPoleLeft = -21.5;
    private double yValueForChannelHighPoleRight = -20;
    private double yValueForChannelForConeDropOff = 0.0;
    private double yValueForChannelForConeDropOffLeft = -21;
    private double yValueForChannelForConeDropOffRight = -20.0;

    private double xValueForwardTowardsXConeStack = 0.0;
    private double xValueForwardTowardsXConeStackLeftClaw1 = -59.7;
    private double xValueForwardTowardsXConeStackRightClaw1 = -60.25;
    private double xValueForwardTowardsXConeStackLeftClaw2 = -59.5;
    private double xValueForwardTowardsXConeStackRightClaw2 = -59.7;
    private double xValueForChannel = 0.0;
    private double xValueForChannelLeft = -14.0;
    private double xValueForChannelRight = -15.0;
    private double xValueForChannelShortPole = 0.0;
    private double xValueForChannelShortPoleLeft = -36.75;
    private double xValueForChannelShortPoleRight = -38.25;
    public AutoPowerPlayWithFiveConesAndRotatorArm(int TeamColor, boolean SlideToSide) {
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
        LinearSlidePIDWithVelocity linearSlide = new LinearSlidePIDWithVelocity(hardwareMap);
        Claw claw = new Claw(hardwareMap, linearSlide);
        //Turret turret = new Turret(hardwareMap);
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
        Pose2d startPose = new Pose2d(-36*teamColor, -72 , Math.toRadians(90));
        if(teamColor ==1)
        {
            yValueForChannel = yValueForChannelLeft;
            yValueForChannelHighPole = yValueForChannelHighPoleLeft;
            yValueForChannelForConeDropOff = yValueForChannelForConeDropOffLeft;
            xValueForChannel = xValueForChannelLeft;
            xValueForChannelShortPole = xValueForChannelShortPoleLeft;
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
            xValueForChannelShortPole = xValueForChannelShortPoleRight;
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
                //.lineTo(new Vector2d(-36, -64 ))// this is for the low pole.
                .lineToLinearHeading(new Pose2d(-36*teamColor, -41.4 ,Math.toRadians(90))) // this is for the middle pole.
                .build();
        Trajectory initialForwardToLowerTheRotator = drive.trajectoryBuilder(initialForwardTowardsFirstPole.end())
                .lineToLinearHeading(new Pose2d(-36*teamColor, -39 ,Math.toRadians(90)))
                .build();
        Trajectory forwardToOverShootSignalCone = drive.trajectoryBuilder(initialForwardTowardsFirstPole.end())
                .lineToLinearHeading(new Pose2d(-36*teamColor, -12 ,Math.toRadians(90)))
                .build();
        Trajectory forwardTowardsYConeStack = drive.trajectoryBuilder(forwardToOverShootSignalCone.end())
                .lineToLinearHeading(new Pose2d(-36*teamColor, yValueForChannel,Math.toRadians(90)))
                .build();
        double newAngleToUse = Math.toRadians(90)+Math.toRadians(90*teamColor);
        Trajectory forwardTowardsXConeStack = drive.trajectoryBuilder(forwardTowardsYConeStack.end().plus(new Pose2d(0,0,Math.toRadians(90*teamColor))))
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse))
                .build();
//region Second Pole
        Trajectory backLittleTowardsSecondPole = drive.trajectoryBuilder(forwardTowardsXConeStack.end())
                .lineToLinearHeading(new Pose2d(-57*teamColor, yValueForChannel,newAngleToUse))
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsSecondPole = drive.trajectoryBuilder(backLittleTowardsSecondPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannelShortPole*teamColor, yValueForChannelForConeDropOff,newAngleToUse))
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
//region Third Pole        
        Trajectory forwardTowardsXConeStackThirdPole = drive.trajectoryBuilder(backTowardsSecondPole.end())
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse))
                .build();
        Trajectory backLittleTowardsThirdPole = drive.trajectoryBuilder(forwardTowardsXConeStackThirdPole.end())
                .lineToLinearHeading(new Pose2d(-57*teamColor, yValueForChannel ,newAngleToUse))
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsThirdPole = drive.trajectoryBuilder(backLittleTowardsThirdPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannelShortPole*teamColor, yValueForChannelForConeDropOff,newAngleToUse))
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
//region Fourth Pole        
        Trajectory forwardTowardsXConeStackFourthPole = drive.trajectoryBuilder(backTowardsThirdPole.end())
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse))
                .build();
        Trajectory backLittleTowardsFourthPole = drive.trajectoryBuilder(forwardTowardsXConeStackFourthPole.end())
                .lineToLinearHeading(new Pose2d(-57*teamColor, yValueForChannel ,newAngleToUse))
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsFourthPole = drive.trajectoryBuilder(backLittleTowardsFourthPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannel*teamColor, yValueForChannelForConeDropOff,newAngleToUse)
//                        ,drive.getVelocityConstraint(30.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(30.0)
                )
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
//region Fifth Pole
        Trajectory forwardTowardsXConeStackFifthPole = drive.trajectoryBuilder(backTowardsFourthPole.end())
                .lineToLinearHeading(new Pose2d(xValueForwardTowardsXConeStack*teamColor, yValueForChannel,newAngleToUse))
                .build();
        Trajectory backLittleTowardsFifthPole = drive.trajectoryBuilder(forwardTowardsXConeStackFifthPole.end())
                .lineToLinearHeading(new Pose2d(-57*teamColor, yValueForChannel ,newAngleToUse))
//                .lineToLinearHeading(new Pose2d(-58, -22.5,Math.toRadians(185) ))
                .build();
        Trajectory backTowardsFfifthPole = drive.trajectoryBuilder(backLittleTowardsFifthPole.end())
                .lineToLinearHeading(new Pose2d(xValueForChannel*teamColor, yValueForChannelForConeDropOff,newAngleToUse)
//                        ,drive.getVelocityConstraint(30.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        drive.getAccelerationConstraint(30.0)
                )
//                .lineToLinearHeading(new Pose2d(-50.25, -22.5,Math.toRadians(185) ))
                .build();
//endregion
        
        Trajectory parkPositionOneDot = drive.trajectoryBuilder(backTowardsFfifthPole.end())
                .lineToLinearHeading(new Pose2d(-58*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(57.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(57.0)
                )
//                .lineToLinearHeading(new Pose2d(-58, -22,Math.toRadians(185) ))
                .build();
        Trajectory parkPositionTwoDot = drive.trajectoryBuilder(backTowardsFourthPole.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-35*teamColor, yValueForChannel ,newAngleToUse)
                        ,drive.getVelocityConstraint(51.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50.0)
                )
//                .lineToLinearHeading(new Pose2d(-38, -22,Math.toRadians(185) ))
                //.strafeTo(new Vector2d(-38, -22 ))
                .build();
        Trajectory parkPositionThreeDot = drive.trajectoryBuilder(backTowardsFourthPole.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-11*teamColor, yValueForChannel,newAngleToUse)
                        ,drive.getVelocityConstraint(51.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50.0)
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

        waitForStart();
        //drive.ArmLifter(0,4);
        posData = vuforiaStuff.vuforiascan(true, true,false,teamColor);
        double distanceToDropOffSkystone = 0;
        double distanceBackToCenterLine = 0;
        double distanceBackToSecondStone = 0;
        boolean turnOnlyOneAtIntake = false;
        pos = posData.sleeveSignalDetected;


        if (opModeIsActive()) {
/*
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.time() < 2.0) {
                telemetry.addData("Position", pos);
                telemetry.addData("LeftYellowCount", posData.yellowCountLeft);
                telemetry.addData("CenterYellowCount", posData.yellowCountCenter);
                telemetry.addData("RightYellowCount", posData.yellowCountRight);


                telemetry.update();
            }
  */
            telemetry.addData("Position", pos);
            telemetry.addData("OneDotCount", posData.oneDotCount);
            telemetry.addData("TwoDotsCount", posData.twoDotsCount);
            telemetry.addData("ThreeDotsCount", posData.threeDotsCount);


            telemetry.update();
        }

        if (isStopRequested()) return;
        claw.CloseClaw();
        sleep(300);
        //linearSlide.moveToLowPole();
        linearSlide.moveToMiddlePole();
        if(teamColor <0) {
            rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
        }
        else
        {
            rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);

        }
        currentState = State.raiseTheLinearSlide;



        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        int levelArmShouldGoTo = 0;
        //  pos = VuforiaStuff2023.sleeveSignal.ONEDOT;

        ElapsedTime timer = new ElapsedTime();
        double timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = 500;
        double timeToWaitBeforeBringingRotatingArmDownInMilliSecondsForShortDistance = 200;
        double timeToWaitBeforeBringingRotatingArmDownBetweenCycles = 565;
        double timeToWaitBeforeBringingRotatingArmDownBetweenCyclesForEnd = 525;


        while (opModeIsActive() && !isStopRequested()) {

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
                    if(!linearSlide.isBusy() && !rotatingArm.isBusy())
                    {
                        drive.followTrajectoryAsync(initialForwardTowardsFirstPole);
                        currentState = State.initalForwardTowardsFirstPole;


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
                    claw.OpenClawWithLinearSlide();
                    if(claw.clawMode == Claw.statesWithinClaw.clawOpenDone) {
                        //sleep(300);
                        currentState = State.forwardOverShootForSignal;
                        claw.clawMode = Claw.statesWithinClaw.IDLE;
                        //turret.turretGoHome(teamColor);
                        drive.followTrajectoryAsync(forwardToOverShootSignalCone);
                        timer.reset();
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
                        if(timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownInMilliSeconds)
                        {
                            rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                            linearSlide.moveToLevel5ConeStack();
                        }
                    }
                    break;
                case forwardTowardsParking:
                    if(!drive.isBusy()&& !rotatingArm.isBusy())
                    {
                        currentState = State.forwardTowardsYConeStack;
                    }
                    break;
                case forwardTowardsYConeStack:
                    if(!linearSlide.isBusy() )
                    {
                        currentState = State.forwardTowardsXConeStack;
                        drive.turn(Math.toRadians(90*teamColor));
                        drive.followTrajectoryAsync(forwardTowardsXConeStack);
                    }
                    break;
                case forwardTowardsXConeStack:
                    switch(counterForAutoCycle) {
                        case 1:
                            if (!drive.isBusy()) {
                                currentState = State.pickupSecondCone;
                                claw.CloseClaw();
                                sleep(300);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();

                            }
                             break;

                        case 2:
                            if(!drive.isBusy() && !rotatingArm.isBusy() && !linearSlide.isBusy())
                            {
                                currentState = State.pickupSecondCone;
                                claw.CloseClaw();
                                sleep(300);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();
                            }
                            else
                            {
                                if(timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownBetweenCycles)
                                {
                                    rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                                    linearSlide.moveToLevel4ConeStack();
                                }
                            }
                            break;
                        case 3:
                            if(!drive.isBusy() && !rotatingArm.isBusy() && !linearSlide.isBusy())
                            {
                                currentState = State.pickupSecondCone;
                                claw.CloseClaw();
                                sleep(400);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();
                            }
                            else
                            {
                                if(timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownBetweenCycles)
                                {
                                    rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                                    linearSlide.moveToLevel3ConeStack();
                                }
                            }
                            break;
                        case 4:
                            if(!drive.isBusy() && !rotatingArm.isBusy() && !linearSlide.isBusy())
                            {
                                currentState = State.pickupSecondCone;
                                claw.CloseClaw();
                                sleep(300);
                                linearSlide.moveToLevelToRaiseTheConeFromStack();
                            }
                            else
                            {
                                if(timer.time(TimeUnit.MILLISECONDS) > timeToWaitBeforeBringingRotatingArmDownBetweenCycles)
                                {
                                    rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                                    linearSlide.moveToLevel2ConeStack();
                                }
                            }
                            break;                    }
                    break;
                case pickupSecondCone:
                    if(!linearSlide.isBusy())
                    {
                        currentState = State.backLittleTowardsSecondPole;
                        switch(counterForAutoCycle) {
                            case 1:
                                drive.followTrajectoryAsync(backLittleTowardsSecondPole);
                                break;
                            case 2:
                                drive.followTrajectoryAsync(backLittleTowardsThirdPole);
                                break;
                            case 3:
                                drive.followTrajectoryAsync(backLittleTowardsFourthPole);
                                break;
                            case 4:
                                drive.followTrajectoryAsync(backLittleTowardsFifthPole);
                                break;                        }

                    }
                    break;
                case backLittleTowardsSecondPole:
                    if(!drive.isBusy())
                    {
                        currentState = State.backTowardsSecondPole;
                        switch(counterForAutoCycle) {
                            case 1:
                                drive.followTrajectoryAsync(backTowardsSecondPole);
                                linearSlide.moveToLowPole();
                                if(teamColor <0) {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);
                                }
                                else
                                {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
                                }
                                break;
                            case 2:
                                drive.followTrajectoryAsync(backTowardsThirdPole);
                                linearSlide.moveToLowPole();
                                if(teamColor <0) {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);
                                }
                                else
                                {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
                                }
                                break;
                            case 3:
                                drive.followTrajectoryAsync(backTowardsFourthPole);
                                linearSlide.moveToMiddlePole();
                                if(teamColor <0) {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);
                                }
                                else
                                {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
                                }
                                break;
                            case 4:
                                drive.followTrajectoryAsync(backLittleTowardsFifthPole);
                                linearSlide.moveToMiddlePole();
                                if(teamColor <0) {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_RIGHT);
                                }
                                else
                                {
                                    rotatingArm.setRotatorArmPositionRaw(rotatingArm.ROTATOR_LEFT);
                                }
                                break;

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
                    if(counterForAutoCycle == 1 || counterForAutoCycle == 2)
                    {
                        claw.OpenClaw();
                        sleep(200);
                        claw.clawMode = Claw.statesWithinClaw.clawOpenDone;
                    }
                    else
                    {
                        if(claw.clawMode == Claw.statesWithinClaw.IDLE)
                        {
                            claw.clawMode = Claw.statesWithinClaw.linearSlideDown;
                        }
                        claw.OpenClawWithLinearSlide();
                    }

                    if(claw.clawMode == Claw.statesWithinClaw.clawOpenDone) {

                        //sleep(300);
                        claw.clawMode = Claw.statesWithinClaw.IDLE;
                        switch (counterForAutoCycle) {
                            case 1:
                                counterForAutoCycle += 1;
                                drive.followTrajectoryAsync(forwardTowardsXConeStackThirdPole);
                                currentState = State.forwardTowardsXConeStack;
                                timer.reset();
                                break;
                            case 2:
                                counterForAutoCycle += 1;
                                drive.followTrajectoryAsync(forwardTowardsXConeStackFourthPole);
                                currentState = State.forwardTowardsXConeStack;
                                timer.reset();
                                break;
                            case 3:
                                counterForAutoCycle += 1;
                                drive.followTrajectoryAsync(forwardTowardsXConeStackFifthPole);
                                currentState = State.forwardTowardsXConeStack;
                                timer.reset();
                                break;
                            case 4:
                                currentState = State.goToTheParkingPosition;
                                break;

                        }
                    }

                    //turret.turretGoHome(teamColor);
                    break;
                case goToTheParkingPosition:
                    currentState = State.lowerTheRotatorAndSlide;

                    if (pos == VuforiaStuff2023.sleeveSignal.ONEDOT) {
                        if(teamColor == 1) {
                            drive.followTrajectoryAsync(parkPositionOneDot);
                            timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownBetweenCyclesForEnd;
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
                            timeToWaitBeforeBringingRotatingArmDownInMilliSeconds = timeToWaitBeforeBringingRotatingArmDownBetweenCyclesForEnd;

                        }

                        //     drive.followTrajectoryAsync(goToBasketTowerLevelLow);
                        //   levelArmShouldGoTo = 1;
                        // //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                    }
                    timer.reset();
                    break;
                case lowerTheRotatorAndSlide:
                    if (!drive.isBusy() && !rotatingArm.isBusy()&& !linearSlide.isBusy()) {
                        linearSlide.moveToAboveTheCameraHeight();
						currentState = State.parkPosition;
                        ////drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        // drive.followTrajectoryAsync(goTowardsSecondPoleFirst);
                    }
                    else
                    {
                        if(timer.time(TimeUnit.MILLISECONDS)>timeToWaitBeforeBringingRotatingArmDownInMilliSeconds)
                        {
                            rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                            linearSlide.moveToAboveTheCameraHeight();

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
            //telemetry.addData("ArmPosition",drive.SlideMotor.getCurrentPosition());
            telemetry.update();
        }
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
