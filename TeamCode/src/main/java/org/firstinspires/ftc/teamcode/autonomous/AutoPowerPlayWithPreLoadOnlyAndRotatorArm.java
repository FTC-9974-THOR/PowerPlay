package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.AdrianControls.Turret;
import org.firstinspires.ftc.teamcode.AdrianControls.VuforiaStuff2023;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive9974;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

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
public class AutoPowerPlayWithPreLoadOnlyAndRotatorArm extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
// region STATES FOR LEVEL MIDDLE
        IDLE,
        raiseTheLinearSlide,
        initalForwardTowardsFirstPole,
        placeTheConeInFirstPole,
        forwardTowardsParking,
        lowerTheRotatorAndSlide,
        parkPosition


    }
    private int teamColor;//1=Left -1= Right
    private boolean slideToSide = false;

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

    public AutoPowerPlayWithPreLoadOnlyAndRotatorArm(int TeamColor, boolean SlideToSide) {
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
        Claw claw = new Claw(hardwareMap,linearSlide);
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
        Pose2d startPose = new Pose2d(-36, -72 , Math.toRadians(90));

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        //region TRAJECTORIES
        Trajectory initialForwardTowardsFirstPole = drive.trajectoryBuilder(startPose)
                //.lineTo(new Vector2d(-36, -64 ))// this is for the low pole.
                .lineTo(new Vector2d(-36, -42 )) // this is for the middle pole.
                .build();
        Trajectory initialForwardToLowerTheRotator = drive.trajectoryBuilder(initialForwardTowardsFirstPole.end())
                .lineTo(new Vector2d(-36, -58 ))
                .build();
        Trajectory forwardTowardsParkingOverShootForSignal = drive.trajectoryBuilder(initialForwardToLowerTheRotator.end())
                .lineTo(new Vector2d(-36, -38.75 ))
                .build();
        Trajectory forwardTowardsParking = drive.trajectoryBuilder(forwardTowardsParkingOverShootForSignal.end())
                .lineTo(new Vector2d(-36, -44.75 ))
                .build();
        Trajectory parkPositionOneDot = drive.trajectoryBuilder(forwardTowardsParking.end())
                .strafeTo(new Vector2d(-60, -44.75 ))
                .build();
        Trajectory parkPositionTwoDot = drive.trajectoryBuilder(forwardTowardsParking.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .strafeTo(new Vector2d(-37, -44.75 ))
                .build();
        Trajectory parkPositionThreeDot = drive.trajectoryBuilder(forwardTowardsParking.end())
                //.splineToConst
                // antHeading(new Vector2d( -65,-72), Math.toRadians(90))
                .strafeTo(new Vector2d(-10, -44.75 ))
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
        sleep(500);
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
                    claw.OpenClaw();
                    sleep(500);
                    currentState = State.forwardTowardsParking;
                    //turret.turretGoHome(teamColor);
                    drive.followTrajectoryAsync(forwardTowardsParking);
                    break;

                case forwardTowardsParking:
                    if(!drive.isBusy())
                    {
                        rotatingArm.setRotatorArmPositionRaw(RotatingArm.ROTATOR_BOTTOM);
                        linearSlide.moveToAboveTheCameraHeight();
                        currentState = State.lowerTheRotatorAndSlide;
                    }
                    break;

                case lowerTheRotatorAndSlide:
                    if(!linearSlide.isBusy() && !rotatingArm.isBusy()) {
                        linearSlide.goToZero();
                        if (pos == VuforiaStuff2023.sleeveSignal.ONEDOT) {
                            currentState = State.parkPosition;
                            drive.followTrajectoryAsync(parkPositionOneDot);

                            //       drive.followTrajectoryAsync(goToBasketTowerLevelMiddle);
                            //     levelArmShouldGoTo = 2;
                            //   //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        }
                        if (pos == VuforiaStuff2023.sleeveSignal.TWODOTS) {
                            currentState = State.parkPosition;
                            drive.followTrajectoryAsync(parkPositionTwoDot);

                            //        drive.followTrajectoryAsync(goToBasketTowerLevelHigh);
                            //      levelArmShouldGoTo = 4;
                            //    //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        }
                        if (pos == VuforiaStuff2023.sleeveSignal.THREEDOTS) {
                            currentState = State.parkPosition;
                            drive.followTrajectoryAsync(parkPositionThreeDot);

                            //     drive.followTrajectoryAsync(goToBasketTowerLevelLow);
                            //   levelArmShouldGoTo = 1;
                            // //drive.ArmLifterAsyncUpdate(levelArmShouldGoTo);
                        }
                    }
                    break;
                case parkPosition:
                    if (!drive.isBusy() && !linearSlide.isLiftAboveCameraHeight()) {
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
