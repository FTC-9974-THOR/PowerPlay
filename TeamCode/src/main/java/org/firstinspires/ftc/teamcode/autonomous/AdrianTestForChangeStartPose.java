/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AdrianControls.LinearSlidePIDWithVelocity;
import org.firstinspires.ftc.teamcode.AdrianControls.RotatingArm;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive9974;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AdrianTestForChangeStartPose", group="Linear Opmode")
@Disabled
public class AdrianTestForChangeStartPose extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //Encoder Int Values
    private int RightEncoderValue;
    private int LeftEncoderValue;
    private int SideEncoderValue;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Initialize Hardware( see AdrianMecanumControls)
        MecanumDrive9974 drive = new MecanumDrive9974(hardwareMap);
        double DISTANCE = 50;
        waitForStart();

        Pose2d startPose = new Pose2d(0,0,0);
        drive.getLocalizer().setPoseEstimate(startPose);
        Trajectory trajectoryForward = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10,0,0))
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .lineToLinearHeading(new Pose2d(0,0,0))
                .build();

        Pose2d startPose2 = new Pose2d(10,10,Math.toRadians(90));
        Trajectory trajectoryForward2 = drive.trajectoryBuilder(startPose2)
                .lineToLinearHeading(new Pose2d(10,25,Math.toRadians(90)))
                .build();

        Trajectory trajectoryBackward2 = drive.trajectoryBuilder(trajectoryForward2.end())
                .lineToLinearHeading(new Pose2d(10,10,Math.toRadians(90)))
                .build();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            drive.getLocalizer().setPoseEstimate(startPose);
            drive.followTrajectory(trajectoryForward);
            telemetry.addData("x",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.addData("y",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.update();
            drive.followTrajectory(trajectoryBackward);
            telemetry.addData("x",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.addData("y",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.update();
            sleep(2000);
            drive.getLocalizer().setPoseEstimate(startPose2);
            drive.followTrajectory(trajectoryForward2);
            telemetry.addData("x",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.addData("y",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.update();
            drive.followTrajectory(trajectoryBackward2);
            telemetry.addData("x",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.addData("y",drive.getLocalizer().getPoseEstimate().getX());
            telemetry.update();
            sleep(2000);
                 }
        }

    }

