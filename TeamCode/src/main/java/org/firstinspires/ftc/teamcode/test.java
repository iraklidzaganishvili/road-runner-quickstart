package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "MoveRightAuto", group = "Autonomous")
public class MoveRightAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Define the initial pose (starting position)
        Pose2d initialPose = new Pose2d(0, 0, 0); // Starting at origin, facing 0 degrees

        // Create the MecanumDrive instance
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Wait for the game to start
        waitForStart();

        if (isStopRequested()) return;

        // Create a trajectory to move the robot to the right by 20 units
        drive.followTrajectory(
                drive.trajectoryBuilder(initialPose)
                        .strafeRight(20) // Move 20 units to the right
                        .build()
        );

        // You can add additional actions here if needed

        telemetry.addData("Status", "Autonomous completed");
        telemetry.update();
    }
}
