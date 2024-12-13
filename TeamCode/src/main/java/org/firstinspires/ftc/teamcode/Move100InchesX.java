package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "Move100InchesX", group = "Autonomous")
public class Move100InchesX extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Initialize the        starting pose at the origin facing forward
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Build the trajectory to move 100 inches along the x-axis
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose).lineToX(48).strafeTo(new Vector2d(44.5, 30)).waitSeconds(3);
        Action traj = tab1.endTrajectory().strafeTo(new Vector2d(48, 12)).build();

        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen = tab1.build();
        // Execute the trajectory
        Actions.runBlocking(new SequentialAction(traj, trajectoryActionChosen));

        // Indicate completion
        telemetry.addData("Status", "Movement complete");
        telemetry.update();
    }
}
