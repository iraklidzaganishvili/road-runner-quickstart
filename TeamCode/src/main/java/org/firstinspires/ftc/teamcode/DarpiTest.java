package fonntRightMotorftrrg.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.ParallelAction;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;

@Autonomous(name="DarpiTest", group="LinearOpMode")

public abstract class DarpiTest extends Line
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
arOpMode{
private DcMotor leftDrive = null;
private DcMoto rightDrive = null;



}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.actions.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveExample {

    public MecanumDriveExample(HardwareMap hardwareMap, Pose2d initialPose) {
        // Initialize the MecanumDrive with the robot's hardware map and starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(initialPose);
    }

    // Define an action to drive forward in the autonomous routine
    public Action driveForward(double distance) {
        // Define a trajectory action to move forward by a specified distance
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Move the robot forward
                drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(distance)
                        .build()
                );
                return true;  // Action has been completed
            }
        };
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.actions.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveExample {

    public MecanumDriveExample(HardwareMap hardwareMap, Pose2d initialPose) {
        // Initialize the MecanumDrive with the robot's hardware map and starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(initialPose);
    }

    // Define an action to drive forward in the autonomous routine
    public Action driveForward(double distance) {
        // Define a trajectory action to move forward by a specified distance
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Move the robot forward
                drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(distance)
                        .build()
                );
                return true;  // Action has been completed
            }
        };
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.actions.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveExample {

    public MecanumDriveExample(HardwareMap hardwareMap, Pose2d initialPose) {
        // Initialize the MecanumDrive with the robot's hardware map and starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(initialPose);
    }

    // Define an action to drive forward in the autonomous routine
    public Action driveForward(double distance) {
        // Define a trajectory action to move forward by a specified distance
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // Move the robot forward
                drive.followTrajectory(
                    drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(distance)
                        .build()
                );
                return true;  // Action has been completed
            }
        };
    }
}
package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {}

public class DarpiTest extends Autonomous {


}public class Lift {
    private DcMotorEx lift;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}

// claw class
public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
    }
}