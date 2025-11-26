package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="BlueWall")
public class RedWall extends LinearOpMode {
    Servo trigger;
    DcMotor belt;
    DcMotor intake;
    DcMotor LShooter;
    DcMotor RShooter;

    public class Shoot implements InstantFunction {
        float targetPosition;

        public Shoot(float targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public void run() {
            trigger.setPosition(targetPosition);
            sleep(500);
            trigger.setPosition(0);
            belt.setPower(0.8);
            sleep(1000);
            belt.setPower(0.0);
            trigger.setPosition(targetPosition);
            sleep(500);
            trigger.setPosition(0);
            belt.setPower(0.8);
            sleep(1000);
            belt.setPower(0.0);
            trigger.setPosition(targetPosition);
            sleep(500);
            trigger.setPosition(0);
            belt.setPower(0.8);
            sleep(1000);
            belt.setPower(0.0);
        }
    }

    public class Intake implements InstantFunction {
        boolean wheelOn;

        public Intake(Boolean wheelOn) {
            this.wheelOn = wheelOn;
        }

        @Override
        public void run() {
            if (wheelOn) {
                intake.setPower(1.0);
            } else if (!wheelOn) {
                intake.setPower(0.0);
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        trigger = hardwareMap.get(Servo.class, "trigger");
        belt = hardwareMap.get(DcMotor.class, "belt");
        intake = hardwareMap.get(DcMotor.class, "intake");
        LShooter = hardwareMap.get(DcMotor.class, "LShooter");
        RShooter = hardwareMap.get(DcMotor.class, "RShooter");

        belt.setDirection(DcMotorSimple.Direction.REVERSE);
        LShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        RShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        Pose2d beginPose = new Pose2d(new Vector2d(22, -63), Math.toRadians(1.5708));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        LShooter.setPower(1.0);
        RShooter.setPower(1.0);

        Action path = drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(-12, 12), Math.toRadians(135))
                .stopAndAdd(new Shoot(5))
                .strafeToSplineHeading(new Vector2d(11.5, -31), Math.toRadians(270))
                .stopAndAdd(new Intake(true))
                .waitSeconds(1)
                .lineToY(-44)
                .stopAndAdd(new Intake(false))
                .strafeToSplineHeading(new Vector2d(-12, 12), Math.toRadians(135))
                .stopAndAdd(new Shoot(5))
                .build();

        Actions.runBlocking(new SequentialAction(path));
    }
}
