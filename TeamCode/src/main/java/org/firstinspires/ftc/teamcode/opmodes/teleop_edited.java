package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp(name="teleop_edited")
public class teleop_edited extends LinearOpMode {

    private DcMotor FR, FL, BR, BL;
    private IMU imu;

    private vision visionSub;
    private turret turretSub;
    private intake intakeSub;
    private launch launchSub;

    private boolean intakeOn = false;
    private boolean prevB = false;
    private boolean prevX = false;

    @Override
    public void runOpMode() {

        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        intakeSub = new intake(hardwareMap);
        launchSub = new launch(hardwareMap, intakeSub);

        visionSub = new vision(hardwareMap);
        turretSub = new turret(
                hardwareMap.get(Servo.class,"leftServo"),
                hardwareMap.get(Servo.class,"rightServo")
        );

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX *= 1.1;

            double d = Math.max(Math.abs(rotX)+Math.abs(rotY)+Math.abs(rx),1);
            FL.setPower((rotY + rotX + rx)/d);
            BL.setPower((rotY - rotX + rx)/d);
            FR.setPower((rotY - rotX - rx)/d);
            BR.setPower((rotY + rotX - rx)/d);

            visionSub.update();
            if (visionSub.hasTarget()) {
                turretSub.autoTrack(visionSub.getStrafeIn());
            } else {
                turretSub.hold();
            }

            if (gamepad1.b && !prevB) {
                intakeOn = !intakeOn;
                intakeSub.setIntake1(intakeOn ? 1.0 : 0.0);
            }
            prevB = gamepad1.b;

            if (gamepad1.x && !prevX) {
                launchSub.startLaunch();
            }
            prevX = gamepad1.x;

            launchSub.update();
        }
    }
}


