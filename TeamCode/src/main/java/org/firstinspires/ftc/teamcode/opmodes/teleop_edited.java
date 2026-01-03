package org.firstinspires.ftc.teamcode.DecodeOpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.launch;
import org.firstinspires.ftc.teamcode.subsystems.turret;
import org.firstinspires.ftc.teamcode.subsystems.vision;

@TeleOp(name="teleop_edited")
public class teleop_edited extends LinearOpMode {

    // --- Drive Motors ---
    private DcMotor FR, FL, BR, BL;

    // --- Intake / Shooter Loader ---
    private DcMotor intakeOne, intakeTwo;
    private DcMotor fastFingler;

    // --- Turret ---
    private Servo leftServo, rightServo;

    // --- Subsystems ---
    private vision visionSub;
    private turret turretSub;
    private launch launchSub;

    // --- IMU ---
    private IMU imu;

    // --- Button state variables ---
    private boolean intakeOneOn = false;
    private boolean intakeTwoOn = false;
    private boolean fastFinglerOn = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Drive Motors ---
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeOne = hardwareMap.get(DcMotor.class,"intakeOneMotor");
        intakeTwo = hardwareMap.get(DcMotor.class,"intakeTwoMotor");
        fastFingler = hardwareMap.get(DcMotor.class,"Outertake");

        leftServo = hardwareMap.get(Servo.class,"leftServo");
        rightServo = hardwareMap.get(Servo.class,"rightServo");
        turretSub = new turret(leftServo, rightServo);

        visionSub = new vision(hardwareMap);
        launchSub = new launch(hardwareMap);

        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            FL.setPower((rotY + rotX + rx) / denom);
            BL.setPower((rotY - rotX + rx) / denom);
            FR.setPower((rotY - rotX - rx) / denom);
            BR.setPower((rotY + rotX - rx) / denom);

            visionSub.update();
            if(visionSub.hasTarget()) {
                double error = visionSub.getStrafeIn();
                turretSub.autoTrack(error);
            } else {
                turretSub.hold();
            }

            if(gamepad1.b && !prevB) {
                intakeOneOn = !intakeOneOn;
                intakeOne.setPower(intakeOneOn ? -1 : 0);
            }
            prevB = gamepad1.b;

            if(gamepad1.dpad_up) {
                intakeTwoOn = !intakeTwoOn;
                intakeTwo.setPower(intakeTwoOn ? -0.7 : 0);
                intakeOneOn = !intakeOneOn;
                intakeOne.setPower(intakeOneOn ? 1 : 0);
            }

            if(gamepad1.x && !prevX) {
                launchSub.startLaunch();
            }
            prevX = gamepad1.x;

            launchSub.update();

            if(gamepad1.y && !prevY) {
                intakeOne.setPower(0); intakeOneOn=false;
                intakeTwo.setPower(0); intakeTwoOn=false;
                fastFingler.setPower(0); fastFinglerOn=false;
                launchSub = new launch(hardwareMap); // reset launch subsystem
            }
            prevY = gamepad1.y;

            idle();
        }
    }
}

