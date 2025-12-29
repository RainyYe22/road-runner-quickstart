package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "teleop")
public class teleop extends LinearOpMode {

    private DcMotor FranklinDelanoRoosevelt; // FR
    private DcMotor FL;
    private DcMotor BoeingDriveR;            // BR
    private DcMotor BL;

    private DcMotor intakeOne;
    private DcMotor intakeTwo;

    private Servo Servoo;

    IMU imu;

    boolean intakeOneOn = false;
    boolean intakeTwoOn = false;
    boolean prevCircle = false;
    boolean prevSquare = false;

    boolean servoUp = false;
    boolean prevX = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive Motors
        FranklinDelanoRoosevelt = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BoeingDriveR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        // Intakes
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOneMotor");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwoMotor");

        // Servo
        Servoo = hardwareMap.get(Servo.class, "Servo");

        // Directions
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BoeingDriveR.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU INIT
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Input
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5;

            // IMU FIELD-CENTRIC
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            // Fix motor directions
            frPower *= -1;
            brPower *= -1;

            // Set powers
            FL.setPower(flPower);
            BL.setPower(blPower);
            FranklinDelanoRoosevelt.setPower(frPower);
            BoeingDriveR.setPower(brPower);

            if (gamepad1.circle && !prevCircle) {
                intakeOneOn = !intakeOneOn;
                intakeOne.setPower(intakeOneOn ? -0.7 : 0);
            }
            prevCircle = gamepad1.circle;

            if (gamepad1.square && !prevSquare) {
                intakeTwoOn = !intakeTwoOn;
                intakeTwo.setPower(intakeTwoOn ? 1 : 0);
            }
            prevSquare = gamepad1.square;

            if (gamepad1.dpad_up) {
                intakeOne.setPower(0.35);
                intakeTwo.setPower(1);

                intakeOneOn = true;
                intakeTwoOn = true;
            }

            if (gamepad1.x && !prevX) {
                servoUp = !servoUp;  // flip state
                Servoo.setPosition(servoUp ? 0.8 : 1.0);  // 1.0 = up, 0.0 = down
            }
            prevX = gamepad1.x;
        }
    }
}