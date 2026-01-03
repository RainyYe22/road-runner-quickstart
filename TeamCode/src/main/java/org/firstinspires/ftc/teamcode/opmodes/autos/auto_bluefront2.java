package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.field_constants;

@Autonomous(name = "auto_bluefront2", group = "Auto")
public class auto_bluefront2 extends LinearOpMode {

    // Drive motors
    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    IMU imu;

    // Pose of the robot (x, y in inches, heading in radians)
    Pose2d pose;

    // PID-ish constants
    static final double DRIVE_P = 0.05;
    static final double STRAFE_P = 0.05;
    static final double TURN_P = 1.5;

    // encoder to inch conversion (example, adjust to your hardware)
    static final double ENC_IN_PER_TICK = 1.0 / 1000.0; // placeholder, adjust

    // previous encoder positions
    int lastLeftFront = 0, lastLeftBack = 0, lastRightFront = 0, lastRightBack = 0;

    @Override
    public void runOpMode() {

        //hardware
        leftFront  = hardwareMap.get(DcMotorEx.class, "FL");
        leftBack   = hardwareMap.get(DcMotorEx.class, "BL");
        rightBack  = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        // IMU init
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        ));

        pose = field_constants.FRONT_BLUE_START;

        waitForStart();
        if (isStopRequested()) return;

        driveTo(field_constants.BLUE_BALL_LOCATIONS[0]);
        driveTo(field_constants.LEVER);
        driveTo(field_constants.BLUE_BALL_LOCATIONS[1]);
        driveTo(field_constants.BACK_BLUE_START);

        stopDrive();
    }


    private void driveTo(Pose2d target) {
        while (opModeIsActive()) {

            updatePoseFromEncodersAndIMU(); // approximate X/Y + heading

            Vector2d error = target.position.minus(pose.position);
            double headingError = angleWrap(target.heading.toDouble() - pose.heading.toDouble());

            // stop condition
            if (error.norm() < 1.5 && Math.abs(headingError) < Math.toRadians(5))
                break;

            double forward = error.x * DRIVE_P;
            double strafe  = error.y * STRAFE_P;
            double turn    = headingError * TURN_P;

            setDrivePowers(forward, strafe, turn);

            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            telemetry.addData("H", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }

        stopDrive();
        sleep(200);
    }

    private void setDrivePowers(double forward, double strafe, double turn) {

        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;
        double rf = forward - strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(lb),
                                Math.max(Math.abs(rb), Math.abs(rf)))));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
        rightFront.setPower(rf / max);
    }

    private void stopDrive() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    private void updatePoseFromEncodersAndIMU() {

        int currLF = leftFront.getCurrentPosition();
        int currLB = leftBack.getCurrentPosition();
        int currRF = rightFront.getCurrentPosition();
        int currRB = rightBack.getCurrentPosition();

        // change in encoder ticks
        double dLF = (currLF - lastLeftFront) * ENC_IN_PER_TICK;
        double dLB = (currLB - lastLeftBack)  * ENC_IN_PER_TICK;
        double dRF = (currRF - lastRightFront) * ENC_IN_PER_TICK;
        double dRB = (currRB - lastRightBack) * ENC_IN_PER_TICK;

        lastLeftFront  = currLF;
        lastLeftBack   = currLB;
        lastRightFront = currRF;
        lastRightBack  = currRB;

        // Mecanum forward/strafe calculation
        double forward = (dLF + dLB + dRF + dRB)/4.0;
        double strafe  = (-dLF + dLB + dRF - dRB)/4.0;

        // IMU heading
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double heading = angles.getYaw(AngleUnit.RADIANS);

        // rotate forward/strafe by heading
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        double dx = forward * cosH - strafe * sinH;
        double dy = forward * sinH + strafe * cosH;

        pose = new Pose2d(
                pose.position.plus(new Vector2d(dx, dy)),
                heading
        );
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}