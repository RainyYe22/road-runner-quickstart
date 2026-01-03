package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.field_constants;

@Autonomous(name = "autonomous_blue")
public class autonomous_blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, field_constants.blue_left);  // start pose
        vision_imu turret = new vision_imu(hardwareMap);

        DcMotor intakeOne = hardwareMap.get(DcMotor.class, "intakeOneMotor");
        DcMotor intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwoMotor");

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcherMotor");


        waitForStart();

        if (isStopRequested()) return;

        drive.actionBuilder(field_constants.blue_left)
                .lineToX(field_constants.blue_intake.position.x)
                .lineToY(field_constants.blue_intake.position.y)
                .build();

        intakeOne.setPower(1);
        intakeTwo.setPower(1);
        sleep(2000);
        intakeOne.setPower(0);
        intakeTwo.setPower(0);


        drive.actionBuilder(field_constants.blue_intake)
                .lineToX(field_constants.BLUE_GOAL_CENTER.x)
                .lineToY(field_constants.BLUE_GOAL_CENTER.y)
                .build();

        turret.setTargetHeading(Math.atan2(
                field_constants.BLUE_GOAL_CENTER.y - field_constants.blue_intake.position.y,
                field_constants.BLUE_GOAL_CENTER.x - field_constants.blue_intake.position.x
        ));
        sleep(500);


        launcher.setPower(1);
        sleep(1000);
        launcher.setPower(0);

        telemetry.addLine("Autonomous complete!");
        telemetry.update();
    }
}


