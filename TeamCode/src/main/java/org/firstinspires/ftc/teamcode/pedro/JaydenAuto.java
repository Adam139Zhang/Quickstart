package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;

@Autonomous(name = "JaydenAuto", group = "AA")
@Config

public class JaydenAuto extends BaseClass {
    private Follower follower;

    @Override
    public void runOpMode(){
        Hwinit();
        waitForStart();

        while(opModeIsActive()){

            // Mecanum
            MecanumDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            flywheel.setPower(0.465);
            counter.setPower(0.41);

            // Telemetry
            telemetry.addData("Flywheel", flywheel.getPower());
            telemetry.addData("Turret", leftFront.getCurrentPosition());
            telemetry.addData("Joystick", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}