package org.firstinspires.ftc.teamcode.pedro;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "JaydenTeleOp", group = "AA")
@Config

public class JaydenTeleOp extends BaseClass{
    public enum State {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    State state = State.IDLE;

    @Override
    public void runOpMode(){
        Hwinit();
        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        while(opModeIsActive()){
            LLResult limelight_detect = limelight.getLatestResult();

            // Mecanum
            MecanumDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            // Flower collector
            if(gamepad1.left_trigger>0.3) {
                blocker.setPosition(blockDown);
                intake.setPower(1);
                flywheel.setPower(0);
                counter.setPower(0);
                flower.setPosition(flowerUp);
                break;
            }

            // Turret control from gunner
            if(gamepad2.right_bumper) {
                turret.setPower(0.7);
            } else if(gamepad2.left_bumper) {
                turret.setPower(-0.7);
            } else if(limelight_detect.isValid()){
                if(Math.abs(limelight_detect.getTx())>5) {
                    double factor = 0.009 * Math.pow((Math.abs(limelight_detect.getTx()) / 5), 1.15);
                    turret.setPower(factor * limelight_detect.getTx());
                }
            } else {
                turret.setPower(0);
            }

            // FSM
            switch(state){
                case IDLE:
                    blocker.setPosition(blockDown);
                    intake.setPower(0);
                    flywheel.setPower(0);
                    counter.setPower(0);
                    flower.setPosition(flowerDown);
                    if(gamepad1.left_bumper) {
                        state = State.INTAKE;
                    }

                    if(gamepad1.right_bumper) {
                        state = State.OUTTAKE;
                    }
                case INTAKE:
                    blocker.setPosition(blockDown);
                    intake.setPower(1);
                    flywheel.setPower(0);
                    counter.setPower(0);
                case OUTTAKE:
                    flywheel.setPower(0.465);
                    counter.setPower(0.41);
                    if(gamepad1.right_trigger>0.3){
                        blocker.setPosition(blockUp);
                        intake.setPower(1);
                    } else {
                        blocker.setPosition(blockDown);
                        intake.setPower(0);
                    }
            }

            // Telemetry
            telemetry.addData("flywheel", flywheel.getPower());
            telemetry.addData("turret", leftFront.getCurrentPosition());
            telemetry.addData("ty", limelight_detect.getTy());
            telemetry.addData("tx", limelight_detect.getTx());
            telemetry.addData("joystick", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}