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

@TeleOp(name = "MeetsTeleop", group = "AA")
@Config

public class MeetsTeleOp extends BaseClass{

    @Override
    public void runOpMode(){
        Hwinit();
        waitForStart();
        while(opModeIsActive()){
            LLResult result1 = limelight.getLatestResult();
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            MecanumDrive(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            if(gamepad1.right_trigger>0.3){
                flower.setPosition(flowerDown);
            }else{
                flower.setPosition(flowerUp);
            }
            if(gamepad1.left_trigger>0.3){
                intake.setPower(intakeSpeed);
            }else{
                intake.setPower(0);
            }
            flywheelPID(0.5);

            if(gamepad2.right_bumper){
                turret.setPower(0.7);
            }else if(gamepad2.left_bumper){
                turret.setPower(-0.7);
            }
//            else if(Math.abs(gamepad1.left_stick_x)>0.3){
//                double factor = gamepad1.left_stick_x * Math.abs(gamepad1.left_stick_x);
//                turret.setPower(-1*factor);
//            }
            else if(result1.isValid()){
                if(Math.abs(result1.getTx())>5) {
                    double factor = 0.009 * Math.pow((Math.abs(result1.getTx()) / 5), 1.15);
                    turret.setPower(factor * result1.getTx());
                }
            }else{
                turret.setPower(0);
            }
            if(gamepad1.left_bumper){
                intakeSpeed = 1;
                flywheel.setPower(0.465);
                counter.setPower(0.41);
                if(gamepad1.right_bumper){
                    blocker.setPosition(blockUp);
                }else{
                    blocker.setPosition(blockDown);
                }
            }else{
                intakeSpeed = 1;
                flywheel.setPower(0.2);
                counter.setPower(0.2);
                blocker.setPosition(blockDown);
            }
//            switch(state){
//                case IDLE:
//                    blocker.setPosition(blockDown);
//                    intake.setPower(0);
//                    flywheel.setPower(0);
//                    counter.setPower(0);
//                    if(gamepad1.right_bumper){
//                        state = State.INTAKE;
//                    }
//            }
            telemetry.addData("flywheel", flywheel.getPower());
            telemetry.addData("turret", leftFront.getCurrentPosition());
            telemetry.addData("ty", result1.getTy());
            telemetry.addData("tx", result1.getTx());
            telemetry.addData("joystick", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}
