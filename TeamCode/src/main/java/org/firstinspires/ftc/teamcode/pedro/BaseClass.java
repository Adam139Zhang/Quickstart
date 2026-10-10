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

@Config
public abstract class BaseClass extends LinearOpMode {
    DcMotorEx leftFront, rightFront, leftBack, rightBack,
            intake, flywheel, counter;

    Servo flower, blocker;
    CRServo turret;
    IMU imu;
    Limelight3A limelight;

    public ElapsedTime runtime = new ElapsedTime();
    double[] stoptime = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    PIDController flyPID = new PIDController(kP, kI, kD);
    public static double kP = 0, kI = 0, kD = 0, kF = 0.001;

    public static double blockDown = 0.2, blockUp = 0.11,
            flowerDown = 0.455, flowerUp = 0.81,
            intakePower = 0;
    public static double intakeSpeed = 0.7;
    public double maxSpeed = 1.0;
    public boolean blockerDown = false;

    State state = State.IDLE;
    public enum State{
        IDLE,
        INTAKE,
        OUTTAKE,
    }

    @Override
    public abstract void runOpMode() throws InterruptedException;
    public void Hwinit(){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        counter = hardwareMap.get(DcMotorEx.class, "counterRoller");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        counter.setDirection(DcMotorSimple.Direction.REVERSE);

        flower = hardwareMap.get(Servo.class, "flower");
        blocker = hardwareMap.get(Servo.class, "blocker");
        turret = hardwareMap.get(CRServo.class, "turret");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();
    }

    void MecanumDrive(double y, double x, double rx) {


//        y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
//        x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
//        rx = gamepad1.left_stick_x;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double FLPower = (-y + x + rx) / denominator;
        double BLPower = (-y - x + rx) / denominator;
        double FRPower = (-y - x - rx) /denominator;
        double BRPower = (-y + x - rx) / denominator;
        leftFront.setPower(-FLPower);
        leftBack.setPower(-BLPower);
        rightFront.setPower(-FRPower);
        rightBack.setPower(-BRPower);
    }

    void flywheelPID(double pow){
        double target_vel = pow;
        flyPID.setPID(kP, kI, kD);
        double vel =  flywheel.getVelocity();
        double pid = flyPID.calculate(vel,target_vel);
        double ff = kF *target_vel;
        double power = pid + ff;
        flywheel.setPower(power);
    }

}
