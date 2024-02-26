package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="red_aud", group="auto")
public class red_aud extends OpMode
{

    public static double x1 = -45;
    public static double y1 = -5;
    public static double h1 = 180;
    public static double x2 = -40;
    public static double y2 = 4;
    public static double h2 = 180;
    public static double x3 = -50;
    public static double y3 = -50;
    public static double h3 = 270;
    public static double x4 = -32;
    public static double y4 = -90;
    public static double h4 = 270;

    public static double x5 = -50;
    public static double y5 = -90;
    public static double h5 = 270;

    public static double turn = 180;

    public SampleMecanumDrive drive;

    DistanceSensor distance2;
    DistanceSensor distance4;

    DcMotorEx intake;
    DcMotorEx lift1;
    DcMotorEx lift2;
    ServoImplEx arm;
    Servo bucket;
    Servo outtake_lid;
    AnalogInput arm_encoder;


    public static int a = 500;
    public static int b = 100;

    //pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d =0;
    public static double f = 0;
    public static double target = 110;

    double liftpos1;
    double liftpos2;

    double aPos =.15;

    public static double bPosx =.35 ;

    public static double score = 250;



    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);

        target = 0;
        bPosx = 0.40;
        aPos = 0.88;
        controller = new PIDController(p,i,d);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);


        arm = (ServoImplEx) hardwareMap.get(Servo.class, "arm");
        bucket = hardwareMap.get(Servo.class, "bucket");
        arm.setPwmRange(new PwmControl.PwmRange(505, 2495));
        arm_encoder = hardwareMap.get(AnalogInput.class, "arm_encoder");
        outtake_lid = hardwareMap.get(Servo.class, "outtake_lid");

        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance4 = hardwareMap.get(DistanceSensor.class, "distance4");


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");


    }

    @Override
    public void init_loop() {

        if (distance2.getDistance(DistanceUnit.CM)<200) {
            turn = 180;


            x1 = -40;
            y1 = -12;
            h1 = 180;
/*
            x4 = -30;
            y4 = -90;
            h4 = 270;
*/

        }
        else if (distance4.getDistance(DistanceUnit.CM)<200) {
            turn = 180;

            x1 = -45;
            y1 = 0;
            h1 = 180;
/*
            x4 = -25;
            y4 = -91;
            h4 = 270;
*/
        }
        else {

            turn = -90;

            x1 = -26;
            y1 = 2;
            h1 = 270;
/*
            x4 = -29;
            y4 = -91;
            h4 = 260;
*/
        }

        telemetry.addData("distance2", distance2.getDistance(DistanceUnit.CM));
        telemetry.addData("distance4", distance4.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    @Override
    public void start() {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(28)
                .turn(Math.toRadians(turn))
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(h1)))
                .addTemporalMarker(() -> intake.setPower(-0.6))
                .waitSeconds(1)
                .addTemporalMarker(() -> intake.setPower(0))
                .forward(5)
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(h2)))
                .turn(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(h3)))
                .addTemporalMarker(() -> {
                    target = 500;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    aPos = 0.45;
                    bPosx = 0.18;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = score;
                })
                .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(h4)))
                .addTemporalMarker(() -> {
                    outtake_lid.setPosition(0.01);
                })
                .waitSeconds(1.5)
                .back(5)
                .addTemporalMarker(() -> {
                    target = 500;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    bPosx = .36;
                })
                .addTemporalMarker(() -> {
                    aPos = 0.88;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    target = 0;
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(h5)))


                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
    }

    @Override
    public void loop() {


        drive.update();

        controller.setPID(p, i, d);
        int liftPos1 = lift1.getCurrentPosition();
        int liftPos2 = lift2.getCurrentPosition();
        double pid = controller.calculate(liftPos1, target);
        double pid2 = controller.calculate(liftPos2, target);
        double ff = 0;

        double lPower1 = pid +ff;
        double lPower2 = pid2 +ff;

        lift1.setPower(lPower1);
        lift2.setPower(lPower2);

        liftpos1 = lift1.getCurrentPosition();
        liftpos2 = lift2.getCurrentPosition();

        arm.setPosition(aPos);
        bucket.setPosition(bPosx);

        // intake.update();
        double pos = arm_encoder.getVoltage() / 3.3 * 360;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("liftpos1", liftpos1);
        telemetry.addData("liftpos1", liftpos1);
        telemetry.addData("arm", pos);
        telemetry.update();
    }

    @Override
    public void stop() {

    }


}
