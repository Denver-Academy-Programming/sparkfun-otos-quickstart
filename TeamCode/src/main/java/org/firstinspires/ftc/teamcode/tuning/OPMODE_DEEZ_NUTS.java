package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

public class OPMODE_DEEZ_NUTS extends LinearOpMode {

    private DcMotor viperUp;
    private DcMotor viperForward;


    private ServoController ControlHub_ServoController;
    private Servo specimon;
    private Servo sample;
    private CRServo intake;
    private Servo  inrot;
    private CRServo  convayor;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Viper Slide and Servo Initialization

        // servos
        double servo;
        double servo2;
        //double servo3;
        double servo4;
        double servo5;

        // viper slide
        int pos;
        int pos2;
        int pos3;
        int startpos;
        int startpos2;


        ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");
        // servo hardware mapping

        specimon = hardwareMap.get(Servo.class, "specimon");
        sample = hardwareMap.get(Servo.class, "sample");
        intake = hardwareMap.get(CRServo.class, "intake");
        inrot = hardwareMap.get(Servo.class, "inrot");
        // viper slide hardware mapping
        viperUp = hardwareMap.get(DcMotor.class, "viperUp");
        viperForward = hardwareMap.get(DcMotor.class, "viperForward");
        convayor = hardwareMap.get(CRServo.class, "convayor");


        // enable pwm on control hub
        ControlHub_ServoController.pwmEnable();

        // set servo positions

        //servo = specimon.getPosition();
        servo = 0.65;
        specimon.setPosition(servo);

        //servo2 = sample.getPosition();
        servo2 = 0.69;
        sample.setPosition(servo2);

        //servo3 = intake.getPosition();
        //servo3 = 0.69;
        intake.setPower(0);

        servo4 = inrot.getPosition();
        //servo4 = 0.69;
        inrot.setPosition(servo4);

        servo5 = convayor.getPower();
        //servo5 = 0.69;
        convayor.setPower(0);

        // set up viperslide for start

        viperUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pos = viperUp.getCurrentPosition();
        pos2 = viperForward.getCurrentPosition();

        startpos = viperUp.getCurrentPosition();
        startpos2 = viperForward.getCurrentPosition();

        viperUp.setTargetPosition(0);
        viperForward.setTargetPosition(0);

        viperUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperForward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ((DcMotorEx) viperUp).setTargetPositionTolerance(1000);
        ((DcMotorEx) viperForward).setTargetPositionTolerance(1000);


        waitForStart();

        while (opModeIsActive()) {
            // Drive control code
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            (-gamepad1.left_stick_y * 0.6),
                            (-gamepad1.left_stick_x * 0.6)
                    ),
                    (-gamepad1.right_stick_x * 0.6)
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("servo - specimon - val", servo);
            telemetry.addData("servo - specimon - pos", specimon.getPosition());
            telemetry.addData("servo2 - sample - val", servo2);
            telemetry.addData("servo2 - sample - pos", sample.getPosition());
            telemetry.addData("servo3 - intake", intake.getPower());
            telemetry.addData("servo4 - inrot - val", servo4);
            telemetry.addData("servo4 - inrot - pos", inrot.getPosition());
            telemetry.addData("servo5 - convayor - val", convayor.getPower());
            telemetry.addData("servo5 - convayor - pos", inrot.getPosition());
            telemetry.addData("start pos", startpos);
            telemetry.addData("pos", pos);
            telemetry.addData("viperUp", viperUp.getCurrentPosition());
            telemetry.addData("start pos 2", startpos2);
            telemetry.addData("pos 2", pos2);
            telemetry.addData("viperForward", viperForward.getCurrentPosition());




            // Specimon Servo control code
            if (gamepad1.dpad_up) {
                servo += 0.01;
            }
            if (gamepad1.dpad_down) {
                servo -= 0.01;
            }
            if (servo >= 1) {
                servo = 1;
            }
            if (servo <= 0.65) {
                servo = 0.65;
            }
            specimon.setPosition(servo);


            // Sample Servo control code
            if (gamepad1.dpad_left) {
                servo2 += 0.01;
            }
            if (gamepad1.dpad_right) {
                servo2 -= 0.01;
            }
            if (servo2 >= 0.75) {
                servo2 = 0.75;
            }
            if (servo2 <= 0) {
                servo2 = 0;
            }
            sample.setPosition(servo2);

            // Intake Servo control code
            if (gamepad2.a) {
                intake.setPower(1);
            }
            if (gamepad2.y) {
                intake.setPower(-1);
            }
            if (gamepad2.b) {
                intake.setPower(0);
            }

            // Inrot Servo control code
            if (gamepad2.dpad_up) {
                servo4 += 0.01;
            }
            if (gamepad2.dpad_down) {
                servo4 -= 0.01;
            }
            if (servo4 >= 0.7894) {
                servo4 = 0.7894;
            }
            if (servo4 <= 0.3194) {
                servo4 = 0.3194;
            }
            inrot.setPosition(servo4);

            // convayor Servo control code
            if (gamepad2.x) {
                convayor.setPower(-1);
            }
            if (gamepad2.b) {
                convayor.setPower(0);
            }

            // Viper Slide control code up
            if (pos >= startpos) {
                pos = startpos;
            }
            if (pos <= -2446 + startpos) {
                pos = -2446 + startpos;
            }
            if (pos >= startpos) {
                viperUp.setPower(0);
            } else {
                viperUp.setPower(1);
            }
            pos += gamepad1.left_trigger * 20;
            pos += gamepad1.right_trigger * -20;
            viperUp.setTargetPosition(pos);

            // Viper Slide control code forward
            if (pos2 >= startpos2) {
                pos2 = startpos2;
            }
            if (pos2 <= -1550 + startpos2) {
                pos2 = -1550 + startpos2;
            }
            if (pos2 >= startpos2) {
                viperForward.setPower(0);
            } else {
                viperForward.setPower(1);
            }
            pos2 += gamepad2.left_trigger * 20;
            pos2 += gamepad2.right_trigger * -20;
            viperForward.setTargetPosition(pos2);


            // Telemetry updates
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
