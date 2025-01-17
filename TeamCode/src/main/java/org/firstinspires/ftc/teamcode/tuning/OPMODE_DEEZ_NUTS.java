package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

public class OPMODE_DEEZ_NUTS extends LinearOpMode {

    private DcMotor viperUp;
    private ServoController ControlHub_ServoController;
    private Servo specimon;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Viper Slide and Servo Initialization
        int pos;
        double servo;
        int startpos;

        ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");
        specimon = hardwareMap.get(Servo.class, "specimon");
        viperUp = hardwareMap.get(DcMotor.class, "viperUp");

        ControlHub_ServoController.pwmEnable();
        servo = specimon.getPosition();
        pos = viperUp.getCurrentPosition();
        startpos = viperUp.getCurrentPosition();
        viperUp.setTargetPosition(0);
        viperUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) viperUp).setTargetPositionTolerance(1000);
        viperUp.setPower(1);

        waitForStart();

        while (opModeIsActive()) {
            // Drive control code
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

            // Servo control code
            if (gamepad1.dpad_up) {
                servo += 0.005;
            }
            if (gamepad1.dpad_down) {
                servo -= 0.005;
            }
            if (servo >= 1) {
                servo = 1;
            }
            if (servo <= 0.65) {
                servo = 0.65;
            }
            specimon.setPosition(servo);

            // Viper Slide control code
            if (pos >= startpos) {
                pos = 10;
            }
            if (pos <= -3310) {
                pos = -3310;
            }
            if (pos <= startpos) {
                viperUp.setPower(0);
            } else {
                viperUp.setPower(1);
            }
            pos += gamepad1.left_trigger * 20;
            pos += gamepad1.right_trigger * -20;
            viperUp.setTargetPosition(pos);

            // Telemetry updates
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
