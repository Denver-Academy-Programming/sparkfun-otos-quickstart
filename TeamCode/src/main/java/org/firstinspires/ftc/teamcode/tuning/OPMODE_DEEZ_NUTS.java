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
    private DcMotor viperForward;

    private ServoController ControlHub_ServoController;
    private Servo specimon;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Viper Slide and Servo Initialization
        int pos;
        int pos2;
        double servo;
        int startpos;
        int startpos2;


        ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");
        specimon = hardwareMap.get(Servo.class, "specimon");
        viperUp = hardwareMap.get(DcMotor.class, "viperUp");
        viperForward = hardwareMap.get(DcMotor.class, "viperForward");


        ControlHub_ServoController.pwmEnable();
        servo = specimon.getPosition();
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
            telemetry.addData("servo", specimon.getPosition());
            telemetry.addData("start pos", startpos);
            telemetry.addData("pos", pos);
            telemetry.addData("viperUp", viperUp.getCurrentPosition());
            telemetry.addData("start pos 2", startpos2);
            telemetry.addData("pos 2", pos2);
            telemetry.addData("viperForward", viperForward.getCurrentPosition());



            // Servo control code
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

            // Viper Slide control code up
            if (pos >= startpos) {
                pos = startpos;
            }
            if (pos <= -3310 + startpos) {
                pos = -3310 + startpos;
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
