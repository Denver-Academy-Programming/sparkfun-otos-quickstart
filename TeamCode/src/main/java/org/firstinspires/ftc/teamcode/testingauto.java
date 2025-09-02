package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


@Config
@Autonomous(name = "AtesintEZ_NUTS", group = "Autonomous")
public class testingauto extends LinearOpMode {
    public class viperup {
        private DcMotorEx viperup;
        int pos;
        int startpos;


        public viperup(HardwareMap hardwareMap) {
            viperup = hardwareMap.get(DcMotorEx.class, "viperUp");
            viperup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pos = viperup.getCurrentPosition();
            startpos = viperup.getCurrentPosition();
            viperup.setTargetPosition(0);
            viperup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) viperup).setTargetPositionTolerance(1000);

            viperup.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class viperupup implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperup.setTargetPosition(2446);
                    initialized = true;
                }

                double pos = viperup.getCurrentPosition();
                packet.put("viperupupPos", pos);
                if (pos >= startpos) {
                    pos = startpos;
                }
                if (pos <= -2446 + startpos) {
                    pos = -2446 + startpos;
                }
                return false;
            }
        }

        public Action viperupup() {
            return new viperupup();
        }

        public class viperupmid implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperup.setTargetPosition(969);
                    initialized = true;
                }

                double pos = viperup.getCurrentPosition();
                packet.put("viperupmidPos", pos);
                if (pos >= startpos) {
                    pos = startpos;
                }
                if (pos <= -2446 + startpos) {
                    pos = -2446 + startpos;
                }
                return false;
            }
        }

        public Action viperupmid() {
            return new viperupmid();
        }

        public class viperupdown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    viperup.setTargetPosition(0);
                    initialized = true;
                }

                double pos = viperup.getCurrentPosition();
                packet.put("viperupdownPos", pos);
                if (pos >= startpos) {
                    pos = startpos;
                }
                if (pos <= -2446 + startpos) {
                    pos = -2446 + startpos;
                }
                return false;
            }
        }

        public Action viperupdown() {
            return new viperupdown();
        }
    }

    public class claw {

        private Servo specimon;
        private ServoController ControlHub_ServoController;

        double servo;


        public claw(HardwareMap hardwareMap) {
            specimon = hardwareMap.get(Servo.class, "specimon");
            //ControlHub_ServoController.pwmEnable();
            servo = 0.65;
        }

        public class closeclaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo = 1;
                if (servo >= 1) {
                    servo = 1;
                }
                if (servo <= 0.65) {
                    servo = 0.65;
                }
                specimon.setPosition(servo);                return false;
            }
        }
        public Action closeclaw() {
            return new closeclaw();
        }

        public class openclaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo = 0.65;
                if (servo >= 1) {
                    servo = 1;
                }
                if (servo <= 0.65) {
                    servo = 0.65;
                }
                specimon.setPosition(servo);                return false;
            }
        }
        public Action openclaw() {
            return new openclaw();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        claw claw = new claw(hardwareMap);
        viperup viperup = new viperup(hardwareMap);

        //traj go here
        TrajectoryActionBuilder path12 = drive.actionBuilder(new Pose2d(11.64, -63.24, Math.toRadians(90.00)))
                .splineTo(new Vector2d(34.32, -36.00), Math.toRadians(57.51))
                .splineTo(new Vector2d(46.50, -14.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(48.00, -18.00), Math.toRadians(-90.00))
                .splineTo(new Vector2d(47.97, -36.98), Math.toRadians(-89.46))
                .splineTo(new Vector2d(48.00, -50.00), Math.toRadians(-90.00))
                .waitSeconds(5);
        //wait 5 seconds
        TrajectoryActionBuilder path2 = drive.actionBuilder(new Pose2d(48.00, -63.00, Math.toRadians(90.00)));
        //close claw
        TrajectoryActionBuilder path3 = drive.actionBuilder(new Pose2d(48.00, -63.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(0.00, -33.00), Math.toRadians(90.00));
        //viper to mid level then release claw then viper down
        TrajectoryActionBuilder path4 = drive.actionBuilder(new Pose2d(0.00, -33.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(63.00, -62.00), Math.toRadians(0.00));


        TrajectoryActionBuilder path1 = drive.actionBuilder(new Pose2d(24.00, -51.00, Math.toRadians(180.00)))
                .splineToConstantHeading(new Vector2d(0.00, -51.00), Math.toRadians(180.00));





        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeclaw());


        waitForStart();



        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        path1.build()
                )
        );
    }
}