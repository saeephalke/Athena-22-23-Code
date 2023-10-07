/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareMechanumAthenabot;
import org.firstinspires.ftc.teamcode.auton.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "webcam", group = "Pushbot")
public class                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              AutonTemplate extends LinearOpMode
{   
    //INTRODUCE VARIABLES HERE

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    HardwareMechanumAthenabot robot = new HardwareMechanumAthenabot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 0.5;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

     // Tag ID 1,2,3 from the 36h11 family 
     /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.left_front_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_back_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_back_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.left_front_drv_Motor.getCurrentPosition(),
                robot.right_front_drv_Motor.getCurrentPosition(),
                robot.left_back_drv_Motor.getCurrentPosition(),
                robot.right_back_drv_Motor.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //come down
        // Step 3:  Drive Backwards for 1 Second
        //robot.hang_motor.setPower(-.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        if(tagOfInterest == null)
        {
            encoderDrive(DRIVE_SPEED, -10, -10, 0.6); // move front
        } else if(tagOfInterest.id == 1)
        {
            telemetry.addLine("move left");
            encoderDrive(DRIVE_SPEED, -10, -10, 0.6); // move front
            encoderDrive(DRIVE_SPEED, 8, -11, 0.53);
            strafe(DRIVE_SPEED, -10, -10, -10, -10, 0.5);

        } else if (tagOfInterest.id == 2) {
            telemetry.addLine("stay middle");
            encoderDrive(DRIVE_SPEED, -10, -10, 0.6); // move front

        } else if (tagOfInterest.id ==3) {
            telemetry.addLine("move right");
            encoderDrive(DRIVE_SPEED, -10, -10, 0.6); // move front
            encoderDrive(DRIVE_SPEED, -11, 8, 0.53);
            strafe(DRIVE_SPEED, -10, -10, -10, -10, 0.5);

        } else {
            telemetry.addLine("no move");
        }
        telemetry.update();


    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.left_front_drv_Motor.setTargetPosition(newLeftTarget);
            robot.right_front_drv_Motor.setTargetPosition(newRightTarget);
            robot.left_back_drv_Motor.setTargetPosition(newLeftTarget);
            robot.right_back_drv_Motor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_front_drv_Motor.setPower(Math.abs(speed));
            robot.right_front_drv_Motor.setPower(Math.abs(speed));
            robot.left_back_drv_Motor.setPower(Math.abs(speed));
            robot.right_back_drv_Motor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_front_drv_Motor.getCurrentPosition(),
                        robot.right_front_drv_Motor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.left_front_drv_Motor.setPower(0);
            robot.right_front_drv_Motor.setPower(0);
            robot.left_back_drv_Motor.setPower(0);
            robot.right_back_drv_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void strafe(double speed_strafe, double strafe_leftfrontInches, double strafe_rightfrontInches, double strafe_leftbackInches, double strafe_rightbackInches,
                       double strafe_timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (strafe_leftfrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (strafe_rightfrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (strafe_leftbackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (strafe_rightbackInches * COUNTS_PER_INCH);
            robot.left_front_drv_Motor.setTargetPosition(newLeftFrontTarget);
            robot.right_front_drv_Motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_drv_Motor.setTargetPosition(newLeftBackTarget);
            robot.right_back_drv_Motor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_front_drv_Motor.setPower(Math.abs(speed_strafe));
            robot.right_front_drv_Motor.setPower(Math.abs(speed_strafe));
            robot.left_back_drv_Motor.setPower(Math.abs(speed_strafe));
            robot.right_back_drv_Motor.setPower(Math.abs(speed_strafe));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < strafe_timeoutS) &&
                    (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_front_drv_Motor.getCurrentPosition(),
                        robot.right_front_drv_Motor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.left_front_drv_Motor.setPower(0);
            robot.right_front_drv_Motor.setPower(0);
            robot.left_back_drv_Motor.setPower(0);
            robot.right_back_drv_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void vacuum(double speed, double vacuumInches, double timeoutS) {
        int vacuumTarget;

        if (opModeIsActive()) {
            vacuumTarget = robot.vacuum1.getCurrentPosition() + (int) (vacuumInches * COUNTS_PER_INCH);
            robot.vacuum1.setTargetPosition(vacuumTarget);
            // robot.vacuum2.setTargetPosition(vacuumTarget);

            robot.vacuum1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // robot.vacuum2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.vacuum1.setPower(Math.abs(speed));
            // robot.vacuum2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.vacuum1.isBusy() /* && robot.vacuum2.isBusy() */)) {
                telemetry.addData("Path1", "Running to %7d :%7d", vacuumTarget);
                /*telemetry.addData("Path2", "Running at %7d :%7d", */
                robot.vacuum1.getCurrentPosition();
                //robot.vacuum2.getCurrentPosition());
                telemetry.update();
            }

            robot.vacuum1.setPower(0);
            //robot.vacuum2.setPower(0);

            robot.vacuum1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.vacuum2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void elevator(double speed, double elevatorInches, double timeoutS) //pulley system
    {
        int elevatorTarget;

        elevatorTarget = robot.elevator.getCurrentPosition() + (int) (elevatorInches * COUNTS_PER_INCH);
        robot.elevator.setTargetPosition(elevatorTarget);

        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.elevator.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.elevator.isBusy())) {
            telemetry.addData("Path1", "Running to %7d :%7d", elevatorTarget);
            //telemetry.addData("Path2", "Running at %7d :%7d",
            robot.elevator.getCurrentPosition();
            telemetry.update();
        }

        robot.elevator.setPower(0);

        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void guide(double speed, double guideInches, double timeoutS) {
        int guideTarget;

        guideTarget = robot.guide.getCurrentPosition() + (int) (guideInches * COUNTS_PER_INCH);

        robot.guide.setTargetPosition(guideTarget);

        robot.guide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.guide.setPower(Math.abs(speed));

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.guide.isBusy())) {
            telemetry.addData("Path1", "Running to %7d :%7d", guideTarget);
            //telemetry.addData("Path2", "Running at %7d :%7d",
            robot.guide.getCurrentPosition();
            telemetry.update();
        }

        robot.guide.setPower(0);
        robot.guide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void intake(double speed, double intakeinches, double intake2inches, double timeoutS) {
        int newIntakeTarget;
        int newIntake2Target;

        if (opModeIsActive()) {
            newIntakeTarget = robot.intake_motor.getCurrentPosition() + (int) (intakeinches * COUNTS_PER_INCH);
            newIntake2Target = robot.intake2_motor.getCurrentPosition() + (int) (intake2inches * COUNTS_PER_INCH);
            robot.intake_motor.setTargetPosition(newIntakeTarget);
            robot.intake2_motor.setTargetPosition(newIntake2Target);

            robot.intake_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intake2_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.intake_motor.setPower(Math.abs(speed));
            robot.intake2_motor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.intake_motor.isBusy()) && (robot.intake2_motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newIntakeTarget, newIntake2Target);
                //telemetry.addData("Path2", "Running at %7d :%7d",
                robot.intake_motor.getCurrentPosition();
                robot.intake2_motor.getCurrentPosition();
                telemetry.update();
            }
            robot.intake_motor.setPower(0);
            robot.intake2_motor.setPower(0);

            robot.intake_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.intake2_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void box(double box_boxpos) //box that contains the loot
    {
        if (opModeIsActive()) {
            robot.box.setPosition(box_boxpos);
        }
    }

    public void gate(double gate_gatepos) {
        if (opModeIsActive()) {
            robot.gate.setPosition(gate_gatepos);
        }

    }

    public void launch(double speed, double fly_wheelinches, double rampinches, double timeoutS) {
        int newFlyWheelTarget;
        int newRampTarget;

        if (opModeIsActive()) {
            newFlyWheelTarget = robot.fly_wheel.getCurrentPosition() + (int) (fly_wheelinches * COUNTS_PER_INCH);
            newRampTarget = robot.ramp.getCurrentPosition() + (int) (rampinches * COUNTS_PER_INCH);

            robot.fly_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ramp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.fly_wheel.setPower(Math.abs(speed));
            robot.ramp.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.intake_motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFlyWheelTarget, newRampTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d",
                robot.fly_wheel.getCurrentPosition();
                robot.ramp.getCurrentPosition();
                telemetry.update();
            }

            robot.fly_wheel.setPower(0);
            robot.ramp.setPower(0);

            robot.fly_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}