/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the Gamepad1 left stick moves the robot FWD and back, the Gamepad1 Right stick turns left and right.
 * Gamepad2 left stick will control the ball collector motors to suck in
 * the ball or shoot them. Gamepad2 left stick will also control the horizontal roller to move the balls in.
 * or out.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out thae @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AthenabotMechanumTeleop_Linear: Teleop", group="Athenabot")
//@Disabled
public class AthenabotMechanumTeleop_Linear extends LinearOpMode {



    /* Declare OpMode members. */

    HardwareMechanumAthenabot robot  = new HardwareMechanumAthenabot();

    // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    //grab must go before stack a and b
    //out x and y

    //final double PUSH_SPEED = 0.05;
    //final double PUSH2_SPEED = 0.05;
    final double MAX_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        double left;
        double right;
        double max;
        double v1 = 0.0;
        double v2 = 0.0;
        double v3 = 0.0;
        double v4 = 0.0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //robot.leftlift.setPosition(robot.SERVO_POS0);
        //robot.rightlift.setPosition(0.9);
        //telemetry.addData("after" , "init");
        //telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //holonomic(Speed, Turn, Strafe, MAX_SPEED );

            //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
            //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

            double Speed = gamepad1.left_stick_y;
            double Turn = -gamepad1.left_stick_x;
            //double Speed = gamepad1.left_stick_y;
            //double Turn = -gamepad1.left_stick_x;
            double Strafe = -gamepad1.right_stick_x;
            //robot.turn.setPosition(gamepad2.left_stick_x);
            //robot.push2.setPosition(-0.2);
            //telemetry.addData("after" , "opmode");
            //telemetry.update();


            double Magnitude = Math.abs(Speed) + Math.abs(Turn) + Math.abs(Strafe);
            Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 rangestraf
            // Wheels on joystick - gamepad 1

            if (robot.left_front_drv_Motor != null) {
                robot.left_front_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) +
                                        robot.scaleInput(Turn) -
                                        robot.scaleInput(-Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }
            if (robot.left_back_drv_Motor != null) {
                robot.left_back_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) +
                                        robot.scaleInput(Turn) +
                                        robot.scaleInput(-Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }
            if (robot.right_front_drv_Motor != null) {
                robot.right_front_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) -
                                        robot.scaleInput(Turn) -
                                        robot.scaleInput(Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }
            if (robot.right_back_drv_Motor != null) {
                robot.right_back_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) -
                                        robot.scaleInput(Turn) +
                                        robot.scaleInput(Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }


            // Hinge on a and x - gamepad 2

        /*if (robot.hinge != null) {
            if (gamepad2.a) { // cap ball up
                robot.hinge.setDirection(DcMotor.Direction.FORWARD); //might need to change to REVERSE
                robot.hinge.setPower(robot.HINGE_POWER);
            } else if (gamepad2.x) {  // cap ball
                robot.hinge.setDirection(DcMotor.Direction.REVERSE); //might need to change to FORWARD
                robot.hinge.setPower(robot.HINGE_POWER);
            } else {
                robot.hinge.setPower(0.0);
            }
        }*/
         /*if (robot.flip_motor != null) {
             if (gamepad2.left_bumper) { // ball shoot
                 robot.flip_motor.setDirection(DcMotor.Direction.FORWARD); //might need to change to FORWARD/REVERSE
                 robot.flip_motor.setPower(robot.FLIP_POWER);
             } else if (gamepad2.left_trigger > 0) {
                 robot.flip_motor.setDirection(DcMotor.Direction.REVERSE);
                 robot.flip_motor.setPower(robot.FLIP_POWER);
             } else {
                 robot.flip_motor.setPower(0.0);

             }

         }*/




         /*if (robot.stack2_motor != null) {
             if (gamepad2.left_bumper) { // ball shoot
                 robot.stack2_motor.setDirection(DcMotor.Direction.FORWARD); //might need to change to FORWARD/REVERSE
                 robot.stack2_motor.setPower(robot.STACK2_POWER);
             } else if (gamepad2.left_trigger > 0) {
                 robot.stack2_motor.setDirection(DcMotor.Direction.REVERSE);
                 robot.stack2_motor.setPower(robot.STACK2_POWER);
             } else {
                 robot.stack2_motor.setPower(0.0);

             }

         }*/
            // hangigng mechanism - gamepad 2 - trigger and bumper
         /*if (robot.intake_motor != null) {
             if (gamepad2.right_bumper) { // ball shoot
                 robot.intake_motor.setDirection(DcMotor.Direction.FORWARD); //might need to change to FORWARD/REVERSE
                 robot.intake_motor.setPower(robot.a_intake_power());
             } else if (gamepad2.right_trigger > 0) {
                 robot.intake_motor.setDirection(DcMotor.Direction.REVERSE);
                 robot.intake_motor.setPower(robot.a_intake_power());
             } else {
                 robot.intake_motor.setPower(0.0);

             }

         }*/

         /*if (robot.intake2_motor != null) {
             if (gamepad2.right_bumper) { // ball shoot
                 robot.intake2_motor.setDirection(DcMotor.Direction.REVERSE); //might need to change to FORWARD/REVERSE
                 robot.intake2_motor.setPower(robot.INTAKE2_POWER);
             } else if (gamepad2.right_trigger > 0) {
                 robot.intake2_motor.setDirection(DcMotor.Direction.FORWARD);
                 robot.intake2_motor.setPower(robot.INTAKE_POWER);
             } else {
                 robot.intake2_motor.setPower(0.0);

             }

         }*/

            telemetry.addData("Before", "Pushr");    //
            telemetry.update();
         /*if (robot.push != null) {
             telemetry.addData("In", "Push");    //
             telemetry.update();
             if (gamepad2.x) {
                 robot.push.setPosition(0.0);
                 //robot.set_push_position((robot.a_push_position() - PUSH_SPEED));
                 //} else if (gamepad2.b) {
                 //robot.set_push_position(robot.a_push_position() + PUSH_SPEED);
                 //}
             }
             if(gamepad2.y){
                 robot.push.setPosition(0.7);

             }


         }

         if (robot.push2 != null) {
             if (gamepad2.x) {
                 robot.push2.setPosition(0.0);
                 //robot.set_push2_position((robot.a_push2_position() - PUSH2_SPEED));
                 //} else if (gamepad2.b) {
                 //robot.set_push2_position(robot.a_push2_position() + PUSH2_SPEED);
                 //}
             }
             if(gamepad2.y){
                 robot.push2.setPosition(0.7);

             }


         }

         if (robot.grab != null) {
             if (gamepad2.a) {
                 robot.grab.setPosition(0.3);
                 //robot.set_push2_position((robot.a_push2_position() - PUSH2_SPEED));
                 //} else if (gamepad2.b) {
                 //robot.set_push2_position(robot.a_push2_position() + PUSH2_SPEED);
                 //}
             }
             if(gamepad2.b){
                 robot.grab.setPosition(0.0);

             }


         }

         if (robot.grab2 != null) {
             if (gamepad2.a) {
                 robot.grab2.setPosition(0.3);
                 //robot.set_push2_position((robot.a_push2_position() - PUSH2_SPEED));
                 //} else if (gamepad2.b) {
                 //robot.set_push2_position(robot.a_push2_position() + PUSH2_SPEED);
                 //}
             }
             if(gamepad2.b){
                 robot.grab2.setPosition(0.0);

             }


         }*/

           /*
           if(robot.wall != null){
               if(gamepad2.x){
                   robot.wall.setPosition(1.0);
               }

               if(gamepad2.y){
                   robot.wall.setPosition(0.0);
               }
           }

           if (robot.fly_wheel != null){
               if (gamepad2.b){
                   robot.fly_wheel.setDirection(DcMotor.Direction.REVERSE);
                   robot.fly_wheel.setPower(robot.fly_wheel_power);
               }
               else{
                   robot.fly_wheel.setPower(0.0);
               }
           }

           if (robot.ramp != null){
               if (gamepad2.b){
                   robot.ramp.setDirection(DcMotor.Direction.REVERSE);
                   robot.ramp.setPower(robot.ramp_power);
               }
               else {
                   robot.ramp.setPower(0.0);
               }
           }

           if (robot.intake_motor != null){
               if (gamepad2.a){
                   robot.intake_motor.setDirection(DcMotor.Direction.REVERSE);
                   robot.intake_motor.setPower(robot.intake_motor_power);
               } else {
                   robot.intake_motor.setPower(0.0);
               }
           }

           if (robot.intake2_motor != null){
               if (gamepad2.a){
                   robot.intake2_motor.setDirection(DcMotor.Direction.REVERSE);
                   robot.intake2_motor.setPower(robot.intake2_motor_power);
               }
               else{
                   robot.intake2_motor.setPower(0.0);
               }
           }
           */
           /*
           if(robot.vacuum1 != null){
               if(gamepad2.b) {
                   robot.vacuum1.setDirection(DcMotorSimple.Direction.FORWARD);
                   robot.vacuum1.setPower(robot.vacuum1_power);
               }
               else{
                   robot.vacuum1.setPower(0.0);
               }
           }

            */

           /*
           if(robot.vacuum2 != null){
               if(gamepad2.b) {
                   robot.vacuum2.setDirection(DcMotor.Direction.FORWARD);
                   robot.vacuum2.setPower(robot.vacuum2_power);
               }
               else{
                   robot.vacuum2.setPower(0.0);
               }
           }
           */
           /*
           if(robot.elevator != null){
               if(gamepad2.y) {
                   robot.elevator.setDirection(DcMotor.Direction.FORWARD);
                   robot.elevator.setPower(robot.elevator_power);
               }
               else if(gamepad2.a){
                   robot.elevator.setDirection(DcMotor.Direction.REVERSE);
               }
               else{
                   robot.elevator.setPower(0.0);
               }

           }

           if(robot.box != null){
               if(gamepad2.x)
               {
                   robot.box.setPosition(0.0);
               }
           }

           if(robot.gate != null)
           {
               if(gamepad1.x)
               {
                   robot.gate.setPosition(0.0);
               }
           }

           if (robot.guide != null) {
               if (gamepad1.y) {
                   robot.guide.setDirection(DcMotor.Direction.FORWARD);
                   robot.guide.setPower(robot.guide_power);
               } else {
                   robot.guide.setPower(0.0);
               }
           }
           if(robot.guide != null)
           {
               if(gamepad1.a){
                   robot.guide.setDirection(DcMotor.Direction.REVERSE);
                   robot.guide.setPower(robot.guide_power);
               } else {
                   robot.guide.setPower(0.0);
               }
           }

            */

            if (robot.shaft != null) {
                if (gamepad2.dpad_down) {
                    robot.shaft.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.shaft.setPower(robot.shaft_power);
                } else if (gamepad2.dpad_up) {
                    robot.shaft.setDirection(DcMotor.Direction.REVERSE);
                    robot.shaft.setPower(robot.shaft_power);
                } else {
                    robot.shaft.setPower(0.0);
                }
            }



           /*if(robot.hand1 != null && robot.hand2 != null) {
               if(gamepad2.x) {
                   robot.hand1.setPosition(0.0);
                   robot.hand2.setPosition(0.0);
               } else if (gamepad2.b) {
                   robot.hand1.setPosition(1.0);
                   robot.hand2.setPosition(-1.0); //test later on when hand ready
               }
           }

            */
            /*

            if (robot.hand1 != null) {
                //robot.hand1.setDirection(Servo.Direction.FORWARD);
                if (gamepad2.dpad_left) {
                    robot.hand1.setPosition(0.4);
                } else if (gamepad2.dpad_right) {
                    robot.hand1.setPosition(0.4);

                }
            }

             */

            if (robot.hand2 != null) {
                if (gamepad2.y) {
                    robot.hand2.setPosition(0.5);
                } else if (gamepad2.a) {
                    robot.hand2.setPosition(0.9);
                }
            }



         /*if (robot.intake_motor != null) {
             if (gamepad2.right_bumper) { // ball shoot
                 robot.intake_motor.setDirection(DcMotor.Direction.FORWARD); //might need to change to FORWARD/REVERSE
                 robot.intake_motor.setPower(robot.FLIP_POWER);
             } else if (gamepad2.right_trigger > 0) {
                 robot.intake_motor.setDirection(DcMotor.Direction.REVERSE);
                 robot.intake_motor.setPower(robot.FLIP_POWER);
             } else {
                 robot.intake_motor.setPower(0.0);

             }

         }*/
         /*telemetry.addData
                 ("01"
                         , "Left Drive Power: "
                                 + robot.a_left_front_drive_power()
                 );
         telemetry.addData
                 ("02"
                         , "Right Drive Power: "
                                 + robot.a_right_front_drive_power()
                 );
         telemetry.addData
                 ("03"
                         , "Left Drive Power: "
                                 + robot.a_left_back_drive_power()
                 );
         telemetry.addData
                 ("04"
                         , "Right Drive Power: "
                                 + robot.a_right_back_drive_power()
                 );

         telemetry.addData
                 ("10"
                         , "Push Position: "
                                 + robot.a_push_position()
                 );


         telemetry.addData
                 ("12"
                         , "Flip Power: "
                                 + robot.a_flip_power()
                 );


         telemetry.update();

         // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
         robot.waitForTick(40);*/
        }
    }
}








