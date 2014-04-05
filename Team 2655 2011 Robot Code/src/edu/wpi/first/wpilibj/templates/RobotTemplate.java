/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
/**
 * This function is run when the robot is first started up and should be used
 * for any initialization code. 1: A 2: B 3: X 4: Y 5: Left Bumper 6: Right
 * Bumper 7: Back 8: Start 9: Left Joystick 10: Right Joystick
 *
 * Axis Mappings #
 *
 * The axis on the controller return values between -1 and 1, and follow the
 * following mapping :
 *
 * 1: Left Stick X Axis Left:Negative Right: Positive 2: Left Stick Y Axis Up:
 * Negative Down: Positive 3: Triggers Left: Positive Right: Negative 4: Right
 * Stick X Axis Left: Negative Right: Positive 5: Right Stick Y Axis Up:
 * Negative Down: Positive 6: Directional Pad (Not recommended, buggy)
 */
public class RobotTemplate extends IterativeRobot {

    Talon leftMotor; //drive motors
    Talon rightMotor;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    //ALL DIRECTIONS RELATIVE TO ARM AWAY FROM DRIVER
    //RobotDrive(DIO1,DIO2)
    DriverStationLCD screen = DriverStationLCD.getInstance();
    DriverStationLCD.Line prline = DriverStationLCD.Line.kUser2;
    RobotDrive drive = new RobotDrive(1, 2); //1 > left 2 > right - moves robot
    RobotDrive arm = new RobotDrive(3, 4);// 3 > large arm 4 > small arm - moves arm
    RobotDrive endeffector = new RobotDrive(5, 6);//-5 +6-> in close to arm, +5 -6->out away from arm
    RobotDrive minibot = new RobotDrive(7, 8);// drives minibot launcher
    Joystick leftStick = new Joystick(1); //driver control left tire
    Joystick rightStick = new Joystick(2);//driver control right tire
    Joystick xbox = new Joystick(3); //second driver
    //Encoder(Cartridge SlotCRIO,DIO1,Cartridge SlotCRIO, DIO2)
    Encoder driveencoder1 = new Encoder(4, 7, 4, 8, true, CounterBase.EncodingType.k4X); // left tire
    //LIMIT SWITCHES
    //DigitalInput(DIO)
    DigitalInput left = new DigitalInput(4); // digital inputs for line tracking sensors
    DigitalInput middle = new DigitalInput(5);
    DigitalInput right = new DigitalInput(6);
    DigitalInput shortarm = new DigitalInput(1);    //short arm moving up
    DigitalInput shortarm2 = new DigitalInput(2);   //short arm moving dwn
    DigitalInput longarm = new DigitalInput(3);     //Does not exist
    DriverStation ds;
    double defaultSteeringGain = 0.45; // the default value for the steering gain
    int smallarmcount = 47000; // max distance for short arm
    int largearmcount = 35000; // max distance for large arm
    int smallarm = 0; // keeps count of small arm movement
    int largearm = 0; // keeps count of large arm movement
    int drive1max = 3500;// set max distance for left tire
    int drive1count = 0; // start encoders at value o
    int endprogram;
    int backup1;
    double linespeed = 0.5;

    // Relay minibot3 = new Relay (9);
    //DriverStationLCD screen = new DriverStationLCD();
    /**
     * This function is called periodically during autonomous autonomous
     */
    public void robotInit() {
        ds = DriverStation.getInstance();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        double motorspeed = -0.5;
        int binaryValue; // a single binary value of the three line tracking
        // sensors
        int previousValue = 0; // the binary value from the previous loop
        double steeringGain; // the amount of steering correction to apply

        // the power profiles for the straight and forked robot path. They are
        // different to let the robot drive more slowly as the robot approaches
        // the fork on the forked line case.
        double forkProfile[] = {0.70, 0.70, 0.55, 0.60, 0.60, 0.50, 0.40, 0.00};
        double straightProfile[] = {0.7, 0.7, 0.6, 0.6, 0.35, 0.35, 0.35, 0.0};
        double powerProfile[];   // the selected power profile
        // set the straightLine and left-right variables depending on chosen path
        boolean straightLine = ds.getDigitalIn(1);
        powerProfile = (straightLine) ? straightProfile : forkProfile;
        double stopTime = (straightLine) ? 2.0 : 4.0; // when the robot should look for end
        boolean goLeft = !ds.getDigitalIn(2) && !straightLine;
        System.out.println("StraightLine: " + straightLine);
        System.out.println("GoingLeft: " + goLeft);

        boolean atCross = false; // if robot has arrived at end

        // time the path over the line
        Timer timer = new Timer();
        timer.start();
        timer.reset();

        int oldTimeInSeconds = -1;
        double time;
        double speed, turn;

        //     driveencoder1.start();
        //     drive1count = -driveencoder1.get();
        // loop until robot reaches "T" at end or 8 seconds has past
        //while ((time = timer.get()) < 8.0 && !atCross) {
        while (drive1count < drive1max) {
            time = 000000000;
            int timeInSeconds = (int) time;
            // read the sensors
            int leftValue = left.get() ? 0 : 1;
            int middleValue = middle.get() ? 0 : 1;
            int rightValue = right.get() ? 0 : 1;
            // compute the single value from the 3 sensors. Notice that the bits
            // for the outside sensors are flipped depending on left or right
            // fork. Also the sign of the steering direction is different for left/right.
            if (goLeft) {
                binaryValue = leftValue * 4 + middleValue * 2 + rightValue;
                steeringGain = -defaultSteeringGain;
            } else {
                binaryValue = rightValue * 4 + middleValue * 2 + leftValue;
                steeringGain = defaultSteeringGain;
            }

            // get the default speed and turn rate at this time
            speed = powerProfile[timeInSeconds];
            turn = 0;

            // different cases for different line tracking sensor readings
            switch (binaryValue) {

                case 0:  // all sensors off (off line) use last known case to determine which way to turn back onto the line
                    //speed = 0.0;

                    if (previousValue == 0 || previousValue == 1 || previousValue == 3) {
                        speed = 0.2;
                        turn = steeringGain;
                    } else {
                        speed = 0.2;
                        turn = -steeringGain;
                    }
                    break;

                case 1:  // right sensor on (turn right)
                    //if (time > stopTime) {
                    speed = linespeed;
                    turn = steeringGain;
                    //steeringGain;
                    //}
                    break;

                case 2:  // center line don't turn
                    speed = linespeed;
                    turn = 0;
                    break;

                case 3:  // middle and right sensors on (turn right)
                    //                 if (time > stopTime) {

                    speed = linespeed;
                    turn = steeringGain;
                    //steeringGain;
                    //               }
                    break;

                case 4:  // left sensor turn left
                    //             if (time > stopTime) {

                    speed = linespeed;
                    turn = -steeringGain;
                    //-steeringGain;
                    //           }
                    break;

                case 5:  // outside sensors on (maybe past cross - don't move)
                    //         if (time > stopTime) {

                    speed = 0;
                    turn = 0;
                    //       }
                    break;

                case 6:  // lft and middle sensors on (turn left)
                    //   if (time > stopTime) {

                    speed = linespeed;
                    turn = -steeringGain;
                    //-steeringGain;
                    //     }
                    break;

                case 7:  // all sensors on (don't move - maybe at cross)
                    // if (time > stopTime) {

                    speed = 0;
                    turn = 0;
                    //   }
                    break;
                //default:  // all other cases do nothing (Shouldn't ever happen)
                //  turn = 0;
            }

            // print current status for debugging
            if (binaryValue != previousValue) {
                //System.out.println("Time: " + time + " Sensor: " + binaryValue + " speed: " + speed + " turn: " + turn + " atCross: " + atCross);
                screen.println(prline, 2, Double.toString(speed));
                screen.println(prline, 10, Double.toString(turn));
                screen.println(prline, 18, Integer.toString(binaryValue));
                screen.updateLCD();
            }

            // set the robot speed and direction
            drive.arcadeDrive(-speed, -turn);

            if (binaryValue != 0) {
                previousValue = binaryValue;
            }
            oldTimeInSeconds = timeInSeconds;

            Timer.delay(0.01);
        }
        drive.tankDrive(0, 0);
        while (largearm < largearmcount) {

            arm.tankDrive(1.0, 0.0);
            largearm = largearm + 1;

        }
        //drives the small arm motors untill (smallarmcount) is reached
        while (smallarm < smallarmcount) {

            arm.tankDrive(0.0, 0.5);
            smallarm = smallarm + 1;

        }
        arm.tankDrive(0.0, 0.0);
        endprogram = 0;
        backup1 = 0;
        while ((endprogram < 5000) && (backup1 < 3000)) {
            endeffector.tankDrive(-1.0, 1.0);
            endprogram = endprogram + 1;
            drive.tankDrive(0.2, 0.2);
            backup1 = backup1 + 1;
        }
        // Done with loop - stop the robot. Robot ought to be at the end of the line
        drive.arcadeDrive(0, 0);
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //getWatchdog().setEnabled(false);
        //holds motor values (motor1-motor2)
        double motor1;
        double motor2;
        // statrs the Teleop mode and contenues untill told to stop
        if (isOperatorControl() == true && isEnabled()) { // loop until change
            // updates the controler value ((this is a untested function!!!!!!!))
            drive.tankDrive(leftStick.getRawAxis(2), rightStick.getRawAxis(2)); // drive with joysticks
            if (xbox.getRawAxis(2) < 0 && longarm.get() == true) {
                motor1 = (xbox.getRawAxis(2) / 1.5);
            } // xbox.getRawAxis(2) == left analog stick x axis
            else if (xbox.getRawAxis(2) > 0 /*&& longarm2.get() == true*/) {
                motor1 = xbox.getRawAxis(2);
            } else {
                motor1 = 0;
            }
            if (xbox.getRawAxis(5) > 0 && shortarm.get() == true) {
                motor2 = xbox.getRawAxis(5);
            } // xbox.getRawAxis(5) == right analog stick x axis
            else if (xbox.getRawAxis(5) < 0 && shortarm2.get() == true) {
                motor2 = xbox.getRawAxis(5);
            } else {
                motor2 = 0;
            }
            //orignaly devided by 1.5
            arm.tankDrive(motor1, motor2);
            // arm.tankDrive(xbox.getRawAxis(2), xbox.getRawAxis(5))
            if ((xbox.getRawButton(5) && xbox.getRawButton(6)) || (!xbox.getRawButton(5) && !xbox.getRawButton(6)) || (leftStick.getRawButton(10))) { // buttons
                if (xbox.getRawAxis(3) != 0) { //triggers
                    endeffector.tankDrive(xbox.getRawAxis(3), xbox.getRawAxis(3));
                } // trigers
                else {
                    endeffector.tankDrive(0.0, 0.0);
                }
            } // triggers/buttons = not pushed = no power to the motors
            else if (xbox.getRawButton(5) == true) { //leftbumper = pull tube in
                endeffector.tankDrive(1.0, -1.0);
            } else if (xbox.getRawButton(6) == true) { //rightbumper = push tube out
                endeffector.tankDrive(-1.0, 1.0);
            }
            //leftStick.getRawButton(10) driver deploys mini bot

            while ((xbox.getRawButton(2)) == true || (xbox.getRawButton(3)) == true || (xbox.getRawButton(4)) == true) {

                // runs minibot out on the launcher
                if (xbox.getRawButton(3) == true) {
                    minibot.tankDrive(-1.0, 0.0);
                    //minibot2.setDirection(Relay.Direction.kReverse);
                    //minibot2.set(Relay.Value.kOn);
                } // lays down launcher
                else if (xbox.getRawButton(2) == true) {
                    minibot.tankDrive(0.0, 0.4);
                    // minibot1.setDirection(Relay.Direction.kReverse);
                    //minibot1.set(Relay.Value.kOn);
                } else if (xbox.getRawButton(4) == true) {
                    minibot.tankDrive(0.0, -0.4);
                    //minibot1.setDirection(Relay.Direction.kForward);
                    //minibot1.set(Relay.Value.kOn);
                }
            }
            //minibot.tankDrive(0.0, 0.0);
            minibot.tankDrive(0.0, 0.0);
            //  minibot1.set(Relay.Value.kOff);
            //  minibot2.set(Relay.Value.kOff);

        }
    }

}
