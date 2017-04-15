// _______ Drive Style
package org.usfirst.frc.team6009.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.vision.VisionRunner;
//import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
//This is where all the libraries will get imported
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**			`	
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot{

	final static double ENCODER_COUNTS_PER_INCH = 13.1;

	// Define Joysticks, Victors, Limit switches, Etc
	final String centerpeg = "Center Peg";
	final String followtape = "Follow tape";
	final String DriveStraight = "Straight Drive";
	final String vision = "Vision";
	final String altLeftPeg = "Alt Left Peg";
	final String altRightPeg = "Alt Right Peg";
	String autoSelected;
	SendableChooser<String> chooser;

	DigitalOutput light;
	SpeedController leftFront, leftBack, rightFront, rightBack, hopper, climber, launcher;
	Encoder leftEncoder, rightEncoder;
	
	Joystick driver;
	//Joystick operator;
	RobotDrive chassis;
	
	// Servos
	Servo kicker, lifter;

	ADXRS450_Gyro gyroscope;
	CameraServer server;
	//VisionThread visionThread;

	int autoLoopCounter = 0;
	int x = 0;

	NetworkTable table;

	double kP = 0.03; // angle multiplied by kP to scale it for the speed of the drive
	double centerX, offset = 0;

	boolean aButton, bButton, xButton, yButton, startButton, selectButton, upButton, downButton, lbumperButton, rbumperButton;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	double degreesperpixel = 0.1625;

	// Auto Variables
	public enum Step { STRAIGHT, STRAIGHT_PAUSE, TURN, TURN_PAUSE, HANG, DONE };
	public Step autoStep = Step.STRAIGHT;
	public long timerStart;


	/**
	 * Called once when the robot is intialized
	 * 
	 * Setup the chassis,motors, etc
	 */
	@Override
	public void robotInit() {
		chooser = new SendableChooser<String>();
		chooser.addObject("Center Peg", centerpeg);
		chooser.addObject("Follow Tape(Do)", followtape);
		chooser.addObject("DriveStraight", DriveStraight);
		chooser.addObject("Vision", vision);
		chooser.addObject("Alt Left Peg", altLeftPeg);
		chooser.addObject("Alt Right Peg", altRightPeg);

		SmartDashboard.putData("Auto choices", chooser);

		// Drive Train motors
		leftFront = new Victor(0);
		leftBack = new Victor(1);
		rightFront = new Victor (2);
		rightBack = new Victor (3);

		// The right side motors are inverted
		rightFront.setInverted(true);
		rightBack.setInverted(true);
		
		// Servos
		kicker = new Servo(7);
		lifter = new Servo(8);
		
		leftEncoder = new Encoder(0, 1);
		rightEncoder = new Encoder(2, 3);

		//launcher = new Victor (5);

		climber = new Victor(6);

		driver = new Joystick (0);
		//operator = new Joystick(1);
		

		CameraServer.getInstance().startAutomaticCapture();

		chassis = new RobotDrive(leftFront, leftBack, rightFront, rightBack);

		gyroscope = new ADXRS450_Gyro();
		
		table = NetworkTable.getTable("GRIP/myContoursReport"); // rename - GRIP/myContoursReport 


		//double centerX = 0;
		// TODO: Uncomment line below for vision
		//double centerX = table.getNumber("centerX", 160);
	}


	/**
	 * Initialize the Autonomous.  
	 * 
	 * This code runs once at the beginning of autonomous.
	 */
	@Override
	public void autonomousInit() {

		// Initializes autonomous mode
		// setup loop variable as well as the smartdashboard

		autoSelected = (String) chooser.getSelected();
		//autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		autoLoopCounter = 0;

		resetEncoders();
		autoStep = Step.STRAIGHT;
		gyroscope.reset();  // Reset the gyro so current heading is always 0
		
	}

	/**
	 * This function is called periodically (~20ms) during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		updateSmartDashboard();
		
		if (autoSelected.equalsIgnoreCase(altLeftPeg) || autoSelected.equalsIgnoreCase(altRightPeg)) {

			// Calculate the distance since the last reset of the encoders
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				// call driveStraight(heading, speed)
				driveStraight(0, .4);
				
				// Check distance in inches
				if (distance > 72) {
					stop();
					timerStart = System.currentTimeMillis();
					autoStep = Step.STRAIGHT_PAUSE;
				}
				break;
			case STRAIGHT_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 500) {
					autoStep = Step.TURN;
				}
				System.out.println(autoStep);
				break;

			case TURN:
				
				if (autoSelected == altLeftPeg){
					if (turnRight(60)) {
						timerStart = System.currentTimeMillis();
						autoStep = Step.TURN_PAUSE;
					}
				}
				else{
					
					turnLeft(-60);
					timerStart = System.currentTimeMillis();
					autoStep = Step.TURN_PAUSE;
				}
				break;

			case TURN_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 500) {
					autoStep = Step.HANG;
					resetEncoders();
				}
				break;

			case HANG:

				if (autoSelected == altRightPeg) {
					driveStraight(-60, .3);
				} else {
					driveStraight(60, .3);
				}

				if (distance > 64.5) {
					stop();
					autoStep = Step.DONE;
				}
				break;

			case DONE:
				break;
			}

			return;
		}
		
		
		//	Above is the McMaster side auto code that we know works
		//	We're switching back temporarily to ensure side auto works
		//	
		//	Below is the 'updated' auto modes
		//	Turn speed is fixed but tuning is still needed
		
/*
		if (autoSelected.equalsIgnoreCase(altLeftPeg)){
			System.out.println("LEFT PEG");
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				// call driveStraight(heading, speed)
				driveStraight(0, .6);
				System.out.println(autoStep);
				// Check distance in inches
				if (distance > 50) {
					stop();
					timerStart = System.currentTimeMillis();
					autoStep = Step.STRAIGHT_PAUSE;
					
				}
				break;
			case STRAIGHT_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 250) {
					autoStep = Step.TURN;
				}
				System.out.println(autoStep);
				break;

			case TURN:
				if (turnRight(60)) {
					timerStart = System.currentTimeMillis();
					autoStep = Step.TURN_PAUSE;
					System.out.println("LEFT PEG");
				}
				break;

			case TURN_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 250) {
					autoStep = Step.HANG;
					resetEncoders();
				}
				break;

			case HANG:
				driveStraight(60, .35);

				if (distance > 59) {
					stop();
					autoStep = Step.DONE;
				}
				break;

			case DONE:
				break;
			}

		}
		
		else if (autoSelected.equalsIgnoreCase(altRightPeg)){
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				// call driveStraight(heading, speed)
				driveStraight(0, .6);
				
				// Check distance in inches L$ wa here
				if (distance > 50) {
					stop();
					timerStart = System.currentTimeMillis();
					autoStep = Step.STRAIGHT_PAUSE;
				}
				break;
			case STRAIGHT_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 250) {
					autoStep = Step.TURN;
				}
				System.out.println(autoStep);
				break;

			case TURN:
				if (turnLeft(-60)) {
					timerStart = System.currentTimeMillis();
					autoStep = Step.TURN_PAUSE;
					System.out.println("LEFT PEG");
				}
				break;

			case TURN_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 250) {
					autoStep = Step.HANG;
					resetEncoders();
				}
				break;

			case HANG:
				driveStraight(-60, .35);

				if (distance > 59) {
					stop();
					autoStep = Step.DONE;
				}
				break;

			case DONE:
				break;
			}

			return;
		}*/
		
		// If not the altLeftPeg or altRightPeg, then keep the code the same as before.

		// This is where the autonomous code goes. Setup switch cases to choose between the modes using smartdashboard

		//double centerX = 0;


		switch(autoSelected){

		
		case centerpeg:
			double distance = getDistance();
			driveStraight(0, .4);
			if (distance > 71) {
				stop();
				autoStep = Step.DONE;
			}
			break;
		case followtape:
			break;

		case DriveStraight:
			if (autoLoopCounter < 45){
				leftBack.set(0.8);
				rightBack.set(-0.8);
				//climber.set(-1.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 50){
				leftBack.set(0.0);
				rightBack.set(0.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 77){
				leftBack.set(-0.8);
				rightBack.set(0.8);
				autoLoopCounter++;
			}
			else{
				leftBack.set(0.0);
				rightBack.set(0.0);
			}
			break;
		case vision:
			distance = getDistance();
			driveStraight(0, 0.3);
			if (distance > 100.0 ){
				stop();
			}
			break;
			
		}

	}

	/**
	 * This function is called periodically (~20ms) when the robot is disabled
	 */
	@Override
	public void disabledPeriodic() {
		updateSmartDashboard();
	}
	
	private void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	@Override
	public void teleopInit(){
		resetEncoders();
	}
	
	/**
	 * This function is called periodically during operator control
	 */	
	public void teleopPeriodic() {
		// The teleopPeriodic routine is called every ~20ms
		
		// Reset the encoders at the start of teleop
		// only here for testing purposes as a way to reset the encoder count
		
		updateSmartDashboard();

		aButton = driver.getRawButton(1);
		bButton = driver.getRawButton(2);
		xButton = driver.getRawButton(3);
		yButton = driver.getRawButton(4);
		lbumperButton = driver.getRawButton(5);
		rbumperButton = driver.getRawButton(6);


		// Loop that is called while teleop is running
		// Use this to setup drive style (Ex. Tank/Arcade/etc)
		// Use this to map button inputs to motors, etc

		
		//chassis.arcadeDrive((-driver.getY()), (-driver.getX()));		

		selectButton = driver.getRawButton(7);
		startButton = driver.getRawButton(8);
		
		if (selectButton == true){
			x = 0;
		}
		else if (startButton == true){
			x = 1;
		}

		if (x == 0){
			//Forwards Driving
			chassis.arcadeDrive((driver.getX()), (driver.getY()));
			}
		else if (x == 1){
			//Backwards Driving
			chassis.arcadeDrive(driver.getRawAxis(4), -(driver.getRawAxis(5)));
		}
		
		
		
	// TODO: Remove light if we're not using the circuit

		if (aButton == true){
			double lifterAngle = lifter.getAngle();
			lifter.setAngle(lifterAngle + 4);
			//lifter.setAngle(-45);

		}
		else if (bButton == true) {
			double lifterAngle = lifter.getAngle();
			lifter.setAngle(lifterAngle - 4);
			//lifter.setAngle(0);
		}
		/*
		if (xButton == true){
			lifter.setAngle(60);
		}
		else if (yButton == true){
			lifter.setAngle(0);
		}*/
		
		if (lbumperButton == true){
			launcher.set(1.0);
		}
		else if (rbumperButton == true){
			climber.set(-1.0);
			//Timer.delay(0.5);
		}
		else {
			climber.set(0.0);
			//launcher.set(0.0);
		}

		

	}


	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		updateSmartDashboard();
	}
/*
	private void visionDrive(double speed){
		double currentAngle = gyroscope.getAngle()%360.0;
		double centerX = table.getNumber("centerX", 160);
		
		double offset = (160 - centerX) * degreesperpixel;	// If peg is to the right then this value will
															// equal a negative number
		double heading = currentAngle - offset;
		
		double error = heading - currentAngle;

		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;
		
		// adjust the motor speed based on the compass error
		if (error < 0) {
			// turn left
			// slow down the left motors
			leftSpeed += error * kP;
		}
		else {
			// turn right
			// Slow down right motors
			rightSpeed -= error * kP;
		}
	
		// set the motors based on the inputted speed
		leftBack.set(leftSpeed);
		leftFront.set(leftSpeed);
		rightBack.set(rightSpeed);
		rightFront.set(rightSpeed);
		
	}
*/
	private void driveStraight(double heading, double speed) {
		// get the current heading and calculate a heading error
		double currentAngle = gyroscope.getAngle()%360.0;
		System.out.println("driveStraight");
		double error = heading - currentAngle;

		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;

		// FIXME: This code can make the robot turn very 
		//        quickly if it is not pointed close to the
		//        correct direction.  I bet this is the 
		//        problem you are experiencing.
		//        I think if you correct the state machine
		//        if statements above, you will be able to 
		//        control the turning speed.
		
		// adjust the motor speed based on the compass error
		if (error < 0) {
			// turn left
			// slow down the left motors
			leftSpeed += error * kP;
		}
		else {
			// turn right
			// Slow down right motors
			rightSpeed -= error * kP;
		}
	
		// set the motors based on the inputted speed
		leftBack.set(leftSpeed);
		leftFront.set(leftSpeed);
		rightBack.set(rightSpeed);
		rightFront.set(rightSpeed);
	}
	
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
	}
	
	//slow motor speeds while turning
	
	private boolean turnRight(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(0.35);
		leftFront.set(0.35);
		rightBack.set(-0.35);
		rightFront.set(-0.35);

		double currentAngle = gyroscope.getAngle();
		if (currentAngle >= targetAngle - 10){
			return true;
		}
		return false;
	}

	private boolean turnLeft(double targetAngle){
		// We want to turn in place to 60 degrees 
		leftBack.set(-0.35);
		leftFront.set(-0.35);
		rightBack.set(0.35);
		rightFront.set(0.35);

		double currentAngle = gyroscope.getAngle();
		if (currentAngle <= targetAngle + 10){
			return true;
		}
		return false;
	}

	private double getDistance() {
		return ((double)(leftEncoder.get() + rightEncoder.get())) / (ENCODER_COUNTS_PER_INCH * 2);
	}


	private void updateSmartDashboard() {
		
		SmartDashboard.putData("Gyro", gyroscope);
		SmartDashboard.putNumber("Gyro Angle", gyroscope.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyroscope.getRate());

		SmartDashboard.putNumber("Left Encoder Count", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder Count", rightEncoder.get());
		SmartDashboard.putNumber("Encoder Distance", getDistance());
	}

}
