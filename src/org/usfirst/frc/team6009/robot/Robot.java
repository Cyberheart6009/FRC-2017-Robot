// _______ Drive Style		2017 Robot
package org.usfirst.frc.team6009.robot;

//This is where all the libraries will get imported
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
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

	// Constant defining encoder turns per inch of robot travel
	final static double ENCODER_COUNTS_PER_INCH = 13.1;

	// Define Joysticks, Victors, Limit switches, Etc
	
	final String centerpeg = "Center Peg";
	final String followtape = "Follow tape";
	final String DriveStraight = "Straight Drive";
	final String vision = "Vision";
	final String altLeftPeg = "Alt Left Peg";
	final String altRightPeg = "Alt Right Peg";
	final String VisionLeftPeg = "Vision Left Peg";
	final String VisionRightPeg = "Vision Right Peg";
	String autoSelected;
	SendableChooser<String> chooser;

	DigitalOutput light;
	SpeedController leftFront, leftBack, rightFront, rightBack, hopper, climber, launcher;
	Encoder leftEncoder, rightEncoder;
	
	// Driver Joystick
	Joystick driver;
	
	// RobotDrive Object
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
	
	// Constant used for calculating degrees to turn for vision
	double degreesperpixel = 0.1625;

	// Auto Variables
	public enum Step { STRAIGHT, STRAIGHT_PAUSE, TURN, TURN_PAUSE,VISION_ALIGN, HANG, VISION_HANG, DONE };
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
		chooser.addObject("Vision (Actually Vision)", vision);
		chooser.addObject("Alt Left Peg", altLeftPeg);
		chooser.addObject("Alt Right Peg", altRightPeg);
		chooser.addObject("Vision Left Peg", VisionLeftPeg);
		chooser.addObject("Vision Right Peg", VisionRightPeg);

		SmartDashboard.putData("Auto choices", chooser);

		// Drive Train motors			We are assigning the motors to the motor controllers *Called Victors*
		leftFront = new Victor(0);
		leftBack = new Victor(1);
		rightFront = new Victor (2);
		rightBack = new Victor (3);

		// The right side motors are inverted			We did this due to the motors facing the opposite way
		//												In real life, this prevents us from always having to put
		//												A negative value in for the speed in order to drive forward
		rightFront.setInverted(true);
		rightBack.setInverted(true);
		
		// Servos		Never Actually used
		kicker = new Servo(7);
		lifter = new Servo(8);
		
		leftEncoder = new Encoder(0, 1);
		rightEncoder = new Encoder(2, 3);

		//launcher = new Victor (5);

		climber = new Victor(6);

		driver = new Joystick (0);
		//operator = new Joystick(1);
		
		// Starts the default USB camera streaming service
		CameraServer.getInstance().startAutomaticCapture();

		// Defines our main/ only RobotDrive Object
		// This is used in the default drive functions that will be seen in the teleop mode
		chassis = new RobotDrive(leftFront, leftBack, rightFront, rightBack);
		
		// Defaults gyro
		gyroscope = new ADXRS450_Gyro();
		
		// Network table definition
		table = NetworkTable.getTable("GRIP/myContoursReport"); // rename - GRIP/myContoursReport 

		/*			Network Tables
		 * Network tables is the method in which we communicate with the RoboRio
		 * This happens over the local network created by the Radio that connects the Rio with other Devices on the network
		 * We use this 'table' in particular to communicate with the DS laptop which is processing
		 * images from the camera and filtering out colors for vision tracking
		 * 			*Vision is still experimental for us*
		 */
		
		
		
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
		
		// setup loop variable as well as the smartdashboard				- Very old

		autoSelected = (String) chooser.getSelected();
		//autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		autoLoopCounter = 0;

		resetEncoders(); 	// Homemade function which we will explore later
		autoStep = Step.STRAIGHT;				// Probably redundant since it is defined above
		gyroscope.reset();  // Reset the gyro so current heading is always 0
		
	}

	/**
	 * This function is called periodically (~20ms) during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		// Another homemade function that pushed data gathered and displays it on smartdashboard on the DS
		updateSmartDashboard();
		
		// Setup this so that we could preserve our other auto modes
		if (autoSelected.equalsIgnoreCase(altLeftPeg) || autoSelected.equalsIgnoreCase(altRightPeg)) {

			// Calculate the distance since the last reset of the encoders
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				// call driveStraight(heading, speed)
				driveStraight(0, .4);
				
				// Check distance in inches
				if (distance > 70) {
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
		
		if (autoSelected.equalsIgnoreCase(VisionLeftPeg) || autoSelected.equalsIgnoreCase(VisionRightPeg)) {
			// Calculate the distance since the last reset of the encoders
			double distance = getDistance();
			switch (autoStep) {
			case STRAIGHT:
				// call driveStraight(heading, speed)
				driveStraight(0, .4);
				
				// Check distance in inches
				if (distance > 70) {
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
				
				if (autoSelected == VisionLeftPeg){
					if (turnRight(60)) {
						timerStart = System.currentTimeMillis();
						autoStep = Step.TURN_PAUSE;
					}
				}
				// If its VisionRightPeg
				else{
					
					turnLeft(-60);
					timerStart = System.currentTimeMillis();
					autoStep = Step.TURN_PAUSE;
				}
				break;

			case TURN_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 500) {
					autoStep = Step.VISION_ALIGN;
					resetEncoders();
				}
				break;
				
			case VISION_ALIGN:
				//code
				double vision_offset = visionAngle();
				System.out.println("OFFSET = " + vision_offset);
				if (vision_offset > 0){
					// turn right
					turnRight(gyroscope.getAngle() + vision_offset);
					autoStep = Step.VISION_HANG;
				}
				else if (vision_offset < 0){
					// turn left
					turnLeft(gyroscope.getAngle() + vision_offset);
					autoStep = Step.VISION_HANG;
				}
				else{
					// if it is dead on
					autoStep = Step.HANG;
				}
				resetEncoders();
				break;
				
			case HANG:
				vision_offset = gyroscope.getAngle();
				if (autoSelected == VisionRightPeg) {
					driveStraight(vision_offset, .3);
				} else {
					driveStraight(vision_offset, .3);
				}

				if (distance > 64.5) {
					stop();
					autoStep = Step.DONE;
				}
				break;
				
			case VISION_HANG:
				// drive straight
				double target_angle = gyroscope.getAngle();
				driveStraight(target_angle, 0.3);
				
				if (distance > 64.5){
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


		// Original Auto modes, Some from before we had encoders
		switch(autoSelected){

		// Updated to use encoders
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

			// First Auto mode we used, 
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
			
			
		// Experimental mode to test if vision is even possible with current resources
		// Supposed to towards a piece of tape but ends up just turning in place
						// when we demo think of how we can fix this
		case vision:
			double DriveAngle = gyroscope.getAngle()%360;
			boolean vision_drive = false;
			// Drive Straight if no vision target is found, or adjust to follow vision target
			// driveStraight(heading, speed);
			System.out.println(visionAngle());
			distance = getDistance();
			if (distance > 500.0 ){
				stop();
			}
			
			else if (visionAngle() == 0 && vision_drive == true){
				DriveAngle = gyroscope.getAngle()%360;
				vision_drive = false;
				driveStraight(DriveAngle, 0.2);
				System.out.println(DriveAngle);
			}
			else if (visionAngle() == 0 && vision_drive == false){
				// Do nothing (below does the same thing)
			}
			else{ 		// AKA if visionAngle() != 0
				DriveAngle = gyroscope.getAngle() + visionAngle();
				driveStraight(DriveAngle, 0.2);
				System.out.println("Drive Angle = " + DriveAngle);
			}
			
			//FIXME: may not work all the time when there is no vision input
			// ^ Actually fixed but this is a good example of how to keep track of issues
			// TODO: is another one of these commands - can see beside scrollbar
			
			break;
			
		}

	}

	/**
	 * This function is called periodically (~20ms) when the robot is disabled
	 */
	@Override
	public void disabledPeriodic() {
		// This was added in so that we can see encoder and gyro values without the robot being enabled
		// This is much safer plus very usefull
		updateSmartDashboard();
	}
	
	// "Homemade" function - uses buit in encoder value reset method
	// probably the most basic function but easier than writing it everytime
	private void resetEncoders() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	@Override
	public void teleopInit(){
		// Simply resets encoders on the start of teleop - tbh its pretty useless since they are not used in teleop
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

		// all Boolean values (True or False) that check to see if the buttons are pressed
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

		
		// This next section allows us to use one stick for driving forwards 
		// and the other stick for driving backwards one a button is pressed to select the mode
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
		// used to test our active gear slot. basically useless code
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
		
		
		// Used to turn the climber when the Right Bumper is pressed
		if (lbumperButton == true){
			//launcher.set(1.0);
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
	
	// Original VisionDrive wrote during a competition 
	// Don't think we've even tested this before
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
	
	// 			** MAGIC AREA **		//
	// This is the function that allows the robot to drive straight no matter what
	// It automatically corrects itself and stays locked onto the set angle
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

	private double visionAngle(){
		double[] defaultValue = new double[0];
		double xTotal = 0;
		
		// Grabs data from GRIP through network tables and adds it to array
		double[] xValues = table.getNumberArray("centerX", defaultValue);
		
		//			 prints only the first number in the GRIP array
		//System.out.println("First X Value: " + xValues[0]);
		
		// 			 Prints all values in the array
		/*
		System.out.print("X Values Found: ");
		for (double xval : xValues){
			System.out.print(xval + " ");
		}
		System.out.println();*/
		
		
		//		CALCULATE ANGLE TO TURN BASED ON THE POSITION OF THE TARGET
		// 		Attempt 1: Average x values to find the center of the values
		
		// check how many x values were found
		int xArrayLength = xValues.length;
		
		// If more than 1 value then continue with angle calculation
		if (xArrayLength > 0){
			// First Average Values
			for (double xval : xValues){
				xTotal += xval;
			}
			double xAverage = (xTotal/xArrayLength);
			System.out.println("Average Values: " + xAverage);
			// below calculates offset to center using camera width (480px)
			// if > 0: turn right, if < 0 : turn left
			double degreeOffset = -(((176/2) - xAverage) * degreesperpixel);	
			// Get angle the robot is at now
			double currentAngle = gyroscope.getAngle() %360;
			
			return degreeOffset;	
		}
		// If no values are found and the array is empty, then return 0
		else{
			System.out.println("Empty Array length: " + xArrayLength);
			return 0;
		}
	}

	// This is the function that displays the info in smartdashboard 
	// This is seen on the DS
	private void updateSmartDashboard() {
		
		SmartDashboard.putData("Gyro", gyroscope);
		SmartDashboard.putNumber("Gyro Angle", gyroscope.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyroscope.getRate());

		SmartDashboard.putNumber("Left Encoder Count", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder Count", rightEncoder.get());
		SmartDashboard.putNumber("Encoder Distance", getDistance());
	}

}
