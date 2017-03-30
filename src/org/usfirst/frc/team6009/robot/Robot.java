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

	final static double ENCODER_COUNTS_PER_INCH = 1.0;

	// Define Joysticks, Victors, Limit switches, Etc
	final String rightpeg = "Right Peg";
	final String centerpeg = "Center Peg";
	final String leftpeg = "Left Peg";
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

	// FIXME: declare the variable using the full name of the gyro class. - FIXED
	ADXRS450_Gyro gyroscope;
	CameraServer server;
	//VisionThread visionThread;

	int autoLoopCounter = 0;
	int x = 0;

	NetworkTable table;

	double kP = 0.03; // angle multiplied by kP to scale it for the speed of the drive
	double angle;

	boolean aButton, bButton, xButton, yButton, startButton, selectButton, upButton, downButton, lbumperButton, rbumperButton;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	double centerX = 0;
	double centerY = 0;
	double width = 0; 
	double area = 0;
	double height = 0;

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
		//light.set(true);

		chooser = new SendableChooser<String>();
		chooser.addDefault("Right Peg", rightpeg);
		chooser.addObject("Center Peg", centerpeg);
		chooser.addObject("Left Peg", leftpeg);
		chooser.addObject("Follow Tape", followtape);
		chooser.addObject("DriveStraight", DriveStraight);
		chooser.addObject("Vision", vision);
		chooser.addObject("Alt Left Peg", altLeftPeg);
		chooser.addObject("Alt Right Peg", altRightPeg);

		SmartDashboard.putData("Auto choices", chooser);

		leftFront = new Victor(0);
		leftBack = new Victor(1);

		rightFront = new Victor (2);
		rightBack = new Victor (3);

		leftEncoder = new Encoder(0, 1);
		rightEncoder = new Encoder(2, 3);

		launcher = new Victor (5);
		//launcher = new Spark (4); 

		climber = new Victor(6);

		driver = new Joystick (0);

		//operator = new Joystick(1);
		light = new DigitalOutput(0);

		CameraServer.getInstance().startAutomaticCapture();
		//server.startAutomaticCapture("cam0");

		chassis = new RobotDrive(leftFront, leftBack, rightFront, rightBack);

		gyroscope = new ADXRS450_Gyro(); 
		//gyroscope.calibrate();

		table = NetworkTable.getTable("GRIP/myContoursReport"); // rename - GRIP/myContoursReport 


		double centerX = 0;
		double centerY = 0;
		double width = 0; 
		double area = 0;
		double height = 0;


		table.putNumber("CenterX", centerX);
		table.putNumber("CenterY", centerY);
		table.putNumber("Width", width);
		table.putNumber("Area", area);
		table.putNumber("Height", height);
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

		leftEncoder.reset();
		rightEncoder.reset();
		autoStep = Step.STRAIGHT;
		gyroscope.reset();  // Reset the gyro so current heading is always 0
		
		// reset the gyr FIXME:  - FIXED
		
	}

	/**
	 * This function is called periodically (~20ms) during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		updateSmartDashboard();

		if (autoSelected.equalsIgnoreCase(altLeftPeg) || autoSelected == altRightPeg) {

			// Calculate the distance since the last reset of the encoders
			double distance = getDistance();

			switch (autoStep) {
			case STRAIGHT:
				driveStraight(0, .6);

				if (distance > 35.0) {
					stop();
					timerStart = System.currentTimeMillis();
					autoStep = Step.STRAIGHT_PAUSE;
				}
				break;

			case STRAIGHT_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 500) {
					autoStep = Step.TURN;
				}
				break;

			case TURN:
				if (autoSelected == altRightPeg){
					turnRight();
				}

				// Check the gyro angle and stop short because overshoot
				if (distance > 35.0) {
					leftBack.set(0);
					leftFront.set(0);
					rightBack.set(0);
					rightFront.set(0);
					timerStart = System.currentTimeMillis();
					autoStep = Step.TURN_PAUSE;
				}
				break;

			case TURN_PAUSE:
				if ((System.currentTimeMillis() - timerStart) > 500) {
					autoStep = Step.HANG;
				}
				break;

			case HANG:
				driveStraight(60, .6);

				if (distance > 35.0) {
					leftBack.set(0);
					leftFront.set(0);
					rightBack.set(0);
					rightFront.set(0);
					autoStep = Step.DONE;
				}
				break;

			case DONE:
				break;
			}

			return;
		}

		// If not the altLeftPeg or altRightPeg, then keep the code the same as before.

		// TODO:  remove this loop - the autonomousPeriodic routine gets called every 20ms - FIXED

		// This is where the autonomous code goes. Setup switch cases to choose between the modes using smartdashboard

		double centerX = 0;
		double centerY = 0;
		double width = 0; 
		double area = 0;
		double height = 0;

		table.putNumber("CenterX", centerX);
		table.putNumber("CenterY", centerY);
		table.putNumber("Width", width);
		table.putNumber("Area", area);
		table.putNumber("Height", height);


		switch(autoSelected){

		case leftpeg: //drive straight
			if (autoLoopCounter < 45000){
				leftBack.set(0.8);
				rightBack.set(-0.8);
				//climber.set(-1.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 50000){
				leftBack.set(0.0);
				rightBack.set(0.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 77000){
				leftBack.set(-0.8);
				rightBack.set(0.8);
				autoLoopCounter++;
			}
			else{
				leftBack.set(0.0);
				rightBack.set(0.0);
			} break;
		case rightpeg:
			if (autoLoopCounter < 45000){
				leftBack.set(0.8);
				rightFront.set(-0.8);
				//climber.set(-1.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 50000){
				leftBack.set(0.0);
				rightFront.set(0.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 77000){
				leftBack.set(-0.8);
				rightFront.set(0.8);
				autoLoopCounter++;
			}
			else{
				leftBack.set(0.0);
				rightFront.set(0.0);

			}
			break;
		case centerpeg:
			if (autoLoopCounter < 22000){
				leftBack.set(0.4);
				rightFront.set(-0.4);
				//climber.set(-1.0);
				autoLoopCounter++;
			}
			/*else if (autoLoopCounter < 80000){
				leftBack.set(0.0);
				rightFront.set(0.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 107000){
				leftBack.set(-0.4);
				rightFront.set(0.4);
				autoLoopCounter++;
			}*/
			else{
				leftBack.set(0.0);
				rightFront.set(0.0);

			}
			break;
		case followtape:
			break;

		case DriveStraight:
			if (autoLoopCounter < 45000){
				leftBack.set(0.8);
				rightBack.set(-0.8);
				//climber.set(-1.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 50000){
				leftBack.set(0.0);
				rightBack.set(0.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 77000){
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
			if (autoLoopCounter < 45000){
				leftBack.set(0.8);
				rightBack.set(-0.8);
				//climber.set(-1.0);
				autoLoopCounter++;
			}
			else if (autoLoopCounter < 45001){
				leftBack.set(0.0);
				rightBack.set(0.0);
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


	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		// TODO:  remove this while loop - FIXED
		// The teleopPeriodic routine is called every ~20ms

		updateSmartDashboard();

		aButton = driver.getRawButton(1);
		bButton = driver.getRawButton(2);
		xButton = driver.getRawButton(3);
		yButton = driver.getRawButton(4);
		lbumperButton = driver.getRawButton(5);
		rbumperButton = driver.getRawButton(6);

		table.putNumber("CenterX", centerX);
		table.putNumber("CenterY", centerY);
		table.putNumber("Width", width);
		table.putNumber("Area", area);
		table.putNumber("Height", height);
		table.putNumber("x_value", x);

		// Loop that is called while teleop is running
		// Use this to setup drive style (Ex. Tank/Arcade/etc)
		// Use this to map button inputs to motors, etc

		chassis.arcadeDrive((-driver.getY()), (-driver.getX()));

		//double[] defaultValue = new double[0];
		//double[] centerX = table.getNumberArray("centerX",defaultValue );

		selectButton = driver.getRawButton(7);
		startButton = driver.getRawButton(8);
		
		if (selectButton == true){
			x = 0;
		}
		else if (startButton == true){
			x = 1;
		}

		if (x == 0){
			chassis.arcadeDrive((-driver.getY()), (-driver.getX()));
			// This is A good program		
			}
		else if (x == 1){
			//Backwards Driving
			chassis.arcadeDrive(driver.getRawAxis(5), (-driver.getRawAxis(4)));
			// TODO: create rawaxis variables and change X axis to proper value
		}


		if (aButton == true){
			light.set(true);
		}
		else if (bButton == true) {
			light.set(false);
		}
		if (lbumperButton == true){
			launcher.set(1.0);
		}
		else if (rbumperButton == true){
			climber.set(-1.0);
			//Timer.delay(0.5);
		}
		else {
			climber.set(0.0);
			launcher.set(0.0);
		}

		

	}


	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		updateSmartDashboard();
	}


	private void driveStraight(double heading, double speed) {

		//		leftBack.set(0.6);
		//		leftFront.set(0.6);
		//		rightBack.set(-0.6);
		//		rightFront.set(-0.6);
		// get the current heading and calculate a heading error
		double currentAngle = gyroscope.getAngle();
		double error = heading - currentAngle;

		// calculate the speed for the motors
		double leftSpeed = speed;
		double rightSpeed = speed;

		// adjust the motor speed based on the compass error
		if (error < 0) {
			// turn left
			// slow down the left motor
			leftSpeed -= error * k;
		}
		else if (error > 0){
			
		}
		else {
			// turn right
		}

		// set the motors based on the speeds
	}
	// TODO: Create stop function - FIXED
	private void stop(){
		leftBack.set(0);
		leftFront.set(0);
		rightBack.set(0);
		rightFront.set(0);
	}
	// TODO: Create TurnRight and TurnLeft Functions
	private void turnRight(){
		//TODO: Populate function with drive speed values
		// We want to turn in place to 60 degrees 
	}
	private void turnLeft(){
		// We want to turn in place -60 degrees
	}

	private double getDistance() {
		return ((double)(leftEncoder.get() + rightEncoder.get())) / (ENCODER_COUNTS_PER_INCH * 2);
	}


	private void updateSmartDashboard() {

		// TODO: add the gyro to the smartdashboard update - FIXED
		SmartDashboard.putData("Gyro", gyroscope);
		SmartDashboard.putNumber("Gyro Angle", gyroscope.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyroscope.getRate());

		SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.get());
		SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.get());
		SmartDashboard.putNumber("Encoder Distance", getDistance());
	}

}

