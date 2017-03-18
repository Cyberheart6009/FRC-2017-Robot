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
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
	SendableChooser chooser;
	
	DigitalOutput light;
	Victor leftFront, leftBack, rightFront, rightBack, hopper, climber, launcher;
	Encoder leftEncoder, rightEncoder;
	
	//Spark launcher;
	Joystick driver;
	//Joystick operator;
	RobotDrive chassis;
	Gyro gyroscope;
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
    public void robotInit() {
		// Called when the Robot is initialized
		// Setup the chassis,motors, etc
    	//light.set(true);
    	
    	
    	chooser = new SendableChooser();
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
        
    	aButton = driver.getRawButton(1);
    	bButton = driver.getRawButton(2);
    	xButton = driver.getRawButton(3);
    	yButton = driver.getRawButton(4);
    	lbumperButton = driver.getRawButton(5);
    	rbumperButton = driver.getRawButton(6);
    	
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
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    
    	
    
	 
    public void autonomousInit() {
	
		// Initializes autonomous mode
		// setup loop variable as well as the smartdashboard

    	autoSelected = (String) chooser.getSelected();
		//autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
        autoLoopCounter = 0;
        
        leftEncoder.reset();
        rightEncoder.reset();
        step = Step.STRAIGHT;
        
    }

    /**
     * This function is called periodically during autonomous
     */
    
    

    public double getDistance() {
    	return ((double)(leftEncoder.get() + rightEncoder.get())) / (ENCODER_COUNTS_PER_INCH * 2);
    }
    
    public enum Step { STRAIGHT, STRAIGHT_PAUSE, TURN, TURN_PAUSE, HANG, DONE };
    public Step step = Step.STRAIGHT;
    public long timerStart;
    
    public void disabledPeriodic() {
    	
		updateSmartDashboard();

    }
    
    public void autonomousPeriodic() {
    	
		updateSmartDashboard();

		if (autoSelected == altLeftPeg || autoSelected == altRightPeg) {
    		
    		// Calculate the distance since the last reset of the encoders
    		double distance = getDistance();
    		
    		switch (step) {
    		case STRAIGHT:
				leftBack.set(0.6);
				leftFront.set(0.6);
				rightBack.set(-0.6);
				rightFront.set(-0.6);
				
				if (distance > 35.0) {
					rightBack.set(0);
					rightFront.set(0);
					leftBack.set(0);
					leftFront.set(0);
					timerStart = System.currentTimeMillis();
					step = Step.STRAIGHT_PAUSE;
				}
    			break;

    		case STRAIGHT_PAUSE:
    			if ((System.currentTimeMillis() - timerStart) > 500) {
    				step = Step.TURN;
    			}
    			break;
    			
    		case TURN:
				leftBack.set(0.3);
				leftFront.set(0.3);
				rightBack.set(0);
				rightFront.set(0);
				
				if (distance > 35.0) {
					leftBack.set(0);
					leftFront.set(0);
					rightBack.set(0);
					rightFront.set(0);
					timerStart = System.currentTimeMillis();
					step = Step.TURN_PAUSE;
				}
    			break;
    			
    		case TURN_PAUSE:
    			if ((System.currentTimeMillis() - timerStart) > 500) {
    				step = Step.HANG;
    			}
    			break;

    		case HANG:
				leftBack.set(0.6);
				leftFront.set(0.6);
				rightBack.set(-0.6);
				rightFront.set(-0.6);
				
				if (distance > 35.0) {
					leftBack.set(0);
					leftFront.set(0);
					rightBack.set(0);
					rightFront.set(0);
					step = Step.DONE;
				}
    			break;
    			
    		case DONE:
    			break;
    		}
    		
    		return;
    	}
    	
    	while (isAutonomous()) {
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
    			
    			/*
    			 if (autoLoopCounter < 200){
    			 	leftBack.set(-0.5);
    			 	rightBack.set(-0.5);
    			 	autoLoopCounter++;
    			 }
    			 else if (autoLoopCounter < 300){
    			 	leftBack.set(-0.5);
    			 	rightBack.set(-1.0);
    			 	autoLoopCounter++;
    			 }
    			 else if (autoLoopCounter <= 500){
    			 	leftBack.set(-0.3);
    			 	rightBack.set(-0.3);
    			 	autoLoopCounter++;
    			 }
    			 else {
    			 	leftBack.set(0.0);
    			 	rightBack.set(0.0);
    			 }
    			 */
    			break;
    		
    		
    				
    			}
    		}
            
    	}
    	
    
            
    /**
     * This function is called periodically during operator control
     */
    
    
    public void teleopPeriodic() {
    	
    	while (isOperatorControl() && isEnabled()){

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
    		
    		double[] defaultValue = new double[0];
    		double[] centerX = table.getNumberArray("centerX",defaultValue );
    		SmartDashboard.putNumber("gyro", gyroscope.getAngle());
    		// SmartDashboard.putNumber("Center X: ", centerX);
    		
    		

        	
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
    		}
    		else if (x == 1){
    	    //Backwards Driving
    	    chassis.arcadeDrive(driver.getRawAxis(5), (-driver.getRawAxis(5)));
    		}

        	
    		if (aButton == true){
        		light.set(true);
        	}
            else if (bButton == true) {
            	light.set(false);
            }
        	
            if (xButton == true) {
            	gyroscope.reset();
            	angle = gyroscope.getAngle() + 90.0;
        		chassis.arcadeDrive(0.05, angle*kP);
            } 
            else if (yButton == true) {
            	gyroscope.reset();
            	angle = gyroscope.getAngle() + 15.0;
        		chassis.arcadeDrive(0.05, angle*kP);
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
    	
    }
        
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    public void updateSmartDashboard() {
    	SmartDashboard.putNumber("Left Encoder Distance", leftEncoder.get());
    	SmartDashboard.putNumber("Right Encoder Distance", rightEncoder.get());
    	SmartDashboard.putNumber("Encoder Distance", getDistance());
    }
    
}

