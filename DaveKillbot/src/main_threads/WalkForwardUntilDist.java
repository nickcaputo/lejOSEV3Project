package main_threads;

import java.util.HashMap;
import java.util.Map;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.lcd.Font;
import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class WalkForwardUntilDist {

	private static final double CELL_DISTANCE = 50;
	private static final double CHECK_DISTANCE = .5;

	private static int sWidth;
	private static int sHeight;
	private static GraphicsLCD screen;

	private static DifferentialPilot robot;
	private static final double WHEEL_DIAMETER = 5.4;
	private static final double TRACK_WIDTH = 14.5;
	private static final double TRAVEL_SPEED = 30;
	private static final double ROTATE_SPEED = 60;
	private static final RegulatedMotor LEFT_MOTOR = Motor.C;
	private static final RegulatedMotor RIGHT_MOTOR = Motor.B;
	private static final RegulatedMotor TOP_MOTOR = Motor.A;

	private static EV3ColorSensor colorSense;

	private static EV3UltrasonicSensor distSense;
	private static SampleProvider sampleProvider;
	private static float[] samples;

	public static void main(String[] args) {

		WalkForwardUntilDist run = new WalkForwardUntilDist();

		run.main.start();

	}

	/**
	 * Starts the program when the user presses a button.
	 * 
	 * @param testing,
	 *            whether to only perform one action before stopping or continue
	 *            until the maze is solved.
	 */
	private void startProgram(boolean debug) {
		write("Press any button to start.");

		Button.waitForAnyPress();

		stopper.start();

		try {
			showCountdown(5, (short) 1000);
		} catch (InterruptedException err) {
			write(err.getMessage());
		}

		if (!debug) {
			solve();
		} else {
			debug_checkDistances();
		}
	}

	/**
	 * Attempts to solve the maze with a left-forward-right-back rule.
	 * 
	 * @param testing,
	 *            whether we are only doing one test
	 */
	private void solve() {
		float[] distances = new float[3];

		while (!atGoal(6)) {
			// checks left
			neckLeft90();
			distances[0] = getDistanceMeasurement();

			// checks forward
			neckRight90();
			distances[1] = getDistanceMeasurement();

			// checks right
			neckRight90();
			distances[2] = getDistanceMeasurement();

			// sets head back to straight
			neckLeft90();

			// prints results
			write("Left: " + distances[0] + "\nForward: " + distances[1] + "\nRight: " + distances[2] + '\n');

			/*
			 * Goes left if possible, otherwise forward, otherwise right,
			 * otherwise backwards
			 */
			if (distances[0] >= CHECK_DISTANCE) {
				left90();
				forwardOneCell(null);
			} else if (distances[1] >= CHECK_DISTANCE) {
				forwardOneCell(null);
			} else if (distances[2] >= CHECK_DISTANCE) {
				right90();
				forwardOneCell(null);
			} else { // it is trapped in a box
				turn180();
				forwardOneCell(null);
			}

		}

		System.out.println("Found the goal!");
	}

	/**
	 * Rotates the distance sensor 90 degrees left
	 */
	private void neckLeft90() {
		TOP_MOTOR.rotate(-90);
	}

	/**
	 * Rotates the distance sensor 90 degrees right
	 */
	private void neckRight90() {
		TOP_MOTOR.rotate(90);
	}

	/**
	 * Turns the physical robot 90 degrees left
	 */
	private void left90() {
		robot.rotate(-90);
	}

	/**
	 * Turns the physical robot 90 degrees right
	 */
	private void right90() {
		robot.rotate(90);
	}

	/**
	 * Turns the physical robot arounds
	 */
	private void turn180() {
		robot.rotate(180);
	}

	/**
	 * Moves the robot forward the specified distance, or the standard cell
	 * distance if null is a parameter
	 * 
	 * @param distance,
	 *            how far to move forward
	 */
	private void forwardOneCell(Double distance) {
		if (distance != null) {
			robot.travel(distance);
		} else {
			robot.travel(CELL_DISTANCE);
		}
	}

	/**
	 * Analyzes the distance to an object in front of the distance sensor, and
	 * returns the result in a float
	 * 
	 * @return, float value for distance
	 */
	private float getDistanceMeasurement() {
		float[] distances = new float[sampleProvider.sampleSize()];

		distSense.fetchSample(distances, 0);
		return distances[0];
	}

	/**
	 * Uses the color sensor to check if we are on top of a goal, and returns if
	 * we are
	 * 
	 * @param colorId,
	 *            the id of the color to check for (default 6)
	 * @return if we are on top of that color
	 */
	private boolean atGoal(int colorId) {
		return colorSense.getColorID() == colorId;
	}

	/*
	 * 
	 * 
	 * 
	 * 
	 * 
	 * ***************** DEBUG METHODS **************
	 * 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	/**
	 * Turns the distance sensor in the four cardinal directions, scans
	 * distance, and prints its results to the screen
	 */
	private static void debug_checkDistances() {
		Map<Character, Float> distances = new HashMap<Character, Float>();

		while (true) { // repeat infinitely

			// set distance sensor forward
			TOP_MOTOR.rotateTo(0);
			sampleProvider.fetchSample(samples, 0);
			distances.put('N', samples[0]);
			write("Forward distance: " + distances.get('N'));
			try {
				Thread.sleep(1000);
			} catch (InterruptedException err) {
				err.printStackTrace();
			}

			// set distance sensor left
			TOP_MOTOR.rotate(90);
			sampleProvider.fetchSample(samples, 0);
			distances.put('E', samples[0]);
			write("East distance: " + distances.get('E'));
			try {
				Thread.sleep(1000);
			} catch (InterruptedException err) {
				err.printStackTrace();
			}

			// set distance sensor right
			TOP_MOTOR.rotate(-180);
			sampleProvider.fetchSample(samples, 0);
			distances.put('W', samples[0]);
			write("West distance: " + distances.get('W'));
			try {
				Thread.sleep(1000);
			} catch (InterruptedException err) {
				err.printStackTrace();
			}

			// set distance sensor back
			TOP_MOTOR.rotate(-90);
			sampleProvider.fetchSample(samples, 0);
			distances.put('S', samples[0]);
			write("South distance: " + distances.get('S'));
			try {
				Thread.sleep(1000);
			} catch (InterruptedException err) {
				err.printStackTrace();
			}

		}
	}

	/*
	 * 
	 * 
	 * 
	 * 
	 * 
	 * ************** INITIALIZATION METHODS *****************
	 * 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	/**
	 * Sets up the LCD display for use in this class
	 */
	private static void setUpScreen() {
		screen = BrickFinder.getDefault().getGraphicsLCD();
		sWidth = screen.getWidth();
		sHeight = screen.getHeight();
		screen.setFont(Font.getSmallFont());

		write("Initialized Screen");
	}

	/**
	 * Sets up the motors for use
	 */
	private static void setUpMotors() {
		robot = new DifferentialPilot(WHEEL_DIAMETER, TRACK_WIDTH, LEFT_MOTOR, RIGHT_MOTOR, false);
		robot.setTravelSpeed(TRAVEL_SPEED);
		robot.setRotateSpeed(ROTATE_SPEED);

		write("DifferentialPilot and motors initialized.");
	}

	// /**
	// * Sets up the bumper for use
	// */
	// private static void setUpBumper() {
	// bumper = new EV3TouchSensor(SensorPort.S2);
	//
	// write("Bumper initialized.");
	// }

	/**
	 * Sets up the Ultrasonic sensor for use
	 */
	private static void setUpDistanceSensor() {
		distSense = new EV3UltrasonicSensor(SensorPort.S1);
		distSense.enable();

		sampleProvider = distSense.getDistanceMode();

		samples = new float[sampleProvider.sampleSize()];

		write("Distance sensor initialized.");
	}

	private static void setUpColorSensor() {
		colorSense = new EV3ColorSensor(SensorPort.S4);

		write("Color sensor initialized.");
	}

	/*
	 * 
	 * 
	 * 
	 * 
	 * 
	 * **************** UTILITY METHODS *********************
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	/**
	 * Writes a message to the LCD screen in the center of the display
	 * 
	 * @param message,
	 *            the string to write to the screen
	 */
	private static void write(String message) {
		screen.clear();
		screen.drawString(message, sWidth / 2, sHeight / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	}

	/**
	 * Convenience method to accept numbers and print them out in the write()
	 * method.
	 * 
	 * @param message,
	 *            a double value to print to the screen. Will also accept int,
	 *            long, short, byte, or float
	 */
	private static void write(double message) {
		write("" + message);
	}

	/**
	 * Shows a countdown on the screen.
	 * 
	 * @param startTime,
	 *            how many units of time to wait, uses absolute value of this
	 *            time
	 * @param timeToPause,
	 *            the length in ms of the unit of time (e.g. 1000 ms for one
	 *            second), uses absolute value of this value entered, and a
	 *            short data type ensures it cannot wait longer than ~32 seconds
	 * @throws InterruptedException,
	 *             if the notify() method is called during waiting
	 */
	private static void showCountdown(int startTime, short timeToPause) throws InterruptedException {
		for (int i = Math.abs(startTime); i >= 0; i--) {
			write(i);
			Thread.sleep(Math.abs(timeToPause));
		}
	}

	/*
	 * 
	 * 
	 * 
	 * ********************* THREADS ************************
	 * 
	 * 
	 * 
	 * 
	 * 
	 * 
	 */

	/**
	 * The main thread which starts all accessories, started from the main()
	 * method
	 */
	Thread main = new Thread() {
		@Override
		public void run() {
			setUpScreen();

			setUpMotors();

			// setUpBumper();

			setUpDistanceSensor();

			setUpColorSensor();

			startProgram(false);
		}
	};

	/**
	 * The thread which allows us to stop at any point by pressing the ESCAPE
	 * button. Started from the final method in the main thread.
	 */
	Thread stopper = new Thread() {
		@Override
		public void run() {
			while (true) {

				Button.waitForAnyPress();

				if (Button.ESCAPE.isDown()) {
					System.exit(0);
				}
			}
		}
	};

}