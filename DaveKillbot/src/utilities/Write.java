package utilities;

import lejos.hardware.lcd.GraphicsLCD;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class Write {

	/**
	 * Writes a single message in the center of the screen.
	 * 
	 * @param message,
	 *            the string to write
	 */
	public static void writeLine(GraphicsLCD screen, String message) {
		clearScreen();
		screen.drawString(message, LCD.SCREEN_HEIGHT / 2, LCD.SCREEN_WIDTH / 2, GraphicsLCD.BASELINE | GraphicsLCD.HCENTER);
	}

	/**
	 * Writes a message to the screen. Convenience method.
	 * 
	 * @param message
	 */
	public static void writeLine(GraphicsLCD screen, double message) {
		clearScreen();
		writeLine(screen, "" + message);
	}

	/**
	 * Writes a series of strings progressively lower on the screen, provides
	 * easy access to word wrap on the screen.
	 * 
	 * @param messages,
	 *            the strings to include on the screen
	 */
	public static void writeLines(String... messages) {
		clearScreen();

		for (int i = 0; i < messages.length; i++) {
			LCD.drawString(messages[i], 0, i);
		}

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
	public static void showCountdown(GraphicsLCD screen, int startTime, short timeToPause) {
		clearScreen();
		
		for (int i = Math.abs(startTime); i >= 0; i--) {
			writeLine(screen, i);
			Delay.msDelay((Math.abs(timeToPause)));
		}
	}

	/**
	 * Clears the LCD screen for the robot.
	 */
	public static void clearScreen() {
		LCD.clear();
	}

}
