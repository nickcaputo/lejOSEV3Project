package main_threads;


import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ColorSensorTest {

	/*
public static final int	BLACK	7
public static final int	BLUE	2
public static final int	CYAN	12
public static final int	DARK_GRAY	11
public static final int	GRAY	9
public static final int	GREEN	1
public static final int	LIGHT_GRAY	10
public static final int	MAGENTA	4
public static final int	NONE	-1
public static final int	ORANGE	5
public static final int	PINK	8
public static final int	RED	0
public static final int	WHITE	6
public static final int	YELLOW	3
	 
	 */
	public static String getColorName(int colorId) {
		switch (colorId) {
			case -1: return "NONE";
			case 0: return "RED";
			case 1: return "GREEN";
			case 2: return "BLUE";
			case 3: return "YELLOW";
			case 4: return "MAGENTA";
			case 5: return "ORANGE";
			case 6: return "WHITE";
			case 7: return "BLACK";
			case 8: return "PINK";
			case 9: return "GRAY";
			case 10: return "LIGHT_GRAY";
			case 11: return "DARK_GRAY";
		}
		return "OTHER?";
	}
	public static void main(String[] args) {
		EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S4);
		
		while(!Button.ENTER.isDown()) {
			LCD.clear();
			int colorId = sensor.getColorID();
			String colorName = getColorName(colorId);
			LCD.drawString("Color Id: " + colorName, 0 , 0);
			Delay.msDelay(1000);
		}
		
		sensor.close();
	}

}
