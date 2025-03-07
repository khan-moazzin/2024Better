package com.team5817.lib.Lights;

/**
 * Represents a color with red, green, and blue components.
 */
public class Color {

	/**
	 * Predefined color constants.
	 */
	public static Color RED = new Color(255, 0, 0);
	public static Color RED_DIMMED = new Color(120, 0, 0);
	public static Color PINK = new Color(255, 18, 143);
	public static Color GREEN = new Color(0, 255, 8);
	public static Color GREEN_DIMMED = new Color(0, 50, 0);
	public static Color PURPLE = new Color(196, 18, 255);
	public static Color ORANGE = new Color(255, 53, 13);
	public static Color YELLOW = new Color(255, 150, 5);
	public static Color CYAN = new Color(52, 155, 235);
	public static Color BLUE = new Color(0, 0, 255);
	public static Color BLUE_DIMMED = new Color(0, 0, 120);

	public int r;
	public int g;
	public int b;

	/**
	 * Constructs a new Color with the specified red, green, and blue values.
	 *
	 * @param red the red component (0-255)
	 * @param green the green component (0-255)
	 * @param blue the blue component (0-255)
	 */
	public Color(int red, int green, int blue) {
		r = red;
		g = green;
		b = blue;
	}

	/**
	 * Returns a Color instance representing the color off (black).
	 *
	 * @return a Color instance with all components set to 0
	 */
	public static Color off() {
		return new Color(0, 0, 0);
	}

	/**
	 * Returns a string representation of the color in the format (r,g,b).
	 *
	 * @return a string representation of the color
	 */
	@Override
	public String toString() {
		return "(" + r + "," + g + "," + b + ")";
	}
}
