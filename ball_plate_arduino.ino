/*
 * Ball and Plate System - Servo Control
 * =============================================
 * 
 * This Arduino sketch controls two servo motors for a ball and plate balancing system.
 * It receives servo angle commands from the Python vision system via serial communication
 * and positions the servos accordingly to balance the ball.
 * 
 * Hardware Requirements:
 * - Arduino board (Uno, Nano, etc.)
 * - 2x Servo motors (for X and Y axis control)
 * - Servo connections: Pin 9 (X-axis), Pin 4 (Y-axis)
 * 
 * Serial Communication:
 * - Baud rate: 9600
 * - Format: "angleX,angleY\n" 
 * 
 * Author: UPM Hesham
 */

#include <Servo.h>

// ============================================================================
// SERVO OBJECTS AND VARIABLES
// ============================================================================

// Create servo objects for X and Y axis control
Servo servoX;  // Servo for X-axis (horizontal) movement
Servo servoY;  // Servo for Y-axis (vertical) movement

// Current servo angles (default center position)
float servoXAngle = 75;  // X-axis servo angle (0-180 degrees)
float servoYAngle = 75;  // Y-axis servo angle (0-180 degrees)

// Serial communication variables
int serialCount = 0;  // Counter for serial data processing

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Parse comma-separated string to extract specific value
 * 
 * This function splits a string by a separator character and returns
 * the value at the specified index. Used to parse servo angle commands.
 * 

 */
String AngleSp(String inputData, char separatorChar, int valueIndex) {
    int foundCount = 0;           // Count of separators found
    int indexArray[] = { 0, -1 }; // Array to store start and end indices
    int dataLength = inputData.length() - 1;  // Length of input string
    
    // Iterate through the string to find separators
    for (int i = 0; i <= dataLength && foundCount <= valueIndex; i++) {
        // Check if current character is separator or end of string
        if (inputData.charAt(i) == separatorChar || i == dataLength) {
            foundCount++;
            indexArray[0] = indexArray[1] + 1;  // Start index of current value
            indexArray[1] = (i == dataLength) ? i+1 : i;  // End index of current value
        }
    }
    
    // Return the extracted value if found, otherwise empty string
    return foundCount > valueIndex ? inputData.substring(indexArray[0], indexArray[1]) : "";
}

// ============================================================================
// ARDUINO SETUP FUNCTION
// ============================================================================


void setup() {
    // Initialize serial communication at 9600 baud rate
    Serial.begin(9600);
    
    // Attach servos to their respective pins
    servoX.attach(9);  // X-axis servo on pin 9
    servoY.attach(4);  // Y-axis servo on pin 4
    
    // Set servos to center position (75 degrees)
    servoX.write(servoXAngle);
    servoY.write(servoYAngle);
    
    // Wait for servos to reach position
    delay(1000);
}

// ============================================================================
// ARDUINO MAIN LOOP FUNCTION
// ============================================================================


void loop() {
    // Check if serial data is available
    if(Serial.available() > 0) {
        // Read incoming serial data until newline character
        String serialInput = Serial.readStringUntil('\n');
        
        // Parse servo angles from serial data
        // Format: "angleX,angleY" (e.g., "75,80")
        servoXAngle = AngleSp(serialInput, ',', 0).toFloat();  // Extract X angle
        servoYAngle = AngleSp(serialInput, ',', 1).toFloat();  // Extract Y angle
        
        // Move servos to calculated positions
        servoX.write(servoXAngle);  // Set X-axis servo
        servoY.write(servoYAngle);  // Set Y-axis servo
        
        // Small delay to allow servos to move
        delay(3);
    }
    
    // Note: The loop continues immediately to check for new commands
    // This provides responsive servo control for real-time ball balancing
} 