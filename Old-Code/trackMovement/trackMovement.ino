/* 
Purpose: Maintain heading given
Input:  desired angle : float of the desired angle to turn , speed : float of the speed that it is going 
Output: VOID
*/
// angle : Max Left (60) - Max Right (120)
// speed : 0 - 255 (Max Speed is 70 MPH
void trackMovement() {
    static float angleHistory[ARRAY_SIZE];
    static float speedHistory[ARRAY_SIZE];
    static int index = 0;

    // Store the current angle and speed in the local arrays
    angleHistory[index] = ypr[0];
    speedHistory[index] = currBC[0];
    index = (index + 1) % ARRAY_SIZE; // Wrap around the array

    // Compare the actual movement (from simulated values) with the expected movement (from angle and speed)
    //float expectedDistance = speedHistory[index] * 1.0; // Assuming a time interval of 1 second
    //float actualDistance = sqrt(sq(Vx) + sq(Vy));
    float expectedDistance = speedHistory[index] * T_INT;
    float actualDistance = currBC[0] * T_INT; // Use the current speed (currBC[0]) as the actual distance traveled
    float distanceError = actualDistance - expectedDistance;

    float expectedAngle = angleHistory[index];
    //float actualAngle = ypr[0];
    float actualAngle = ypr[0] * 180.0 / M_PI; // Convert ypr[0] from radians to degrees
    float angleError = actualAngle - expectedAngle;

    // Course correction using vector addition of angle error and new desired angle
    float newDesiredAngle = expectedAngle + angleError;

    // Normalize the new desired angle to the range [60, 120] degrees
    newDesiredAngle = fmod(newDesiredAngle, M_PI); // Wrap to [0, PI]
    if (newDesiredAngle < M_PI / 3.0) {
        newDesiredAngle += 2.0 * M_PI / 3.0; // Shift to [60, 120] range
    }

    // Set the new desired angle and speed
    currBC[1] = newDesiredAngle;
    currBC[0] = speedHistory[index]; // Maintain the same speed for now

    // Print the updated values for verification
    Serial.print("Angle (deg): ");
    Serial.print(currBC[1] * 180.0 / M_PI);
    Serial.print(" Speed: ");
    Serial.println(currBC[0]);
}

void updateYawPitchRoll(float deltaYaw, float deltaPitch, float deltaRoll) {
    ypr[0] += deltaYaw;
    ypr[1] += deltaPitch;
    ypr[2] += deltaRoll;
}

void updateVelocity(float deltaVx, float deltaVy) {
    Vx += deltaVx;
    Vy += deltaVy;

    // Limit the speed to the maximum value
    float speed = sqrt(sq(Vx) + sq(Vy));
    if (speed > MAX_SPEED) {
        Vx = (Vx / speed) * MAX_SPEED;
        Vy = (Vy / speed) * MAX_SPEED;
    }
}
