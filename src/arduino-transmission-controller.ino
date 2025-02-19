/**
 * High-Performance Arduino Transmission Controller
 * 
 * This program controls an automotive transmission using an Arduino Uno,
 * with support for manual and automatic shift modes. Features include:
 *  - Direct port manipulation for maximum speed
 *  - Debouncing for reliable button operation
 *  - Manual/Auto mode switching
 *  - Speed-based initial gear selection
 *  - RPM-based automatic shifting
 *  - Four-gear drive implementation with P/R/N/D selector support
 *  - PWM solenoid control for precise pressure regulation
 *  - Comprehensive debugging output
 *  - Test mode support for development
 */

// ---- Pin Definitions ----
#define TACH_PIN 2             // Tachometer input (INT0)
#define SPEED_SENSOR_PIN 3     // Vehicle speed sensor (INT1)
#define UPSHIFT_PIN 4          // Upshift button input (active LOW with pullup)
#define DOWNSHIFT_PIN 5        // Downshift button input (active LOW with pullup)
#define PARK_PIN 6             // Park position input
#define DRIVE_PIN 7            // Drive position input
#define REVERSE_PIN 8          // Reverse position input
#define NEUTRAL_PIN 9          // Neutral position input
#define MANUAL_MODE_PIN A1     // Manual/Auto mode selector (HIGH=MANUAL, LOW=AUTO)

// Solenoid Output Pins
#define SOLENOID_1_PIN 10      // PWM capable pin
#define SOLENOID_2_PIN 11      // PWM capable pin
#define SOLENOID_3_PIN 12      // PWM capable pin
#define SOLENOID_4_PIN 13      // Digital output
#define SOLENOID_5_PIN A0      // Digital output (using analog pin as digital)

// ---- Configuration Constants ----
#define PWM_FREQUENCY 1000     // PWM frequency in Hz for solenoids
#define DEBOUNCE_TIME 50       // Debounce time in milliseconds
#define SHIFT_DELAY 500        // Minimum time between shifts in milliseconds
#define MAX_RPM 8000           // Maximum engine RPM
#define MAX_SPEED 200          // Maximum vehicle speed in km/h
#define GEAR_COUNT 4           // Number of drive gears (D1, D2, D3, D4)
#define DEBUG_MODE true        // Set to true to enable debug output

// Test mode settings
#define TEST_DRIVE_MODE true   // Set to true to force DRIVE mode for testing
#define TEST_SHIFT_MODE 2      // 0=Normal (read from pin), 1=Force AUTO, 2=Force MANUAL

// ---- State Variables ----
// Sensor readings
volatile uint16_t currentRPM = 0;
volatile uint16_t currentSpeed = 0;
volatile uint32_t lastTachPulseTime = 0;
volatile uint32_t lastSpeedPulseTime = 0;
volatile uint16_t tachPulseCount = 0;
volatile uint16_t speedPulseCount = 0;

// Transmission state
uint8_t currentGear = 0;       // 0=Not in Drive, 1-4=Drive gears (D1-D4)
uint8_t targetGear = 0;
uint8_t transmissionMode = 0;  // 0=Park, 1=Drive, 2=Reverse, 3=Neutral
uint8_t isManualMode = false;  // Shift mode: false=AUTO, true=MANUAL
uint32_t lastShiftTime = 0;
uint8_t solenoidStates[5] = {0, 0, 0, 0, 0};

// Shift button debounce variables
uint8_t lastUpShiftState = HIGH;     // Last stable state (HIGH = not pressed)
uint8_t lastDownShiftState = HIGH;   // Last stable state (HIGH = not pressed)
uint32_t lastUpShiftDebounceTime = 0;  // Last time up shift button state changed
uint32_t lastDownShiftDebounceTime = 0; // Last time down shift button state changed
uint8_t upShiftReading = HIGH;       // Temporary state for debouncing
uint8_t downShiftReading = HIGH;     // Temporary state for debouncing

// Debugging variables
uint32_t loopCounter = 0;      // Counts main loop executions
uint32_t lastDebugTime = 0;    // Tracks timing for debug messages

// ---- Shift Parameters ----
// RPM thresholds for automatic shifting (calibrate for your transmission)
const uint16_t upshiftRPM[GEAR_COUNT-1] = {3000, 4000, 4500};
const uint16_t downshiftRPM[GEAR_COUNT-1] = {1200, 2000, 3000};

// Speed thresholds for initial gear selection when entering Drive (in km/h)
// These define the maximum speed for each gear when initially selecting Drive
const uint16_t initialGearSpeedThresholds[GEAR_COUNT] = {
  15,   // D1: 0-15 km/h
  40,   // D2: 16-40 km/h
  70,   // D3: 41-70 km/h
  MAX_SPEED  // D4: 71+ km/h (up to MAX_SPEED)
};

// Solenoid patterns for each transmission mode 
// Format: {SOLENOID_1_PWM, SOLENOID_2_PWM, SOLENOID_3_PWM, SOLENOID_4_PIN, SOLENOID_5_PIN}
// First 3 values are PWM (0-255), last 2 are digital (0/1)
const uint8_t solenoidPatterns[4][5] = {
  // Park mode
  {0, 0, 0, 0, 0},
  
  // Reverse mode  
  {255, 0, 0, 0, 0},
  
  // Neutral mode
  {0, 0, 0, 0, 0},
  
  // Drive mode - initial pattern when Drive is selected but before shifting
  {0, 0, 0, 0, 0}
};

// Drive gear solenoid patterns (D1, D2, D3, D4) as specified
const uint8_t driveGearPatterns[GEAR_COUNT][5] = {
  // D1 gear - SOLENOID_4_PIN and SOLENOID_5_PIN are on
  {255, 0, 0, 1, 1},
  
  // D2 gear - SOLENOID_4_PIN is off and SOLENOID_5_PIN is on
  {0, 255, 0, 0, 1},
  
  // D3 gear - SOLENOID_4_PIN and SOLENOID_5_PIN are off
  {0, 0, 255, 0, 0},
  
  // D4 gear - SOLENOID_4_PIN is on and SOLENOID_5_PIN is off
  {128, 128, 0, 1, 0}
};

// Function prototypes
void updateSensorReadings();
void updateTransmissionMode();
void updateShiftMode();
void checkShiftButtons();
void determineOptimalGear();
void handleUpshift();
void handleDownshift();
bool canShift();
void executeGearChange();
void applyDriveGearPattern(uint8_t gear);
void applyBaseSolenoidPattern(uint8_t mode);
uint8_t determineInitialGear();
void disableUnusedPeripherals();
void setupFastPWM();
void configureInterrupts();
void outputDebugInfo();

// ---- Setup Function ----
void setup() {
  // Initialize serial communication if debugging is enabled
  if (DEBUG_MODE) {
    Serial.begin(115200);  // Using high baud rate for performance
    Serial.println(F("Transmission Controller Debug Mode Enabled"));
  }
  
  // Configure pin modes using direct port manipulation for speed
  // Inputs on Port D
  DDRD &= ~((1 << TACH_PIN) | (1 << SPEED_SENSOR_PIN) | (1 << UPSHIFT_PIN) | 
           (1 << DOWNSHIFT_PIN) | (1 << PARK_PIN) | (1 << DRIVE_PIN) | 
           (1 << REVERSE_PIN) | (1 << NEUTRAL_PIN));
  
  // Manual mode pin (on analog port C)
  DDRC &= ~(1 << (MANUAL_MODE_PIN - 14)); // Set A1 as input
  
  // Enable pull-up resistors for grounded inputs and sensor inputs
  PORTD |= (1 << UPSHIFT_PIN) | (1 << DOWNSHIFT_PIN) | 
           (1 << TACH_PIN) | (1 << SPEED_SENSOR_PIN);
  
  // Outputs
  DDRB |= (1 << (SOLENOID_1_PIN - 8)) | (1 << (SOLENOID_2_PIN - 8)) | 
          (1 << (SOLENOID_3_PIN - 8)) | (1 << (SOLENOID_4_PIN - 8));
  DDRC |= (1 << (SOLENOID_5_PIN - 14)); // A0 is PC0
  
  // Configure PWM for solenoid control at specified frequency
  setupFastPWM();
  
  // Set up interrupts for tachometer and speed sensor
  configureInterrupts();
  
  // Disable all Arduino library overhead that isn't needed
  if (!DEBUG_MODE) {
    disableUnusedPeripherals();
  } else {
    // In debug mode, keep UART enabled but disable other unused peripherals
    PRR = (1 << PRTWI);  // Only disable I2C
  }
  
  // Initialize shift button states - start with reading the actual pin state
  upShiftReading = (PIND & (1 << UPSHIFT_PIN)) ? HIGH : LOW;
  downShiftReading = (PIND & (1 << DOWNSHIFT_PIN)) ? HIGH : LOW;
  lastUpShiftState = upShiftReading;
  lastDownShiftState = downShiftReading;
  
  // Read initial manual/auto mode
  updateShiftMode();
  
  // Small delay to allow sensors to stabilize
  delay(100);
  
  // Initial state
  #if TEST_DRIVE_MODE
    transmissionMode = 1;  // Force DRIVE mode
    currentGear = determineInitialGear();  // Set appropriate gear based on speed
    targetGear = currentGear;
    
    if (DEBUG_MODE) {
      Serial.print(F("TEST MODE: Initialized to DRIVE with gear D"));
      Serial.println(currentGear);
    }
  #else
    // Normal mode - read from selector inputs
    updateTransmissionMode();
  #endif
  
  // Apply initial solenoid pattern
  if (transmissionMode == 1 && currentGear > 0) {
    applyDriveGearPattern(currentGear);
  } else {
    applyBaseSolenoidPattern(transmissionMode);
  }
  
  // Initialize debug timing
  lastDebugTime = millis();
  
  if (DEBUG_MODE) {
    Serial.println(F("Setup complete. System ready."));
  }
}

// ---- Main Loop ----
void loop() {
  // Increment loop counter
  loopCounter++;
  
  // Main control loop - keep this as tight as possible
  
  // 1. Read transmission mode from selector inputs
  updateTransmissionMode();
  
  // 2. Read manual/auto mode status
  updateShiftMode();
  
  // 3. Handle manual shift requests with debouncing (only in MANUAL mode)
  if (isManualMode && transmissionMode == 1) {
    checkShiftButtons();
  }
  
  // 4. Automatic shift logic (only in Drive mode AND AUTO mode)
  if (transmissionMode == 1 && !isManualMode) {
    determineOptimalGear();
  }
  
  // 5. Execute gear change if needed
  if (targetGear != currentGear && canShift()) {
    executeGearChange();
  }
  
  // 6. Periodic pulse counting and sensor timeout checking 
  updateSensorReadings();
  
  // 7. Debug output if enabled
  if (DEBUG_MODE) {
    outputDebugInfo();
  }
}

// ---- Interrupt Handlers ----
// Tachometer interrupt - count pulses for RPM calculation
ISR(INT0_vect) {
  // Read the actual pin state to verify it's a real signal (should be LOW with falling edge detection)
  uint8_t pinState = digitalRead(TACH_PIN);
  
  // Only process the interrupt if the pin is actually in the expected state
  if (pinState == LOW) {
    uint32_t currentTime = micros();
    
    // More robust debounce with minimum time between pulses
    if (currentTime - lastTachPulseTime > 500) { // 500µs minimum between pulses (max ~2000Hz)
      tachPulseCount++;
      lastTachPulseTime = currentTime;
      
      #if DEBUG_MODE && 1  // Set to 1 to enable pin state debugging (warning: can slow performance)
      static uint32_t lastPrintTime = 0;
      if (currentTime - lastPrintTime > 1000000) { // Only print once per second
        Serial.print(F("TACH pulse detected. Pin state: "));
        Serial.println(pinState);
        lastPrintTime = currentTime;
      }
      #endif
    }
  }
}

// Speed sensor interrupt - count pulses for speed calculation
ISR(INT1_vect) {
  // Read the actual pin state to verify it's a real signal (should be LOW with falling edge detection)
  uint8_t pinState = digitalRead(SPEED_SENSOR_PIN);
  
  // Only process the interrupt if the pin is actually in the expected state
  if (pinState == LOW) {
    uint32_t currentTime = micros();
    
    // More robust debounce with minimum time between pulses
    if (currentTime - lastSpeedPulseTime > 500) { // 500µs minimum between pulses (max ~2000Hz)
      speedPulseCount++;
      lastSpeedPulseTime = currentTime;
      
      #if DEBUG_MODE && 0  // Set to 1 to enable pin state debugging (warning: can slow performance)
      static uint32_t lastPrintTime = 0;
      if (currentTime - lastPrintTime > 1000000) { // Only print once per second
        Serial.print(F("SPEED pulse detected. Pin state: "));
        Serial.println(pinState);
        lastPrintTime = currentTime;
      }
      #endif
    }
  }
}

// ---- Core Functions ----
// Calculate RPM and speed from pulse counts
void updateSensorReadings() {
  static uint32_t lastCalcTime = 0;
  uint32_t currentTime = millis();
  
  // Calculate RPM and speed every 100ms
  if (currentTime - lastCalcTime >= 100) {
    // Disable interrupts for consistent readings
    noInterrupts();
    
    // Get local copies of volatile variables 
    uint16_t localTachCount = tachPulseCount;
    uint16_t localSpeedCount = speedPulseCount;
    uint32_t localTachTime = lastTachPulseTime;
    uint32_t localSpeedTime = lastSpeedPulseTime;
    
    // Reset counters
    tachPulseCount = 0;
    speedPulseCount = 0;
    
    // Re-enable interrupts as soon as possible
    interrupts();
    
    // Check for sensor timeout (indicating 0 RPM/speed)
    // Using 1 second as timeout period (more reliable than 500ms)
    uint32_t timeSinceLastTach = micros() - localTachTime;
    uint32_t timeSinceLastSpeed = micros() - localSpeedTime;
    
    if (timeSinceLastTach > 1000000) { // 1 second timeout
      currentRPM = 0;
    } else {
      // Calculate RPM based on pulse count
      // Assuming x pulses per revolution (adjust multiplier accordingly)
      currentRPM = (localTachCount * 600); // 60 seconds * 10 for 100ms window
    }
    
    if (timeSinceLastSpeed > 1000000) { // 1 second timeout
      currentSpeed = 0;
    } else {
      // Calculate speed based on pulse count
      // Assuming y pulses per km (adjust multiplier accordingly)
      currentSpeed = (localSpeedCount * 36); // 3600 seconds * 0.1 for 100ms window
    }
    
    // Sanity check - prevent unrealistically high readings
    if (currentRPM > MAX_RPM) currentRPM = MAX_RPM;
    if (currentSpeed > MAX_SPEED) currentSpeed = MAX_SPEED;
    
    // Update calculation timestamp
    lastCalcTime = currentTime;
    
    #if DEBUG_MODE && 0 // Set to 1 to enable verbose sensor debugging
    Serial.print(F("Sensor update: RPM="));
    Serial.print(currentRPM);
    Serial.print(F(" ("));
    Serial.print(localTachCount);
    Serial.print(F(" pulses), Speed="));
    Serial.print(currentSpeed);
    Serial.print(F(" km/h ("));
    Serial.print(localSpeedCount);
    Serial.println(F(" pulses)"));
    #endif
  }
}

// Determine the appropriate starting gear based on vehicle speed
uint8_t determineInitialGear() {
  // If vehicle is stationary or moving very slowly, always use first gear
  if (currentSpeed <= 5) {
    return 1;
  }
  
  // Search through speed thresholds to find appropriate gear
  for (uint8_t gear = 1; gear <= GEAR_COUNT; gear++) {
    if (currentSpeed <= initialGearSpeedThresholds[gear-1]) {
      return gear;
    }
  }
  
  // Failsafe - if we somehow exceed all thresholds, use top gear
  return GEAR_COUNT;
}

// Update transmission mode based on selector position
void updateTransmissionMode() {
  uint8_t oldMode = transmissionMode;
  
  #if TEST_DRIVE_MODE
    // In test mode, always maintain Drive mode regardless of selector position
    transmissionMode = 1; // Force Drive mode
    
    // Only initialize gear if we weren't already in Drive mode
    if (oldMode != 1) {
      // Select optimal starting gear based on current speed
      currentGear = determineInitialGear();
      targetGear = currentGear;
      
      if (DEBUG_MODE) {
        Serial.print(F("TEST MODE: Switching to DRIVE mode with gear D"));
        Serial.println(currentGear);
      }
    }
  #else
    // Normal operation - read selector inputs
    // Read all selector inputs directly from port for speed
    uint8_t portDInputs = PIND;
    
    if (portDInputs & (1 << PARK_PIN)) {
      transmissionMode = 0; // Park
      currentGear = 0;
      targetGear = 0;
    }
    else if (portDInputs & (1 << DRIVE_PIN)) {
      transmissionMode = 1; // Drive
      // Initialize gear based on vehicle speed if entering Drive mode
      if (oldMode != 1) {
        currentGear = determineInitialGear();
        targetGear = currentGear;
        
        if (DEBUG_MODE) {
          Serial.print(F("Entering DRIVE mode at speed "));
          Serial.print(currentSpeed);
          Serial.print(F(" km/h - selected gear D"));
          Serial.println(currentGear);
        }
      }
    }
    else if (portDInputs & (1 << REVERSE_PIN)) {
      transmissionMode = 2; // Reverse
      currentGear = 0;
      targetGear = 0;
    }
    else if (portDInputs & (1 << NEUTRAL_PIN)) {
      transmissionMode = 3; // Neutral
      currentGear = 0;
      targetGear = 0;
    }
  #endif
  
  // If mode changed, apply appropriate solenoid pattern
  if (oldMode != transmissionMode) {
    if (transmissionMode == 1 && currentGear > 0) {
      // If in Drive mode with a specific gear, apply that gear's pattern
      applyDriveGearPattern(currentGear);
    } else {
      // Otherwise apply the base pattern for the current mode
      applyBaseSolenoidPattern(transmissionMode);
    }
  }
}

// Read the manual/auto mode selector pin
void updateShiftMode() {
  bool oldMode = isManualMode;
  
  #if TEST_SHIFT_MODE == 0
    // Normal operation - read from pin
    // Read the manual mode pin using direct port manipulation for speed
    // HIGH = MANUAL mode, LOW = AUTO mode
    uint8_t pinState = (PINC & (1 << (MANUAL_MODE_PIN - 14))) ? HIGH : LOW;
    
    // Update the mode state
    isManualMode = (pinState == HIGH);
  #elif TEST_SHIFT_MODE == 1
    // Force AUTO mode
    isManualMode = false;
    
    if (oldMode != isManualMode && DEBUG_MODE) {
      Serial.println(F("TEST MODE: Forcing AUTO shift mode"));
    }
  #elif TEST_SHIFT_MODE == 2
    // Force MANUAL mode
    isManualMode = true;
    
    if (oldMode != isManualMode && DEBUG_MODE) {
      Serial.println(F("TEST MODE: Forcing MANUAL shift mode"));
    }
  #endif
  
  // If mode changed and we're in debug mode, log it
  if (DEBUG_MODE && oldMode != isManualMode) {
    Serial.print(F("Shift mode changed to: "));
    Serial.println(isManualMode ? F("MANUAL") : F("AUTO"));
  }
}

// Check shift buttons with debounce logic
void checkShiftButtons() {
  uint32_t currentTime = millis();
  
  // Read the current state of both shift buttons using direct port reads for speed
  uint8_t upShiftPinState = (PIND & (1 << UPSHIFT_PIN)) ? HIGH : LOW;
  uint8_t downShiftPinState = (PIND & (1 << DOWNSHIFT_PIN)) ? HIGH : LOW;
  
  // Check for upshift button state change
  if (upShiftPinState != upShiftReading) {
    // Reset the debounce timer
    lastUpShiftDebounceTime = currentTime;
    // Update the temporary reading
    upShiftReading = upShiftPinState;
  }
  
  // Check for downshift button state change
  if (downShiftPinState != downShiftReading) {
    // Reset the debounce timer
    lastDownShiftDebounceTime = currentTime;
    // Update the temporary reading
    downShiftReading = downShiftPinState;
  }
  
  // Check if upshift button has been stable long enough to consider it a valid change
  if ((currentTime - lastUpShiftDebounceTime) > DEBOUNCE_TIME) {
    // Only trigger upshift on the transition from HIGH to LOW (button press)
    if (upShiftReading == LOW && lastUpShiftState == HIGH && canShift()) {
      handleUpshift();
    }
    // Update the last stable state
    lastUpShiftState = upShiftReading;
  }
  
  // Check if downshift button has been stable long enough to consider it a valid change
  if ((currentTime - lastDownShiftDebounceTime) > DEBOUNCE_TIME) {
    // Only trigger downshift on the transition from HIGH to LOW (button press)
    if (downShiftReading == LOW && lastDownShiftState == HIGH && canShift()) {
      handleDownshift();
    }
    // Update the last stable state
    lastDownShiftState = downShiftReading;
  }
}

// Automatic shift logic based on RPM
void determineOptimalGear() {
  // Only perform automatic shift logic when in Drive mode
  if (transmissionMode != 1) return;
  
  // Simple automatic shift logic based on RPM
  if (currentGear < GEAR_COUNT && currentRPM > upshiftRPM[currentGear-1]) {
    targetGear = currentGear + 1;
  }
  else if (currentGear > 1 && currentRPM < downshiftRPM[currentGear-2]) {
    targetGear = currentGear - 1;
  }
}

// Handle manual upshift request
void handleUpshift() {
  // Manual upshift can only happen in Drive mode
  if (transmissionMode == 1 && currentGear < GEAR_COUNT) {
    targetGear = currentGear + 1;
    if (DEBUG_MODE) {
      Serial.print(F("Manual upshift requested to D"));
      Serial.println(targetGear);
    }
  }
}

// Handle manual downshift request
void handleDownshift() {
  // Manual downshift can only happen in Drive mode
  if (transmissionMode == 1 && currentGear > 1) {
    targetGear = currentGear - 1;
    if (DEBUG_MODE) {
      Serial.print(F("Manual downshift requested to D"));
      Serial.println(targetGear);
    }
  }
}

// Check if minimum shift time has elapsed
bool canShift() {
  // Prevent shift bounce and ensure minimum time between shifts
  uint32_t currentTime = millis();
  if (currentTime - lastShiftTime < SHIFT_DELAY) {
    return false;
  }
  return true;
}

// Execute a gear change
void executeGearChange() {
  // Execute the gear change by setting solenoids
  if (transmissionMode == 1 && targetGear >= 1 && targetGear <= GEAR_COUNT) {
    // We're in Drive mode and changing between drive gears
    applyDriveGearPattern(targetGear);
    if (DEBUG_MODE) {
      Serial.print(F("Gear change executed: D"));
      Serial.println(targetGear);
    }
  } else {
    // We're changing to a base transmission mode
    applyBaseSolenoidPattern(transmissionMode);
  }
  
  currentGear = targetGear;
  lastShiftTime = millis();
}

// Apply solenoid pattern for a specific drive gear (1-4)
void applyDriveGearPattern(uint8_t gear) {
  // Validate gear number (1-4)
  if (gear < 1 || gear > GEAR_COUNT) return;
  
  // Adjust index (gear 1 is at index 0)
  uint8_t index = gear - 1;
  
  // PWM solenoids (1-3)
  analogWrite(SOLENOID_1_PIN, driveGearPatterns[index][0]);
  analogWrite(SOLENOID_2_PIN, driveGearPatterns[index][1]);
  analogWrite(SOLENOID_3_PIN, driveGearPatterns[index][2]);
  
  // Digital solenoids (4-5)
  if (driveGearPatterns[index][3]) {
    PORTB |= (1 << (SOLENOID_4_PIN - 8));
  } else {
    PORTB &= ~(1 << (SOLENOID_4_PIN - 8));
  }
  
  if (driveGearPatterns[index][4]) {
    PORTC |= (1 << (SOLENOID_5_PIN - 14));
  } else {
    PORTC &= ~(1 << (SOLENOID_5_PIN - 14));
  }
  
  // Update solenoid states for debugging
  for (int i = 0; i < 5; i++) {
    solenoidStates[i] = driveGearPatterns[index][i];
  }
}

// Apply pattern for base transmission modes (Park, Reverse, Neutral)
void applyBaseSolenoidPattern(uint8_t mode) {
  // Validate mode (0-3)
  if (mode > 3) return;
  
  // PWM solenoids (1-3)
  analogWrite(SOLENOID_1_PIN, solenoidPatterns[mode][0]);
  analogWrite(SOLENOID_2_PIN, solenoidPatterns[mode][1]);
  analogWrite(SOLENOID_3_PIN, solenoidPatterns[mode][2]);
  
  // Digital solenoids (4-5)
  if (solenoidPatterns[mode][3]) {
    PORTB |= (1 << (SOLENOID_4_PIN - 8));
  } else {
    PORTB &= ~(1 << (SOLENOID_4_PIN - 8));
  }
  
  if (solenoidPatterns[mode][4]) {
    PORTC |= (1 << (SOLENOID_5_PIN - 14));
  } else {
    PORTC &= ~(1 << (SOLENOID_5_PIN - 14));
  }
  
  // Update solenoid states for debugging
  for (int i = 0; i < 5; i++) {
    solenoidStates[i] = solenoidPatterns[mode][i];
  }
}

// ---- Utility Functions ----
// Disable unused Arduino peripherals to maximize performance
void disableUnusedPeripherals() {
  // Disable ADC if not needed for digital reads
  // We need to keep ADC enabled because we're using A1 as a digital input
  ADCSRA &= ~(1 << ADEN); // Disable ADC conversion but keep the peripheral
  
  // Disable the brown-out detector
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR &= ~_BV(BODSE);
  
  // Disable digital input buffers on unused analog pins to save power
  // We're using A0 and A1, so keep those enabled
  DIDR0 = 0x3C; // Disable digital input buffers on A2-A5, keep A0-A1
  
  // Power reduction register - disable unused peripherals
  PRR = (1 << PRTWI);    // Disable I2C
  // We need Timer0 for millis() and USART for debug output
}

// Configure PWM for precise solenoid control
void setupFastPWM() {
  // Configure Timer1 for higher frequency PWM
  // We'll use 10-bit fast PWM mode for pins 9 and 10
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // No prescaler
  ICR1 = F_CPU / PWM_FREQUENCY; // Set top value for desired frequency
  
  // Configure Timer2 for pin 11
  TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Fast PWM mode
  TCCR2B = (1 << CS20); // No prescaler
}

// Configure external interrupts for tachometer and speed sensor
void configureInterrupts() {
  // Set pins as inputs with pull-up resistors to prevent floating inputs
  pinMode(TACH_PIN, INPUT_PULLUP);
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  
  // Configure external interrupts INT0 and INT1
  // Using FALLING edge instead of RISING (more stable with pull-ups)
  EICRA = (1 << ISC01) | (0 << ISC00) | // INT0 triggers on falling edge
          (1 << ISC11) | (0 << ISC10);   // INT1 triggers on falling edge
  
  // Enable both interrupts
  EIMSK = (1 << INT0) | (1 << INT1);
  
  // Reset the counters and timestamps to ensure clean initial state
  tachPulseCount = 0;
  speedPulseCount = 0;
  lastTachPulseTime = micros();
  lastSpeedPulseTime = micros();
  
  if (DEBUG_MODE) {
    Serial.println(F("Interrupts configured - using FALLING edge detection with pull-ups"));
  }
}

// ---- Debug Function ----
// Output debugging information to serial port
void outputDebugInfo() {
  uint32_t currentTime = millis();
  
  // Output debug information once per second
  if (currentTime - lastDebugTime >= 1000) {
    // Calculate loops per second
    uint32_t loopsPerSecond = loopCounter;
    
    // Output to serial
    Serial.print(F("Loops/sec: "));
    Serial.print(loopsPerSecond);
    
    // Output test mode indicators if active
    #if TEST_DRIVE_MODE
    Serial.print(F(" | TEST_DRIVE"));
    #endif
    
    #if TEST_SHIFT_MODE == 1
    Serial.print(F(" | TEST_AUTO"));
    #elif TEST_SHIFT_MODE == 2
    Serial.print(F(" | TEST_MANUAL"));
    #endif
    
    Serial.print(F(" | RPM: "));
    Serial.print(currentRPM);
    Serial.print(F(" | Speed: "));
    Serial.print(currentSpeed);
    Serial.print(F(" | Gear: "));
    
    // Output current gear as text
    if (transmissionMode == 1 && currentGear >= 1 && currentGear <= GEAR_COUNT) {
      Serial.print(F("D"));
      Serial.print(currentGear);
    } else {
      Serial.print(F("N/A"));
    }
    
    Serial.print(F(" | Mode: "));
    
    // Output transmission mode as text
    switch (transmissionMode) {
      case 0: Serial.print(F("PARK")); break;
      case 1: Serial.print(F("DRIVE")); break;
      case 2: Serial.print(F("REVERSE")); break;
      case 3: Serial.print(F("NEUTRAL")); break;
      default: Serial.print(F("UNKNOWN")); break;
    }
    
    // Output shift mode
    Serial.print(F(" | Shift: "));
    Serial.print(isManualMode ? F("MANUAL") : F("AUTO"));
    
    // Output solenoid states
    Serial.print(F(" | Solenoids: "));
    for (int i = 0; i < 5; i++) {
      if (i < 3) {
        // Show PWM values for first three solenoids
        Serial.print(solenoidStates[i]);
      } else {
        // Show binary state for digital solenoids
        Serial.print(solenoidStates[i] ? "ON" : "OFF");
      }
      if (i < 4) Serial.print(F(","));
    }
    
    // Output button states
    Serial.print(F(" | Buttons [Up:"));
    Serial.print(lastUpShiftState == LOW ? "PRESSED" : "RELEASED");
    Serial.print(F(",Down:"));
    Serial.print(lastDownShiftState == LOW ? "PRESSED" : "RELEASED");
    Serial.print(F("]"));
    
    Serial.println();
    
    // Reset loop counter and update timer
    loopCounter = 0;
    lastDebugTime = currentTime;
  }
}
