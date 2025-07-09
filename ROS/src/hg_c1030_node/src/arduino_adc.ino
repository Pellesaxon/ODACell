
#include <math.h> // Needed for sqrt()

#define RELEASE // Comment this out for verbose debugging output
#define PROGRAM_NAME "HG-C Sensor Interface (Current mode)"
#define PROGRAM_VERSION "1.1.0"

// --- Hardware & ADC Configuration ---
#define SENSOR_PIN A0
#define SHUNT_RESISTOR_OHMS 51.7f
#define ADC_REFERENCE_VOLTAGE 1.1f
#define ADC_RESOLUTION 1023.0f

// --- Sensor Physical Characteristics (for HG-C1030 model) ---
#define MIN_DISTANCE_M 0.025f      // Distance at 4mA
#define MAX_DISTANCE_M 0.035f      // Distance at 20mA
#define MIN_CURRENT_MA 4.0f
#define MAX_CURRENT_MA 20.0f

// --- Data Smoothing & Streaming Configuration ---
#define SENSOR_SPEED_MS 10
#define AVG_WINDOW_SIZE 100
float distance_window[AVG_WINDOW_SIZE] = {0.0f};
int window_index = 0;
bool isStreaming = false;

// --- Function Prototypes ---
void handleSerialCommands();
float currentToDistance(float mA);
float calculate_mean(float arr[], int size);
float calculate_stddev(float arr[], int size, float mean);

void setup() {
  Serial.begin(115200);
  analogReference(INTERNAL);
  delay(5);

  Serial.println("------------------------------------------");
  Serial.print(PROGRAM_NAME);
  Serial.print(" v");
  Serial.println(PROGRAM_VERSION);
  Serial.println("Ready. Send 'HELP' for a list of commands.");
  Serial.println("Output unit is METERS.");
  Serial.println("------------------------------------------");
}

void loop() {
  handleSerialCommands();

  if (isStreaming) {
    int adcValue = analogRead(SENSOR_PIN);
    float voltage = adcValue * (ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION);
    float current_mA = (voltage / SHUNT_RESISTOR_OHMS) * 1000.0f;
    float distance_m = currentToDistance(current_mA);

    distance_window[window_index++] = distance_m;

    if (window_index >= AVG_WINDOW_SIZE) {
      float mean_dist_m = calculate_mean(distance_window, AVG_WINDOW_SIZE);
      
      #ifndef RELEASE
        float std_dev_dist_m = calculate_stddev(distance_window, AVG_WINDOW_SIZE, mean_dist_m);
        Serial.print("Avg: ");
        Serial.print(mean_dist_m, 6); 
        Serial.print(" m  |  StdDev: ");
        Serial.print(std_dev_dist_m, 7);
        Serial.println(" m");
      #else
        Serial.println(mean_dist_m, 6);
      #endif

      window_index = 0;
    }
    delay(SENSOR_SPEED_MS);
  }
}

/**
 * @brief Checks for and processes commands received over the serial port.
 *        Includes START, STOP, HELP, and ID commands.
 */
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "START") {
      if (!isStreaming) {
        isStreaming = true;
        window_index = 0; // Reset window on start
        Serial.println("OK: Streaming started.");
      } else {
        Serial.println("INFO: Streaming is already active.");
      }
    } else if (command == "STOP") {
      if (isStreaming) {
        isStreaming = false;
        Serial.println("OK: Streaming stopped.");
      } else {
        Serial.println("INFO: Streaming is already stopped.");
      }
    } 
    // --- ID Command Implementation ---
    else if (command == "ID") {
      Serial.print("NAME: ");
      Serial.println(PROGRAM_NAME);
      Serial.print("VERSION: ");
      Serial.println(PROGRAM_VERSION);
    } 
    // ---------------------------------
    else if (command == "HELP") {
      Serial.println("--- Available Commands ---");
      Serial.println("START   - Start streaming distance data (meters).");
      Serial.println("STOP    - Stop streaming distance data.");
      Serial.println("ID      - Get program name and version.");
      Serial.println("HELP    - Show this help message.");
      Serial.println("--------------------------");
    } else {
      Serial.print("ERR: Unknown command '");
      Serial.print(command);
      Serial.println("'. Send 'HELP' for options.");
    }
  }
}

/**
 * @brief Maps a current value (in mA) to a distance value (in METERS).
 */
float currentToDistance(float mA) {
  if (mA < (MIN_CURRENT_MA - 1.0)) {
    return -1.0;
  }
  
  float current_span = MAX_CURRENT_MA - MIN_CURRENT_MA;
  float distance_span = MAX_DISTANCE_M - MIN_DISTANCE_M;
  float current_fraction = (mA - MIN_CURRENT_MA) / current_span;
  float distance = MIN_DISTANCE_M + (current_fraction * distance_span);
  
  return distance;
}

/**
 * @brief Calculates the mean (average) of a float array.
 */
float calculate_mean(float arr[], int size) {
  float sum = 0.0f;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

/**
 * @brief Calculates the sample standard deviation of a float array.
 */
float calculate_stddev(float arr[], int size, float mean) {
  if (size <= 1) return 0.0f;
  float sum_of_squares = 0.0f;
  for (int i = 0; i < size; i++) {
    sum_of_squares += pow(arr[i] - mean, 2);
  }
  return sqrt(sum_of_squares / (size - 1));
}