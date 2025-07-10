#include <math.h>

#define RELEASE // Comment this out for verbose debugging output
#define PROGRAM_NAME "HG_C1030 Multi-Sensor Interface"
#define PROGRAM_VERSION "1.2.0"
#define STARTUP_DELAY 50

// --- General Configuration ---
#define ADC_REFERENCE_VOLTAGE 1.1f
#define ADC_RESOLUTION 1023.0f
#define AVG_WINDOW_SIZE 100          // Smoothing window size for all sensors
#define SENSOR_SAMPLE_INTERVAL_MS 10 // 10ms for high accuracy mode

#define HG_C1030_P_MIN_DIST_M 0.025f
#define HG_C1030_P_MAX_DIST_M 0.035f
#define HG_C1030_P_MIN_CURRENT_MA 4.0f
#define HG_C1030_P_MAX_CURRENT_MA 20.0f

#define SHUNT_RESISTORS_MEASURED_RESISTANCE 51.7f

struct Sensor
{
  uint8_t pin;
  float shunt_resistor_ohms;

  float min_distance_m;
  float max_distance_m;
  float min_current_ma;
  float max_current_ma;

  float distance_window[AVG_WINDOW_SIZE];
  int window_index;
};

const uint8_t SENSOR_PINS[] = {A0, A1};
const int NUM_SENSORS = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

Sensor sensors[NUM_SENSORS];

bool isStreaming = false;

void handleSerialCommands();
float currentToDistance(float mA, const Sensor &sensor);
float calculate_mean(float arr[], int size);
float calculate_stddev(float arr[], int size, float mean);
void initializeSensors();

void setup()
{
  Serial.begin(115200);

  initializeSensors();
  Serial.println("------------------------------------------");
  Serial.print(PROGRAM_NAME);
  Serial.print(" v");
  Serial.println(PROGRAM_VERSION);
  Serial.print(NUM_SENSORS);
  Serial.println(" sensors configured.");
  Serial.println("Ready. Send 'HELP' for a list of commands.");
  Serial.println("Output Format: SensorID,Value (in METERS)");
  Serial.println("------------------------------------------");

  analogReference(INTERNAL);
  delay(STARTUP_DELAY);
}

/**
 * @brief Initializes all sensors in the `sensors` array.
 *        Modify this function to set the specific parameters for each sensor.
 */
void initializeSensors()
{
  sensors[0].pin = SENSOR_PINS[0];
  sensors[0].shunt_resistor_ohms = SHUNT_RESISTORS_MEASURED_RESISTANCE;
  sensors[0].min_distance_m = HG_C1030_P_MIN_DIST_M;
  sensors[0].max_distance_m = HG_C1030_P_MAX_DIST_M;
  sensors[0].min_current_ma = HG_C1030_P_MIN_CURRENT_MA;
  sensors[0].max_current_ma = HG_C1030_P_MAX_CURRENT_MA;
  sensors[0].window_index = 0;

  sensors[1].pin = SENSOR_PINS[1];
  sensors[1].shunt_resistor_ohms = SHUNT_RESISTORS_MEASURED_RESISTANCE;
  sensors[1].min_distance_m = HG_C1030_P_MIN_DIST_M;
  sensors[1].max_distance_m = HG_C1030_P_MAX_DIST_M;
  sensors[1].min_current_ma = HG_C1030_P_MIN_CURRENT_MA;
  sensors[1].max_current_ma = HG_C1030_P_MAX_CURRENT_MA;
  sensors[1].window_index = 0;
}

// =========================================================================
// LOOP
// =========================================================================
void loop()
{
  handleSerialCommands();

  if (isStreaming)
  {
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      int adcValue = analogRead(sensors[i].pin);
      float voltage = adcValue * (ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION);
      float current_mA = (voltage / sensors[i].shunt_resistor_ohms) * 1000.0f;
      float distance_m = currentToDistance(current_mA, sensors[i]);

      sensors[i].distance_window[sensors[i].window_index] = distance_m;
      sensors[i].window_index++;
    }

    // If the data windows are full, process and print the data
    if (sensors[0].window_index >= AVG_WINDOW_SIZE)
    {
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        float mean_dist_m = calculate_mean(sensors[i].distance_window, AVG_WINDOW_SIZE);

#ifndef RELEASE
        float std_dev_dist_m = calculate_stddev(sensors[i].distance_window, AVG_WINDOW_SIZE, mean_dist_m);
        Serial.print("ID: ");
        Serial.print(i);
        Serial.print(" | Avg: ");
        Serial.print(mean_dist_m, 6);
        Serial.print(" m | StdDev: ");
        Serial.print(std_dev_dist_m, 7);
        Serial.println(" m");
#else
        // Output in CSV format: "SensorID,Value"
        Serial.print(i);
        Serial.print(",");
        Serial.println(mean_dist_m, 6);
#endif
      }

      // Reset window index for all sensors
      for (int i = 0; i < NUM_SENSORS; i++)
      {
        sensors[i].window_index = 0;
      }
    }

    delay(SENSOR_SAMPLE_INTERVAL_MS);
  }
}

void handleSerialCommands()
{
  if (Serial.available() > 0)
  {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command == "START")
    {
      if (!isStreaming)
      {
        isStreaming = true;
        // Reset all window indices on start
        for (int i = 0; i < NUM_SENSORS; i++)
        {
          sensors[i].window_index = 0;
        }
        Serial.println("OK: Streaming started.");
      }
      else
      {
        Serial.println("INFO: Streaming is already active.");
      }
    }
    else if (command == "STOP")
    {
      if (isStreaming)
      {
        isStreaming = false;
        Serial.println("OK: Streaming stopped.");
      }
      else
      {
        Serial.println("INFO: Streaming is already stopped.");
      }
    }
    else if (command == "ID")
    {
      Serial.print("NAME: ");
      Serial.println(PROGRAM_NAME);
      Serial.print("VERSION: ");
      Serial.println(PROGRAM_VERSION);
      Serial.print("SENSORS_CONFIGURED: ");
      Serial.println(NUM_SENSORS);
    }
    else if (command == "HELP")
    {
      Serial.println("--- Available Commands ---");
      Serial.println("START   - Start streaming data from all sensors.");
      Serial.println("STOP    - Stop streaming data.");
      Serial.println("ID      - Get program name, version, and sensor count.");
      Serial.println("HELP    - Show this help message.");
      Serial.println("--- Output Format ---");
      Serial.println("SensorID,Value (e.g., '0,0.031254')");
      Serial.println("--------------------------");
    }
    else
    {
      Serial.print("ERR: Unknown command '");
      Serial.print(command);
      Serial.println("'. Send 'HELP' for options.");
    }
  }
}

/**
 * @brief Maps a current value (in mA) to a distance (in METERS)
 * @param mA The measured current in milliamps.
 * @param sensor A constant reference to the sensor's configuration struct.
 */
float currentToDistance(float mA, const Sensor &sensor)
{
  if (mA < (sensor.min_current_ma - 0.1))
  {
    return -1.0; // Return an error value for out-of-range low current
  }

  float current_span = sensor.max_current_ma - sensor.min_current_ma;
  float distance_span = sensor.max_distance_m - sensor.min_distance_m;
  float current_fraction = (mA - sensor.min_current_ma) / current_span;
  float distance = sensor.min_distance_m + (current_fraction * distance_span);

  return distance;
}

/**
 * @brief Calculates the mean (average) of a float array. (Unchanged)
 */
float calculate_mean(float arr[], int size)
{
  float sum = 0.0f;
  for (int i = 0; i < size; i++)
  {
    sum += arr[i];
  }
  return sum / size;
}

/**
 * @brief Calculates the sample standard deviation of a float array. (Unchanged)
 */
float calculate_stddev(float arr[], int size, float mean)
{
  if (size <= 1)
    return 0.0f;
  float sum_of_squares = 0.0f;
  for (int i = 0; i < size; i++)
  {
    sum_of_squares += pow(arr[i] - mean, 2);
  }
  return sqrt(sum_of_squares / (size - 1));
}