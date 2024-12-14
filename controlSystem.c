/**
 * Justin Mahr
 * controlSystem.c
 * Lab 13
 * 12/13/2024
 * This program simulates sensor data (temperature, pressure, and level) and uses a PID controller
 * to control an actuator based on the sensor readings. It runs on two threads. The control system adjusts
 * the actuator position based on the target temperature and pressure, with the results placed into an Excel
 * sheet.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <xlsxwriter.h>
#include <termios.h>
#include <fcntl.h>

/**
 * Holds sensor readings and a mutex for thread-safe access.
 */
typedef struct {
    float temperature;
    float pressure;
    int level;
    pthread_mutex_t lock;
} sensorData;

/**
 * Stores parameters and state for a PID controller.
 */
typedef struct {
    float kp;
    float ki; 
    float kd; 
    float integral;
    float previous_error;
} PIDController;

// Prototype Functions
int testADC();
float convertADCtoTemperature(int ADCValue);
float convertADCtoPressure(int ADCValue);
float convertADCtoLevel(int ADCValue);
float computePID(PIDController *pid, float setPoint, float measuredValue);
void *controlSystem(void *arg);
void *readSensorData(void *arg);
void writeToExcel(float temperature, float pressure, float level, float actuatorPosition, float DACoutput);
int kbhit(void);
int getKeyPress();
void cleanup();

/**
 * Defines the sleep time in microseconds for loop delays.
 */
#define SLEEPTIME 50000

/**
 * Defines the maximum allowable value for the integral term in the PID controller.
 */
#define MAX_INTEGRAL 100.0f

/**
 * A flag to control the main loop, set to 1 to stop the loop.
 */
volatile int stopLoop = 0;

/**
 * Pointer to the workbook for Excel file operations.
 */
static lxw_workbook *workbook = NULL;

/**
 * Pointer to the worksheet for Excel file operations.
 */
static lxw_worksheet *worksheet = NULL;

/**
 * The main function initializes the system, processes input, and starts threads.
 */
int main(int argc, char *argv[]) {
    atexit(cleanup);

    srand(time(NULL)); // Seed

    float initialTemperature = 0.0;
    float initialPressure = 0.0;
    float initialLevel = 0.0;

     if (argc == 4) {
        // Parse command-line arguments
        initialTemperature = atof(argv[1]);
        initialPressure = atof(argv[2]);
        initialLevel = atof(argv[3]);
    } else {
        // Random values for testing
        initialTemperature = convertADCtoTemperature(testADC());
        initialPressure = convertADCtoPressure(testADC());
        initialLevel = convertADCtoLevel(testADC());
    }
    printf("Temperature: %.2f\n", initialTemperature);
    printf("Pressure: %.2f\n", initialPressure);
    printf("Level: %.2f\n", initialLevel);

    // Shared sensor data
    sensorData data = {initialTemperature, initialPressure, initialLevel, PTHREAD_MUTEX_INITIALIZER};

    // Threads
    pthread_t sensorThread, controlThread;

    // Create threads
    pthread_create(&sensorThread, NULL, readSensorData, &data);
    pthread_create(&controlThread, NULL, controlSystem, &data);

    pthread_join(sensorThread, NULL);
    pthread_join(controlThread, NULL);

    return 0;
}

/**
 * Calculates the PID control output based on the setpoint and measured value.
 */
float computePID(PIDController *pid, float setPoint, float measuredValue) {
    float error = setPoint - measuredValue;
    pid->integral += error;

    if (pid->integral > MAX_INTEGRAL) pid->integral = MAX_INTEGRAL;
    if (pid->integral < -MAX_INTEGRAL) pid->integral = -MAX_INTEGRAL;

    float derivative = error - pid -> previous_error;
    float output = pid -> kp * error + pid -> ki * pid -> integral + pid -> kd * derivative; 
    pid -> previous_error = error;

    return output;
}

/**
 * Reads sensor data and updates the shared sensorData structure.
 */
void *readSensorData(void *arg) {
    sensorData *data = (sensorData *)arg;

    static int firstRun = 1;

    while (1) {
        float temperature, pressure, level;

        if (firstRun) {
            pthread_mutex_lock(&data->lock);
            temperature = data->temperature;
            pressure = data->pressure;
            level = data->level;
            pthread_mutex_unlock(&data->lock);

            firstRun = 0; 
        } else {
            // Generate random values for testing
            int adcTemperature = testADC();
            int adcPressure = testADC();
            int adcLevel = testADC();

            temperature = convertADCtoTemperature(adcTemperature);
            pressure = convertADCtoPressure(adcPressure);
            level = convertADCtoLevel(adcLevel);
        }

        // Update the shared structure
        pthread_mutex_lock(&data->lock);
        data->temperature = temperature;
        data->pressure = pressure;
        data->level = level;
        pthread_mutex_unlock(&data->lock);

        // Sleep
        usleep(SLEEPTIME);
    }

    return NULL;
}

/**
 * Controls the system based on sensor data using PID controllers.
 */
void *controlSystem(void *arg) {
    sensorData *data = (sensorData *)arg;

    // Initialize PID's
    PIDController temperaturePID = {0.1, 0.01, 0.05, 0, 0};
    PIDController pressurePID = {0.2, 0.02, 0.1, 0, 0};

    // Target temperature in Celcius
    float targetTemperature = 75.0;

    // Target pressure
    float targetPressure = 5.0;
    printf("_________________________________________________________________________________________________________\n");

    while(!stopLoop) {

        pthread_mutex_lock(&data->lock);
        float temperature = data->temperature;
        float pressure = data->pressure;
        float level = data->level;
        pthread_mutex_unlock(&data->lock);

        float temperatureControlUnit = computePID(&temperaturePID, targetTemperature, temperature);
        float pressureControlUnit = computePID(&pressurePID, targetPressure, pressure);

        float actuatorPosition = temperatureControlUnit + pressureControlUnit + level * 0.5;

        if (actuatorPosition < 0.0) {
            actuatorPosition = 0.0;
        }
        if (actuatorPosition > 10.0) {
            actuatorPosition = 10.0;
        }

        float DACoutput = actuatorPosition;

        float actuatorPercentage = actuatorPosition * 10.0;

        writeToExcel(temperature, pressure, level, actuatorPercentage, DACoutput);

        printf("| Actuator Position: %8.2f%% | Temp: %7.2f°C | Pressure: %6.2f bar | Level: %5.2f m | DAC: %5.2f V |\n", actuatorPercentage, temperature, pressure, level, DACoutput);
        usleep(SLEEPTIME);

        if (kbhit()) {
            // Get key press
            int key = getKeyPress(); 
        
            // Check for Enter or Space
            if (key == 10 || key == 32 || key == 13) {
                stopLoop = 1;
                printf("_________________________________________________________________________________________________________\n");
                exit(1);
            }
        }
    }
  
    return NULL;
}

/**
 * Simulates an ADC reading by generating a random value between 0 and 1023.
 */
int testADC() {
    return rand() % 1024;
}

/**
 * Converts an ADC value to a temperature in Celsius.
 */
float convertADCtoTemperature(int ADCValue) {
    return ((float)ADCValue / 1023) * 100.0;
}

/**
 * Converts an ADC value to a pressure in bar.
 */
float convertADCtoPressure(int ADCValue) {
    return ((float)ADCValue / 1023) * 10.0;
}

/**
 * Converts an ADC value to a level in meters.
 */
float convertADCtoLevel(int ADCValue) {
    return ((float)ADCValue / 1023) * 5.0;
}

/**
 * Writes sensor data to an Excel file and creates a chart.
 */
void writeToExcel(float temperature, float pressure, float level, float actuatorPosition, float DACoutput) {
    static int row = 1;  
    static pthread_mutex_t fileLock = PTHREAD_MUTEX_INITIALIZER;

    pthread_mutex_lock(&fileLock); 

    if (workbook == NULL) {
        // printf("Creating workbook and worksheet...\n");
        workbook = workbook_new("sensor_data.xlsx");
        worksheet = workbook_add_worksheet(workbook, "Sensor Data");

        // Write the header
        worksheet_write_string(worksheet, 0, 0, "Temperature", NULL);
        worksheet_write_string(worksheet, 0, 1, "Pressure", NULL);
        worksheet_write_string(worksheet, 0, 2, "Level", NULL);
        worksheet_write_string(worksheet, 0, 3, "Actuator Position", NULL);
        worksheet_write_string(worksheet, 0, 4, "DAC Output", NULL);
    }

    // Write the data to the worksheet
    // printf("Writing data to Excel: %.2f, %.2f, %.2f, %.2f, %.2f\n", temperature, pressure, level, actuatorPosition, DACoutput);
    worksheet_write_number(worksheet, row, 0, temperature, NULL);
    worksheet_write_number(worksheet, row, 1, pressure, NULL);
    worksheet_write_number(worksheet, row, 2, level, NULL);
    worksheet_write_number(worksheet, row, 3, actuatorPosition, NULL);
    worksheet_write_number(worksheet, row, 4, DACoutput, NULL);
    row++;

    // Create a chart
    lxw_chart *chart = workbook_add_chart(workbook, LXW_CHART_SCATTER);

    // Configure the chart
    lxw_chart_series *series = chart_add_series(chart, "=Sensor Data!$A$2:$A$100", "=Sensor Data!$E$2:$E$100");
    chart_title_set_name(chart, "Temperature vs DAC Output");
    chart_axis_set_name(chart->x_axis, "Temperature (Celsius)");
    chart_axis_set_name(chart->y_axis, "DAC Output");
    chart_series_set_trendline(series, LXW_CHART_TRENDLINE_TYPE_LINEAR, 0);
    chart_series_set_trendline_equation(series);
    chart_series_set_trendline_r_squared(series);

    // Insert the chart
    worksheet_insert_chart(worksheet, CELL("G2"), chart);

    pthread_mutex_unlock(&fileLock); 
}

/**
 * Cleans up and closes the workbook.
 */
void cleanup() {
    if (workbook != NULL) {
        // printf("Closing workbook...\n");
        workbook_close(workbook);
        workbook = NULL;
    }
}

/**
 * Checks if a key has been pressed.
 */

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt); 
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0); 
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK); 
    ch = getchar(); 
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
    fcntl(STDIN_FILENO, F_SETFL, oldf); 

    // A key was pressed
    if(ch != EOF) { 
        ungetc(ch, stdin); 
        return 1;
    }
    // No key was pressed
    return 0; 
}

/**
 * Reads a single key press from the user.
 */

int getKeyPress() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt); 
    newt = oldt;                  
    newt.c_lflag &= ~(ICANON | ECHO); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); 
    ch = getchar();  
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 

    // Returns pressed key character
    return ch;
}