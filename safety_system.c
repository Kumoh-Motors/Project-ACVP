#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include <sys/ipc.h>
#include <sys/shm.h>

// Pin definitions
#define CS_GPIO 8
#define TRIG_PIN 16
#define ECHO_PIN 17
#define PWM0 20  // Motor 1
#define PWM1 21
#define PWM2 22  // Motor 2
#define PWM3 23
#define PWM4 24  // Motor 3
#define PWM5 25
#define PWM6 26  // Motor 4
#define PWM7 27

// SPI settings
#define SPI_CH 0
#define SPI_SPEED 1000000
#define SPI_MODE 3

// ADXL345 registers
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37

// Safety parameters
#define BRAKING_ACCELERATION 5.0
#define REACTION_TIME 1.0
#define WINDOW_SIZE 20
#define CALIBRATION_SAMPLES 100

// Bluetooth settings
#define BAUD_RATE 115200
static const char* UART2_DEV = "/dev/ttyAMA2";

// Global variables
float g_speed = 0.0;
float g_ultrasonic_distance = 0.0;
int g_bluetooth_command = -1;
bool g_person_detected = false;
bool g_running = true;

// Mutex
pthread_mutex_t speed_mutex;
pthread_mutex_t sensor_mutex;

// Structures
typedef struct {
    float values[WINDOW_SIZE];
    int index;
    int count;
} MovingWindow;

typedef struct {
    float offsetX;
    float offsetY;
} SensorData;

struct SharedData {
    bool person_detected;
    bool program_running;
};

// Function declarations
void readRegister_ADXL345(char registerAddress, int numBytes, char* values);
void writeRegister_ADXL345(char address, char value);
float convertToG(short rawValue);
void calibrateSensor(SensorData* sensorData);
void initWindow(MovingWindow* window);
void addValue(MovingWindow* window, float value);
float getVariance(MovingWindow* window);
void moter_Rotate(int speed, int dir, int motor_pwm0, int motor_pwm1);
double measure_distance();
float calculate_safe_distance(float velocity);

// Create thread macro
#define CREATE_THREAD(idx, func, arg) \
    if (pthread_create(&threads[idx], NULL, func, arg) != 0) { \
        printf(#func " 스레드 생성 실패\n"); \
        g_running = false; \
        return 1; \
    }

/* ********************
 * ADXL345 functions
 ******************** */
void readRegister_ADXL345(char registerAddress, int numBytes, char * values) {
    values[0] = 0x80 | registerAddress;
    if(numBytes > 1) values[0] = values[0] | 0x40;
    
    digitalWrite(CS_GPIO, LOW);
    wiringPiSPIDataRW(SPI_CH, values, numBytes + 1);
    digitalWrite(CS_GPIO, HIGH);
}

void writeRegister_ADXL345(char address, char value) {
    unsigned char buff[2];
    buff[0] = address;
    buff[1] = value;
    
    digitalWrite(CS_GPIO, LOW);
    wiringPiSPIDataRW(SPI_CH, buff, 2);
    digitalWrite(CS_GPIO, HIGH);
}

float convertToG(short rawValue) {
    return (float)rawValue * 4.0 / 512.0;
}

/* ********************
 * MovingWindow functions
 ******************** */
void initWindow(MovingWindow *window) {
    window->index = 0;
    window->count = 0;
    for(int i = 0; i < WINDOW_SIZE; i++) {
        window->values[i] = 0;
    }
}

void addValue(MovingWindow *window, float value) {
    window->values[window->index] = value;
    window->index = (window->index + 1) % WINDOW_SIZE;
    if(window->count < WINDOW_SIZE) window->count++;
}

float getVariance(MovingWindow *window) {
    if(window->count == 0) return 0;
    
    float mean = 0;
    float sum = 0;
    
    for(int i = 0; i < window->count; i++) {
        mean += window->values[i];
    }
    mean /= window->count;
    
    for(int i = 0; i < window->count; i++) {
        float diff = window->values[i] - mean;
        sum += diff * diff;
    }
    
    return sum / window->count;
}

// Sensor Calibration
void calibrateSensor(SensorData *sensorData) {
    unsigned char buffer[100];
    float sumX = 0, sumY = 0;
    
    printf("Sensor calibration started... Keep the vehicle in a stationary state.\n");
    
    for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
        readRegister_ADXL345(DATAX0, 6, buffer);
        short x = ((short)buffer[2]<<8)|(short)buffer[1];
        short y = ((short)buffer[4]<<8)|(short)buffer[3];
        
        sumX += convertToG(x);
        sumY += convertToG(y);
        delay(10);
    }
    
    sensorData->offsetX = sumX / CALIBRATION_SAMPLES;
    sensorData->offsetY = sumY / CALIBRATION_SAMPLES;
    
    printf("Calibration complete!\n");
    printf("Offset value - X: %.4f, Y: %.4f\n", sensorData->offsetX, sensorData->offsetY);
}

// Ultrasonic Sensor Distance Measurement
double measure_distance() {
    long pulse_start, pulse_end;
    
    // TRIG Pin Initialization
    digitalWrite(TRIG_PIN, LOW);
    delay(1);
    
    // Generate TRIG signal
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Modify ECHO pin wait time
    long timeout = micros() + 30000;  // 30ms timeout
    
    // Wait for ECHO pin to be HIGH
    while(digitalRead(ECHO_PIN) == LOW) {
        if(micros() > timeout) {
            return 999.0;  // Return default value on timeout
        }
    }
    pulse_start = micros();
    
    // Wait for the ECHO pin to be LOW
    while(digitalRead(ECHO_PIN) == HIGH) {
        if(micros() > timeout) {
            return 999.0;  // Return default value on timeout
        }
    }
    pulse_end = micros();
    
    // Distance calculation
    double distance = (pulse_end - pulse_start) * 0.0343 / 2.0;
    
    // Check valid range (2cm to 400cm)
    if(distance > 400 || distance < 2) {
        return 999.0;
    }
    return distance;
}

// Calculate safe distance
float calculate_safe_distance(float velocity) {
    // Calculate safe distance based on speed (m/s)
    float reaction_distance = velocity * REACTION_TIME;
    float braking_distance = (velocity * velocity) / (2 * BRAKING_ACCELERATION);
    float safety_margin = 45.0;

    return reaction_distance * 100 + braking_distance * 100 + safety_margin;  // Convert to cm
}

/* Serial communication functions */
unsigned char serialRead(const int fd) {
    unsigned char x;
    if(read(fd, &x, 1) != 1) return -1;
    return x;
}

void serialWrite(const int fd, const unsigned char c) {
    write(fd, &c, 1);
}

void serialWriteBytes(const int fd, const char *s) {
    write(fd, s, strlen(s));
}

// Motor rotation control function
void moter_Rotate(int speed, int dir, int motor_pwm0, int motor_pwm1){
    // speed: 0~100
    // dir: 0/1 = CW/CCW
    if(dir == 0){
        softPwmWrite(motor_pwm0, 0);     // Off for motor_pwm0
        softPwmWrite(motor_pwm1, speed); // On for motor_pwm1
    }
    else if(dir == 1){
        softPwmWrite(motor_pwm0, speed); // On for motor_pwm0
        softPwmWrite(motor_pwm1, 0);     // Off for motor_pwm1
    }
}

/* ********************
 * Motor control functions
 ******************** */
void stop_motors() {
    moter_Rotate(0, 0, PWM0, PWM1);
    moter_Rotate(0, 0, PWM2, PWM3);
    moter_Rotate(0, 0, PWM4, PWM5);
    moter_Rotate(0, 0, PWM6, PWM7);
}

void move_forward() {
    moter_Rotate(60, 0, PWM0, PWM1);
    moter_Rotate(60, 0, PWM2, PWM3);
    moter_Rotate(60, 0, PWM4, PWM5);
    moter_Rotate(60, 0, PWM6, PWM7);
}

void move_backward() {
    moter_Rotate(60, 1, PWM0, PWM1);
    moter_Rotate(60, 1, PWM2, PWM3);
    moter_Rotate(60, 1, PWM4, PWM5);
    moter_Rotate(60, 1, PWM6, PWM7);
}

void turn_left() {
    moter_Rotate(70, 0, PWM0, PWM1);
    moter_Rotate(0, 1, PWM2, PWM3);
    moter_Rotate(70, 0, PWM4, PWM5);
    moter_Rotate(0, 1, PWM6, PWM7);
}

void turn_right() {
    moter_Rotate(0, 1, PWM0, PWM1);
    moter_Rotate(70, 0, PWM2, PWM3);
    moter_Rotate(0, 1, PWM4, PWM5);
    moter_Rotate(70, 0, PWM6, PWM7);
}

/* ********************
 * Thread Part
 ******************** */

// Camera thread
void* camera_thread(void* arg) {
    key_t key = ftok("/tmp", 65);
    int shmid;
    struct SharedData* sharedData = NULL;
    int retry_count = 0;
    
    while(g_running) {
        if(sharedData == NULL) {
            shmid = shmget(key, sizeof(struct SharedData), 0666);
            if(shmid != -1) {
                sharedData = (struct SharedData*)shmat(shmid, NULL, 0);
                if(sharedData == (void*)-1) {
                    sharedData = NULL;
                }
            }
            if(sharedData == NULL) {
                delay(1000); 
                retry_count++;
                if(retry_count > 10) {
                    printf("Camera program connection failed\n");
                    g_person_detected = false;
                    delay(5000);
                    retry_count = 0;
                }
                continue;
            }
        }
        
        pthread_mutex_lock(&sensor_mutex);
        g_person_detected = sharedData->person_detected;
        pthread_mutex_unlock(&sensor_mutex);
        
        if(!sharedData->program_running) {
            shmdt(sharedData);
            sharedData = NULL;
            printf("Attempting to reconnect the camera program...\n");
        }
        
        delay(100);
    }
    
    if(sharedData) {
        shmdt(sharedData);
    }
    return NULL;
}

// Speed measurement thread
void* speed_measurement(void* arg) {
    SensorData* sensorData = (SensorData*)arg;
    unsigned char buffer[100];
    float vx = 0, vy = 0;
    float dt = 0.05;
    const float g = 9.81;
    const float MOTION_THRESHOLD = 0.03;
    const float STOP_VARIANCE_THRESHOLD = 0.0001;
    
    MovingWindow windowX, windowY;
    initWindow(&windowX);
    initWindow(&windowY);
    
    while(g_running) {
        readRegister_ADXL345(DATAX0, 6, buffer);
        
        float ax = convertToG(((short)buffer[2]<<8)|(short)buffer[1]) - sensorData->offsetX;
        float ay = convertToG(((short)buffer[4]<<8)|(short)buffer[3]) - sensorData->offsetY;
        
        addValue(&windowX, ax);
        addValue(&windowY, ay);
        
        float varX = getVariance(&windowX);
        float varY = getVariance(&windowY);
        
        if(varX < STOP_VARIANCE_THRESHOLD && varY < STOP_VARIANCE_THRESHOLD) {
            vx *= 0.8;
            vy *= 0.8;
            if(fabs(vx) < 0.1) vx = 0;
            if(fabs(vy) < 0.1) vy = 0;
        } else {
            if(fabs(ax) > MOTION_THRESHOLD) vx += (ax * g) * dt;
            if(fabs(ay) > MOTION_THRESHOLD) vy += (ay * g) * dt;
            vx *= 0.98;
            vy *= 0.98;
        }
        
        pthread_mutex_lock(&speed_mutex);
        g_speed = sqrt(vx*vx + vy*vy);
        //printf("V Speed: %.2lfkm/h\n", g_speed); // Debugging
        pthread_mutex_unlock(&speed_mutex);
        
        delay(50);
    }
    return NULL;
}

void* ultrasonic_thread(void* arg) {
    int fail_count = 0;
    const int MAX_FAILS = 3;  // Allowable number of continuous failures
    
    while(g_running) {
        double distance = measure_distance();
        
        pthread_mutex_lock(&sensor_mutex);
        
        if(distance != 999.0) {
            g_ultrasonic_distance = distance;
            fail_count = 0;  // Reset failure count on success
            //printf("Distance: %.2f cm\n", distance);  // Debugging
        } else {
            fail_count++;
            if(fail_count >= MAX_FAILS) {
                g_ultrasonic_distance = 999.0;
                //printf("[!!!Failure!!!] Ultrasonic sensor read failure\n");  // Debugging
                fail_count = 0;
            }
        }
        
        pthread_mutex_unlock(&sensor_mutex);
        delay(100);  // Delay for sensor stabilization
    }
    return NULL;
}

// Bluetooth command thread
void* bluetooth_thread(void* arg) {
    int fd_serial;
    
    if ((fd_serial = serialOpen(UART2_DEV, BAUD_RATE)) < 0) {
        printf("Unable to open serial device.\n");
        return NULL;
    }
    
    while(g_running) {
        if(serialDataAvail(fd_serial)) {
            char dat = serialRead(fd_serial);      

            pthread_mutex_lock(&sensor_mutex);
            printf("!!COMMAND!!: %c\n", dat);
            switch(dat) {
                case '1': g_bluetooth_command = 1; break; // Forward
                case '2': g_bluetooth_command = 2; break; // Right
                case '3': g_bluetooth_command = 3; break; // Left
                case '4': g_bluetooth_command = 4; break; // Stop
                case '5': g_bluetooth_command = 5; break; // Back
            }
            pthread_mutex_unlock(&sensor_mutex);
        }
        delay(10);
    }
    serialClose(fd_serial);
    return NULL;
}

// Safety control thread
void* safety_control_thread(void* arg) {
    while(g_running) {
        pthread_mutex_lock(&speed_mutex);
        float current_speed = g_speed;
        pthread_mutex_unlock(&speed_mutex);
        
        pthread_mutex_lock(&sensor_mutex);
        bool person_detected = g_person_detected;
        float current_distance = g_ultrasonic_distance;
        int command = g_bluetooth_command;
        pthread_mutex_unlock(&sensor_mutex);
        
        float safe_distance = calculate_safe_distance(current_speed);
        bool unsafe_condition = (person_detected && 
                               current_distance < safe_distance && 
                               current_distance != 999.0);

        //printf("safe_distance: %.2f cm\n", safe_distance); // Debugging

        if(unsafe_condition) {
            stop_motors();
            printf("STOP!!! [Current distance: %.2f cm]\n", current_distance);
            delay(2000);
        } else {
            switch(command) {
                case 1: 
                    move_forward(); 
                    break;
                case 2: 
                    turn_right(); 
                    break;
                case 3: 
                    turn_left(); 
                    break;
                case 4: 
                    stop_motors(); 
                    break;
                case 5: 
                    move_backward(); 
                    break;
            }
        }
        
        delay(100);
    }
    return NULL;
}

/* ********************
 * Main
 ******************** */
int main() {
    if (wiringPiSetupGpio() == -1) {
        printf("GPIO initialization failed\n");
        return 1;
    }
    if (wiringPiSPISetupMode(SPI_CH, SPI_SPEED, SPI_MODE) == -1) {
        printf("SPI initialization failed\n");
        return 1;
    }

    // Initialize mutexes
    if (pthread_mutex_init(&speed_mutex, NULL) != 0) {
        printf("Speed mutex initialization failed\n");
        return 1;
    }
    if (pthread_mutex_init(&sensor_mutex, NULL) != 0) {
        printf("Sensor mutex initialization failed\n");
        pthread_mutex_destroy(&speed_mutex);
        return 1;
    }

    // Initialize GPIO pins
    pinMode(CS_GPIO, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize motor pins and PWM
    int pins[] = { PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7 };
    for (int i = 0; i < 8; i++) {
        pinMode(pins[i], OUTPUT);
        if (softPwmCreate(pins[i], 0, 100) != 0) {
            printf("PWM initialization failed: %d\n", pins[i]);
            return 1;
        }
    }

    // Initialize ADXL345
    writeRegister_ADXL345(DATA_FORMAT, 0x01);
    writeRegister_ADXL345(BW_RATE, 0x0C);
    writeRegister_ADXL345(POWER_CTL, 0x08);

    // Initialize sensor data
    SensorData sensorData;
    calibrateSensor(&sensorData);

    // Create threads
    pthread_t threads[5];

    CREATE_THREAD(0, camera_thread, NULL);
    CREATE_THREAD(1, speed_measurement, &sensorData);
    CREATE_THREAD(2, ultrasonic_thread, NULL);
    CREATE_THREAD(3, bluetooth_thread, NULL);
    CREATE_THREAD(4, safety_control_thread, NULL);

    printf("Safety system running... (Press 'q' to exit)\n");

    char input;
    while (g_running) {
        input = getchar();
        if (input == 'q' || input == 'Q') {
            printf("\nShutting down the program...\n");
            g_running = false;
            break;
        }
        delay(100);
    }

    for (int i = 0; i < 5; i++) {
        pthread_join(threads[i], NULL);
    }

    stop_motors();

    for (int i = 0; i < 8; i++) {
        softPwmStop(pins[i]);
    }

    pthread_mutex_destroy(&speed_mutex);
    pthread_mutex_destroy(&sensor_mutex);

    printf("The program has terminated safely.\n");
    return 0;
}