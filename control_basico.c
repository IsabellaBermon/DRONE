#include <stdio.h>
#include <stdint.h>

#include <stdio.h>

#include "nrf24_driver.h"
#include "pico/stdlib.h"
#include "MPU6050_i2c.h"
#include "pico/time.h"
#include <tusb.h> // TinyUSB tud_cdc_connected()

// IMU
#define SDA_MPU 26 //pin 31
#define SCL_MPU 27 //pin 32

// Parámetros del controlador PID
float Kp = 0.5;   // Ganancia proporcional
float Ki = 0.2;   // Ganancia integral
float Kd = 0.1;   // Ganancia derivativa

// Variables del controlador PID
float setpoint = 0.0;   // Valor de referencia
float prev_error_pitch = 0.0; // Error previo
float integral_pitch = 0.0;   // Término integral
float prev_error_roll = 0.0; // Error previo
float integral_roll = 0.0;   // Término integral
float prev_error_empuje = 0.0; // Error previo
float integral_empuje = 0.0;   // Término integral
float angleMax = 50.0;
float PWMangle = 64;

// Función de controlador PID
float pid_controller_pitch(float input, float setpoint)
{
    // Calcula el error
    float error = setpoint - input;
    // Calcula los términos del controlador
    float proportional = Kp * error;
    integral_pitch += Ki * error;
    float derivative = Kd * (error - prev_error_pitch);
    // Calcula la salida del controlador
    float output = proportional + integral_pitch + derivative;
    // Actualiza el error previo
    prev_error_pitch = error;
    return output;
}
float pid_controller_roll(float input, float setpoint)
{
    // Calcula el error
    float error = setpoint - input;
    // Calcula los términos del controlador
    float proportional = Kp * error;
    integral_roll += Ki * error;
    float derivative = Kd * (error - prev_error_m2);
    // Calcula la salida del controlador
    float output = proportional + integral_roll + derivative;
    // Actualiza el error previo
    prev_error_m2 = error;
    return output;
}
float pid_controller_altitud(float input, float setpoint)
{
    // Calcula el error
    float error = setpoint - input;
    // Calcula los términos del controlador
    float proportional = Kp * error;
    integral_empuje += Ki * error;
    float derivative = Kd * (error - prev_error_empuje);
    // Calcula la salida del controlador
    float output = proportional + integral_empuje + derivative;
    // Actualiza el error previo
    prev_error_empuje = error;
    return output;
}

float map_range(float value, float in_min, float in_max, float out_min, float out_max)
{
    // Realiza el mapeo del valor desde el rango inicial al rango final
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculateSetPoint_roll(float angle) {
    float set_point = 0;
    if (angle <= 545) { // pensando que roll arriba es positivo
        set_point = map_range(angle,1/545,1/234,0,10);
        // en posición 0 da 545 por lo que se deben invertir el min y max
    } else if (angle >  545) { // pensando que roll abajo es negativo
        set_point = map_range(angle,546,870,0,-10);
    }
    return set_point;
}

float calculateSetPoint_pitch(float angle) {
    float set_point = 0;
    if (angle <= 559) { // pensando que pitch a la derecha es negativo
        set_point = map_range(angle,241,559,0, -10);
        // en posición 0 da 545 por lo que se deben invertir el min y max
    } else if (angle >  559) { // pensando que pitch a la derecha es positivo
        set_point = map_range(angle,560,862,0,10);
    }
    return set_point;
}

int constrain(int valor, int minimo, int maximo) {
    if (valor < minimo) {
        return minimo;
    } else if (valor > maximo) {
        return maximo;
    } else {
        return valor;
    }
}


int main(void)
{
    // Initialize all present standard stdio types
    stdio_init_all();

    // Lectura de sensórica -----------------------------------------------
    mpu6050_init(SDA_MPU, SCL_MPU);
    int16_t acceleration[3], gyro[3], magneto[3];

    // Configurar el pin GPIO para usar PWM --------------------------------
    gpio_set_function(PWM_PIN1, GPIO_FUNC_PWM);
    uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN1);

    gpio_set_function(PWM_PIN2, GPIO_FUNC_PWM);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN2);

    gpio_set_function(PWM_PIN3, GPIO_FUNC_PWM);
    uint slice_num3 = pwm_gpio_to_slice_num(PWM_PIN3);

    gpio_set_function(PWM_PIN4, GPIO_FUNC_PWM);
    uint slice_num4 = pwm_gpio_to_slice_num(PWM_PIN4);

    // Configurar el divisor de reloj para el módulo PWM
    pwm_set_clkdiv(slice_num1, 16.0f);  // Frecuencia de reloj base de 125 MHz / 4 = 31.25 MHz
    pwm_set_clkdiv(slice_num2, 16.0f);
    pwm_set_clkdiv(slice_num3, 16.0f);
    pwm_set_clkdiv(slice_num4, 16.0f);

    // Configurar el rango del PWM
    pwm_set_wrap(slice_num1, PWM_RANGE);
    pwm_set_chan_level(slice_num1, PWM_CHAN_A, 0);

    pwm_set_wrap(slice_num2, PWM_RANGE);
    pwm_set_chan_level(slice_num2, PWM_CHAN_A, 0);

    pwm_set_wrap(slice_num3, PWM_RANGE);
    pwm_set_chan_level(slice_num3, PWM_CHAN_A, 0);

    pwm_set_wrap(slice_num4, PWM_RANGE);
    pwm_set_chan_level(slice_num4, PWM_CHAN_A, 0);

    // Habilitar el PWM
    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
    pwm_set_enabled(slice_num3, true);
    pwm_set_enabled(slice_num4, true);

    // Inicializar repección de control remoto  --------------------------------
    // // wait until the CDC ACM (serial port emulation) is connected
    // while (!tud_cdc_connected()) 
    // {
    //     sleep_ms(10);
    // }

    // GPIO pin numbers
    pin_manager_t my_pins = { 
        .sck = 2,
        .copi = 3, 
        .cipo = 4, 
        .csn = 5, 
        .ce = 6 
    };

    /**
     * nrf_manager_t can be passed to the nrf_client_t
     * initialise function, to specify the NRF24L01 
     * configuration. If NULL is passed to the initialise 
     * function, then the default configuration will be used.
     */
    nrf_manager_t my_config = {
        // RF Channel 
        .channel = 125,

        // AW_3_BYTES, AW_4_BYTES, AW_5_BYTES
        .address_width = AW_5_BYTES,

        // dynamic payloads: DYNPD_ENABLE, DYNPD_DISABLE
        .dyn_payloads = DYNPD_ENABLE,

        // data rate: RF_DR_250KBPS, RF_DR_1MBPS, RF_DR_2MBPS
        .data_rate = RF_DR_1MBPS,

        // RF_PWR_NEG_18DBM, RF_PWR_NEG_12DBM, RF_PWR_NEG_6DBM, RF_PWR_0DBM
        .power = RF_PWR_NEG_12DBM,

        // retransmission count: ARC_NONE...ARC_15RT
        .retr_count = ARC_10RT,

        // retransmission delay: ARD_250US, ARD_500US, ARD_750US, ARD_1000US
        .retr_delay = ARD_500US 
    };

    // SPI baudrate
    uint32_t my_baudrate = 5000000;

    // provides access to driver functions
    nrf_client_t my_nrf;

    // initialise my_nrf
    nrf_driver_create_client(&my_nrf);

    // configure GPIO pins and SPI
    my_nrf.configure(&my_pins, my_baudrate);

    // not using default configuration (my_nrf.initialise(NULL)) 
    my_nrf.initialise(&my_config);

    /**
     * set addresses for DATA_PIPE_0 - DATA_PIPE_3.
     * These are addresses the transmitter will send its packets to.
     */
    my_nrf.rx_destination(DATA_PIPE_0, (uint8_t[]){0x37,0x37,0x37,0x37,0x37});
    my_nrf.rx_destination(DATA_PIPE_1, (uint8_t[]){0xC7,0xC7,0xC7,0xC7,0xC7});
    my_nrf.rx_destination(DATA_PIPE_2, (uint8_t[]){0xC8,0xC7,0xC7,0xC7,0xC7});
    my_nrf.rx_destination(DATA_PIPE_3, (uint8_t[]){0xC9,0xC7,0xC7,0xC7,0xC7});

    // set to RX Mode
    my_nrf.receiver_mode();

    uint8_t coor[32];

    typedef struct payload_rc_s
    {
        int16_t roll = 0;
        int16_t pitch = 0;
        int16_t empuje = 0;
    } payload_rc_t;

    payload_rc_t payload_rc;

    // data pipe number a packet was received on
    uint8_t pipe_number = 0;
    
  while (1)
  {
    // Leer señal de control remoto
    if (my_nrf.is_packet(&pipe_number))
    {
      switch (pipe_number)
      {
        case DATA_PIPE_0:
        break;        
        case DATA_PIPE_1:
        break;        
        case DATA_PIPE_2:
        // se manda: roll, pitch, empuje
          // read payload
          my_nrf.read_packet(&payload_rc, sizeof(payload_rc));
          // receiving a two byte struct payload on DATA_PIPE_2
          printf("\n%s ,", coor);
          printf("%f, %f, %f\n", payload_rc.roll,payload_rc.pitch,payload_rc.empuje);
          
        break;        
        case DATA_PIPE_3:
        break;        
        case DATA_PIPE_4:
        break;        
        case DATA_PIPE_5:
        break;        
        default:
        break;
      }
    }

    // Lectura de ultrasonido
    // ultra_read(altura); // falta sensar y definir variables
    // float sensor_value_empuje = altura; 

    // Lectura de MPU
    mpu6050_read(acceleration, gyro);
    payload_mpu_t payload_mpu =
      {
        .accel_x = acceleration[0],
        .accel_y = acceleration[1],
        .accel_z = acceleration[2],
        .gyro_x = gyro[0],
        .gyro_y = gyro[1],
        .gyro_z = gyro[2],
      };    
    
    float sensor_value_roll = payload_mpu.gyro_x;
    float sensor_value_pitch = payload_mpu.gyro_y;

    // Estimación del estado actual para calcular setpoint
    // Se debe mapear de valor analogico a ángulo, en las funciones están las constantes por si cambian
    float setpoint_pitch = calculateSetPoint_pitch(payload_rc.pitch);
    float setpoint_roll = calculateSetPoint_roll(payload_rc.roll);
    // este entiendo que está de 0 - 255 y por esto no le hice map 
    float setpoint_altura = payload_rc.empuje;

    // Aplica el controlador PID a cada variable
    float control_pitch = pid_controller_pitch(sensor_value_pitch,setpoint_pitch);
    float control_roll = pid_controller_roll(sensor_value_roll,setpoint_roll);
    // float a = pid_controller_altitud(sensor_value_empuje,setpoint_altura);

    // Mapear salida de cada variable a PWM
    float M1angle = - control_pitch + control_roll;
    float M2angle = - control_pitch - control_roll;
    float M3angle = + control_pitch + control_roll;
    float M4angle = + control_pitch - control_roll;

    // Mapear los valores de angulo a PWM
    int M1 = (int)(M1angle / angleMax * PWMangle);
    int M2 = (int)(M2angle / angleMax * PWMangle);
    int M3 = (int)(M3angle / angleMax * PWMangle);
    int M4 = (int)(M4angle / angleMax * PWMangle);

    M1 = setpoint_altura + M1;
    M2 = setpoint_altura + M2;
    M3 = setpoint_altura + M3;
    M4 = setpoint_altura + M4;

    // Aplicar otro mapeo a los valores de los motores sumándoles setpoint_altura 
    // y mapeando al rango de 100 a 255
    M1 = constrain(M1, 100, 255);
    M2 = constrain(M2, 100, 255);
    M3 = constrain(M3, 100, 255);
    M4 = constrain(M4, 100, 255);

    // Actualizar PWM en motores
    pwm_set_chan_level(slice_num1, PWM_CHAN_A, M1);
    pwm_set_chan_level(slice_num2, PWM_CHAN_A, M2);
    pwm_set_chan_level(slice_num3, PWM_CHAN_A, M3);
    pwm_set_chan_level(slice_num4, PWM_CHAN_A, M4);

    // Aquí ajustar el retardo de tiempo según necesidades
    // Esto determina la frecuencia de muestreo del controlador
    // Tener en cuenta la velocidad de transmisión de los datos 
    sleep_ms(0.1);
  }
  
  return 0;
}
