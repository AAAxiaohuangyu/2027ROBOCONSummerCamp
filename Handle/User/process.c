#include "process.h"

static uint32_t adc_dma_buffer[ADC_CHANNELS];

static KeyStatus key_status[10];
static uint32_t key_press_tick[10];

//将0-4095adc值映射到正负最大平动速度间
static int32_t ADC_MapSpeed(uint32_t adc_value, float max_speed)
{
    int32_t offset;
    float result;

    offset = (int32_t)adc_value -
             (int32_t)ADC_CENTER_VALUE;

    /* 摇杆中心死区 */
    if ((offset >= -(int32_t)ADC_DEAD_ZONE) &&
        (offset <= (int32_t)ADC_DEAD_ZONE))
        return 0;

    if (offset > 0)
        result = ((float)offset / 2047.0f) *
                 max_speed;
    else if(offset < 0)
        result = ((float)offset / 2048.0f) *
                 max_speed;

    return (int32_t)result;
}

//底盘定速旋转
static int32_t ADC_MapOmega(uint32_t adc_value)
{
    int32_t offset;
    float result;

    offset = (int32_t)adc_value -
             (int32_t)ADC_CENTER_VALUE;

    if ((offset >= -(int32_t)ADC_DEAD_ZONE) &&
        (offset <= (int32_t)ADC_DEAD_ZONE))
        return 0;

    if (offset < 0)
        result = -(int32_t)ADC_OMEGA;

    else if (offset > 0)
        result = (int32_t)ADC_OMEGA;

    return (int32_t)result;
}

static uint8_t key_scan(HandleKey_t key, GPIO_TypeDef *key_port, uint16_t key_pin)
{
    uint8_t index;
    uint8_t pressed;
    uint32_t now;
    uint32_t confirm_time_ms;

    index = (uint8_t)key;

    /* 按键低电平有效 */
    pressed = (HAL_GPIO_ReadPin(key_port, key_pin) == GPIO_PIN_RESET) ? 1U : 0U;

    now = HAL_GetTick();

    /* 头文件中的单位为秒，转换成毫秒 */
    confirm_time_ms = (uint32_t)(KEY_PRESS_THRESHOLD * 1000.0f);

    switch (key_status[index])
    {
        case KEY_IDLE:
        {
            if (pressed != 0U)
            {
                key_press_tick[index] = now;
                key_status[index] = KEY_TEMP_PRESSED;
            }
            break;
        }

        case KEY_TEMP_PRESSED:
        {
            if (pressed == 0U)
            {
                key_status[index] = KEY_IDLE;
                break;
            }

            if ((now - key_press_tick[index]) >= confirm_time_ms)
                key_status[index] = KEY_PRESSED;
            
            break;
        }

        case KEY_PRESSED:
        {
            if (pressed == 0U)
                key_status[index] = KEY_IDLE;

            break;
        }

        default:
        {
            key_status[index] = KEY_IDLE;
            break;
        }
    }

    return (key_status[index] == KEY_PRESSED) ? 1U : 0U;
}

void Handle_Init(void)
{
    memset(adc_dma_buffer, 0, sizeof(adc_dma_buffer));
    memset(key_status, 0, sizeof(key_status));
    memset(key_press_tick, 0, sizeof(key_press_tick));

    HAL_ADCEx_Calibration_Start(ADC_ADDRESS);
    HAL_ADC_Start_DMA(ADC_ADDRESS, adc_dma_buffer, ADC_CHANNELS);

    //串口收发相关均在zigbee文件中
}

//全部指令处理
void HandleOrderProcess(SendData_t *sdata)
{
    sdata->ChassisData.chassis_vx = ADC_MapSpeed(adc_dma_buffer[ADC_SPEED_VX_INDEX], ADC_MAX_SPEED_VX);
    sdata->ChassisData.chassis_vy = ADC_MapSpeed(adc_dma_buffer[ADC_SPEED_VY_INDEX], ADC_MAX_SPEED_VY);
    sdata->ChassisData.chassis_omega = ADC_MapOmega(adc_dma_buffer[ADC_OMEGA_INDEX]);

    sdata->ArmData.mode_switch = key_scan(KEY_MODE, KEY_MODE_PORT, KEY_MODE_PIN);
    sdata->ArmData.emergency_stop = key_scan(KEY_STOP, KEY_STOP_PORT, KEY_STOP_PIN);
    sdata->ArmData.arm_grip = key_scan(KEY_GRIP, KEY_GRIP_PORT, KEY_GRIP_PIN);
    //sdata->ArmData.arm_release = key_scan(KEY_RELEASE, KEY_RELEASE_PORT, KEY_RELEASE_PIN);
    sdata->ArmData.joint_forward = key_scan(KEY_FORWARD, KEY_FORWARD_PORT, KEY_FORWARD_PIN);
    sdata->ArmData.joint_backward = key_scan(KEY_BACKWARD, KEY_BACKWARD_PORT, KEY_BACKWARD_PIN);
    sdata->ArmData.joint_lift = key_scan(KEY_LIFT, KEY_LIFT_PORT, KEY_LIFT_PIN);
    sdata->ArmData.joint_down = key_scan(KEY_DOWN, KEY_DOWN_PORT, KEY_DOWN_PIN);
    sdata->ArmData.joint_positive_flip = key_scan(KEY_POSITIVE_FLIP, KEY_POS_FLIP_PORT, KEY_POS_FLIP_PIN);
    sdata->ArmData.joint_negative_flip = key_scan(KEY_NEGATIVE_FLIP, KEY_NEG_FLIP_PORT, KEY_NEG_FLIP_PIN);
}
