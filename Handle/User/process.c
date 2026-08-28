#include "process.h"

static uint16_t adc_dma_buffer[ADC_CHANNELS];

static KeyStatus key_status[10];
static uint32_t key_press_tick[10];

//将0-4095adc值映射到正负最大平动速度间
static float ADC_MapSpeed(uint16_t adc_value, float max_speed)
{
    int32_t offset;
    float result;

    offset = (int32_t)adc_value -
             (int32_t)ADC_CENTER_VALUE +
						 (int32_t)ADC_CENTER_VALUE_SPEED;

    /* 摇杆中心死区 */
    if ((offset >= -(int32_t)ADC_DEAD_ZONE) &&
        (offset <= (int32_t)ADC_DEAD_ZONE))
        result = 0.0f;

    if (offset > 0)
        result = ((float)offset / 2047.0f) *
                 max_speed;
    else if(offset < 0)
        result = ((float)offset / 2048.0f) *
                 max_speed;

    return (result);
}

//根据旋钮方向输出底盘旋转定速值
static float ADC_MapOmega(uint16_t adc_value)
{
    int32_t offset;
    float result;

    offset = (int32_t)adc_value -
             (int32_t)ADC_OMEGA_ADC_OFFSET -
             (int32_t)ADC_CENTER_VALUE;

    if ((offset >= -(int32_t)ADC_DEAD_ZONE) &&
        (offset <= (int32_t)ADC_DEAD_ZONE))
    {
        result = 0.0f;
    }
    else if (offset < 0)
    {
        result = -ADC_OMEGA;
    }
    else if (offset > 0)
    {
        result = ADC_OMEGA;
    }
		
    return result;
}

static uint8_t key_scan(HandleKey_t key, GPIO_TypeDef *key_port, uint16_t key_pin)
{
    uint8_t index;
    uint8_t pressed;
    uint32_t now;

    index = (uint8_t)key;

    /* 按键低电平有效 */
    pressed = (HAL_GPIO_ReadPin(key_port, key_pin) == GPIO_PIN_RESET) ? 1U : 0U;

    now = HAL_GetTick();

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

            if ((now - key_press_tick[index]) >= KEY_PRESS_THRESHOLD)
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
    HAL_ADC_Start_DMA(ADC_ADDRESS, (uint32_t*)adc_dma_buffer, ADC_CHANNELS);

    //串口收发相关均在zigbee文件中
}

//全部指令处理
void HandleOrderProcess(SendData_t *sdata)
{
    uint8_t key_forward;
    uint8_t key_backward;
    uint8_t key_lift;
    uint8_t key_down;
    uint8_t key_positive_flip;
    uint8_t key_negative_flip;

    sdata->ChassisData.chassis_vy = ADC_MapSpeed(adc_dma_buffer[ADC_SPEED_VX_INDEX], ADC_MAX_SPEED_VX) + 0.1f - 0.002f;
		if(sdata->ChassisData.chassis_vy < 0.05f && sdata->ChassisData.chassis_vy > -0.05f) sdata->ChassisData.chassis_vx = 0.0f;
	
    sdata->ChassisData.chassis_vx = ADC_MapSpeed(adc_dma_buffer[ADC_SPEED_VY_INDEX], ADC_MAX_SPEED_VY) - 0.017f;
		if(sdata->ChassisData.chassis_vx < 0.03f && sdata->ChassisData.chassis_vx > -0.03f) sdata->ChassisData.chassis_vy = 0.0f;
	
    sdata->ChassisData.chassis_omega = ADC_MapOmega(adc_dma_buffer[ADC_OMEGA_INDEX]);

    sdata->ArmData.emergency_stop = key_scan(KEY_STOP, KEY_STOP_PORT, KEY_STOP_PIN);
    sdata->ArmData.mode = key_scan(KEY_MODE, KEY_MODE_PORT, KEY_MODE_PIN);
    sdata->ArmData.arm_grip = key_scan(KEY_GRIP, KEY_GRIP_PORT, KEY_GRIP_PIN);
    key_forward = key_scan(KEY_FORWARD, KEY_FORWARD_PORT, KEY_FORWARD_PIN);
    key_backward = key_scan(KEY_BACKWARD, KEY_BACKWARD_PORT, KEY_BACKWARD_PIN);
    key_lift = key_scan(KEY_LIFT, KEY_LIFT_PORT, KEY_LIFT_PIN);
    key_down = key_scan(KEY_DOWN, KEY_DOWN_PORT, KEY_DOWN_PIN);
    key_positive_flip = key_scan(KEY_POSITIVE_FLIP, KEY_POS_FLIP_PORT, KEY_POS_FLIP_PIN);
    key_negative_flip = key_scan(KEY_NEGATIVE_FLIP, KEY_NEG_FLIP_PORT, KEY_NEG_FLIP_PIN);

    sdata->ArmData.joint_front_back = key_forward ? JOINT_SPEED_FORWARD : (key_backward ? JOINT_SPEED_BACKWARD : 0.0f);
    sdata->ArmData.joint_up_down = key_lift ? JOINT_SPEED_LIFT : (key_down ? JOINT_SPEED_DOWN : 0.0f);
    sdata->ArmData.joint_flip = key_positive_flip ? JOINT_SPEED_FLIP_POS : (key_negative_flip ? JOINT_SPEED_FLIP_NEG : 0.0f);

    if (sdata->ArmData.emergency_stop != 0U)
    {
        sdata->ChassisData.chassis_vx = 0.0f;
        sdata->ChassisData.chassis_vy = 0.0f;
        sdata->ChassisData.chassis_omega = 0.0f;

        sdata->ArmData.mode = 1U;
        sdata->ArmData.arm_grip = 0U;
        sdata->ArmData.joint_front_back = 0.0f;
        sdata->ArmData.joint_up_down = 0.0f;
        sdata->ArmData.joint_flip = 0.0f;
    }
}
