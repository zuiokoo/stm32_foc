#include "as5600.h"

#define AS5600_I2C_ADDRESS    (0x36 << 1)
#define AS5600_ANGLE_REGISTER 0x0E


void as5600_init(as5600_t * dev,I2C_HandleTypeDef *hi2c )
{
    dev->address=AS5600_I2C_ADDRESS;
    dev->hi2c=hi2c;
}

HAL_StatusTypeDef  as5600_read_raw(as5600_t * dev,uint16_t *raw_angle)
{

    HAL_StatusTypeDef status;
	uint8_t angle_data[2] = {0};
	status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, AS5600_ANGLE_REGISTER, I2C_MEMADD_SIZE_8BIT, angle_data, 2,10);
    if (status != HAL_OK)
    {
        return status;
    }
	*raw_angle = ((uint16_t)angle_data[0] << 8 | angle_data[1]) & 0x0fff;
	return HAL_OK;
}
HAL_StatusTypeDef as5600_read_mechanical_angle_rad(as5600_t *dev,float *mechanical_angle_rad)
{
    uint16_t raw_angle;
    HAL_StatusTypeDef status;
    status = as5600_read_raw(dev,&raw_angle);
    if (status != HAL_OK)
    {
        return status;
    }
    *mechanical_angle_rad  = (float)raw_angle * 2.0f * 3.14159265358979323846f / 4096.0f;
    
    return HAL_OK;
}

/* ---- I2C 总线自愈 ----
 * 触发场景：读取返回 HAL_BUSY。F4 的 HAL 只有在等 I2C_FLAG_BUSY 清零超时(25ms)时才这么返回，
 * 也就是 SDA 或 SCL 被拉住。分两种情况，处理方式完全不同：
 *   A) SDA 被拉住、SCL 自由 —— 从机停在半个字节里继续驱动 SDA：
 *      给 SCL 打 9 个时钟，让它把剩余位吐完并释放 SDA，再补一个 STOP。
 *   B) SCL 被拉住 —— 握线的是从机(AS5600)或硬件短路：9 个时钟无效
 *      （强行驱动只会和从机对着拉），软件救不回来，只能给从机断电重启。
 * 所以先释放两条线看实际电平，再决定走哪条路。
 * 引脚按 CubeMX 配置写死：PB6 = I2C1_SCL，PB7 = I2C1_SDA（AF4）。
 * 返回：1 = 按 A) 处理过；0 = 检测到 SCL 被拉住（需要断电重启从机）。 */

/* 恢复时钟的半周期。故意取得很慢（1ms → 约 500Hz）：
 * 即使总线只有很弱的上拉（甚至只有 MCU 内部约 40k 上拉），电平也能在半个周期内
 * 充到高电平，从机一定认得到这 9 个时钟。整个恢复约 18ms，只在故障时执行，
 * 且 ISR 不受影响（角度暂停更新十几 ms 无所谓）。 */
static void as5600_delay_halfbit(void)
{
    HAL_Delay(1);
}

/* 把 PB6/PB7 恢复成 I2C 复用并重新初始化外设 */
static void as5600_restore_i2c(as5600_t *dev)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    dev->hi2c->Lock  = HAL_UNLOCKED;
    dev->hi2c->State = HAL_I2C_STATE_RESET;
    HAL_I2C_Init(dev->hi2c);
}

uint8_t as5600_bus_recover(as5600_t *dev)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;
    uint8_t scl_lvl;

    /* 1) 关 I2C，收脚回来当普通开漏 IO，两条线都释放为高 */
    __HAL_I2C_DISABLE(dev->hi2c);
    dev->hi2c->Lock  = HAL_UNLOCKED;              /* 清掉可能残留的锁 */
    dev->hi2c->State = HAL_I2C_STATE_RESET;

    /* 外设整体复位：清掉卡在 BUSY 的内部状态（MCU 侧那一半成因） */
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    gpio.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);   /* 释放成高 */

    /* 2) 看谁在拉低（此时外设已关，读到的是真实线电平） */
    as5600_delay_halfbit();
    scl_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1U : 0U;

    if (scl_lvl == 0U)
    {
        /* SCL 被拉住：软件救不了，恢复 I2C 后返回 0，由上层提示断电 */
        as5600_restore_i2c(dev);
        return 0;
    }

    /* 3) SDA 被拉住、SCL 自由：打 9 个时钟把从机顶出半个字节 */
    for (i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        as5600_delay_halfbit();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        as5600_delay_halfbit();
    }

    /* 4) 补 STOP：SCL 高时 SDA 低 -> 高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    as5600_delay_halfbit();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    as5600_delay_halfbit();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    as5600_delay_halfbit();

    /* 5) 恢复复用功能并重新初始化 I2C */
    as5600_restore_i2c(dev);
    return 1;
}

