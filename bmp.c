#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <math.h>
#include <unistd.h>

int i2c_fd = -1;
const char *i2c_fname = "/dev/i2c-1";

typedef unsigned char u8;

int i2c_init() {
	if ((i2c_fd = open(i2c_fname, O_RDWR)) < 0) {
		char err[200];
		sprintf(err, "open('%s') in i2c_init", i2c_fname);
		perror(err);
		return -1;
	}
	return i2c_fd;
}

void i2c_close() {
	close(i2c_fd);
}

int i2c_write(u8 slave_addr, u8 reg, u8 data) {
    int retval;
    u8 outbuf[2];

    struct i2c_msg msgs[1];
    struct i2c_rdwr_ioctl_data msgset[1];

    outbuf[0] = reg;
    outbuf[1] = data;

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = outbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_write");
        return -1;
    }

    return 0;
}

int i2c_read(u8 slave_addr, u8 reg, u8 *result) {
    int retval;
    u8 outbuf[1], inbuf[1];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];

    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = slave_addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = 1;
    msgs[1].buf = inbuf;

    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    outbuf[0] = reg;

    inbuf[0] = 0;

    *result = 0;
    if (ioctl(i2c_fd, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return -1;
    }

    *result = inbuf[0];
    return 0;
}

#define BMP_ADDR 0x76
#define SLP 1013.25

float temp_calib[3], pres_calib[11];

void bmp_read(float *temperature, float *pressure) {
	u8 status = 0;
	i2c_write(BMP_ADDR, 0x1B, 0x13);
	i2c_write(BMP_ADDR, 0x1B, 0x13);
	while((status & 0x60) != 0x60) {
		i2c_read(BMP_ADDR, 0x03, &status);
	}

	u8 data[6];
	for(int i = 0; i < 6; i++) i2c_read(BMP_ADDR, 0x04+i, &data[i]);
	int		adc_p = data[2] << 16 | data[1] << 8 | data[0],
			adc_t = data[5] << 16 | data[4] << 8 | data[3];
	
	float	pd1 = adc_t - temp_calib[0],
			pd2 = pd1 * temp_calib[1];
	float	temp = pd2 + (pd1 * pd1) * temp_calib[2];

			pd1 = pres_calib[5] * temp,
			pd2 = pres_calib[6] * temp * temp;
	float	pd3 = pres_calib[7] * temp * temp * temp,
			po1 = pres_calib[4] + pd1 + pd2 + pd3;

			pd1 = pres_calib[1] * temp,
			pd2 = pres_calib[2] * temp * temp,
			pd3 = pres_calib[3] * temp * temp * temp;
	float	po2 = adc_p * (pres_calib[0] + pd1 + pd2 + pd3);

			pd1 = adc_p * adc_p;
			pd2 = pres_calib[8] + pres_calib[9] * temp;
			pd3 = pd1 * pd2;
	float	pd4 = pd3 + pres_calib[10] * adc_p * adc_p * adc_p;

	*temperature = temp, *pressure = po1 + po2 + pd4;
}

int main() {
	i2c_init();
	
	u8 chipid;
	i2c_read(BMP_ADDR, 0x00, &chipid);

	if(chipid == 0x60) fprintf(stderr, "Chip is BMP390\n");
	else if(chipid == 0x50) fprintf(stderr, "Chip is BMP388\n");
	else {
		fprintf(stderr, "who even are you?!\n");
		return -1;
	}

	u8 cal_data[21];
	for(int i = 0; i < 21; i++) i2c_read(BMP_ADDR, 0x31+i, &cal_data[i]);
	
	unsigned short	T1 = cal_data[1] << 8 | cal_data[0];
	unsigned short	T2 = cal_data[3] << 8 | cal_data[2];
	char			T3 = cal_data[4];
	short 			P1 = cal_data[6] << 8 | cal_data[5];
	short			P2 = cal_data[8] << 8 | cal_data[7];
	char			P3 = cal_data[9];
	char			P4 = cal_data[10];
	unsigned short	P5 = cal_data[12] << 8 | cal_data[11];
	unsigned short	P6 = cal_data[14] << 8 | cal_data[13];
	char			P7 = cal_data[15];
	char			P8 = cal_data[16];
	short			P9 = cal_data[18] << 8 | cal_data[17];
	char			P10 = cal_data[19];
	char			P11 = cal_data[20];
	
	temp_calib[0] = T1 / pow(2, -8);
	temp_calib[1] = T2 / pow(2, 30);
	temp_calib[2] = T3 / pow(2, 48);

	pres_calib[0] = (P1 - pow(2, 14)) / pow(2, 20);
	pres_calib[1] = (P2 - pow(2, 14)) / pow(2, 29);
	pres_calib[2] = P3 / pow(2, 32);
	pres_calib[3] = P4 / pow(2, 37);
	pres_calib[4] = P5 / pow(2, -3);
	pres_calib[5] = P6 / pow(2, 6);
	pres_calib[6] = P7 / pow(2, 8);
	pres_calib[7] = P8 / pow(2, 15);
	pres_calib[8] = P9 / pow(2, 48);
	pres_calib[9] = P10 / pow(2, 48);
	pres_calib[10] = P11 / pow(2, 65);

	i2c_write(BMP_ADDR, 0x7E, 0xB6);

	float temp, pres;

	bmp_read(&temp, &pres);

	printf("%f %f\n", temp, pres);
	printf("%f\n", 44307.7 * (1 - pow(pres / 100 / SLP, 0.190284)));
}
