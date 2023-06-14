#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <math.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <gmp.h>
#include <mpfr.h>

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

class BMP3 {
	public:
		int addr;
		int init(int addr);
		void read();
	//private:
		void calib();
		mpfr_t T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
};

int BMP3::init(int addr) {
	i2c_init();
	this->addr = addr;
	u8 chipid;
	i2c_read(this->addr, 0x00, &chipid);

	if(chipid == 0x60) fprintf(stderr, "Chip is BMP390\n");
	else if(chipid == 0x50) fprintf(stderr, "Chip is BMP388\n");
	else {
		fprintf(stderr, "who even are you?!\n");
		return -1;
	}
	mpfr_init2(T1, 128);
	mpfr_init2(T2, 128);
	mpfr_init2(T3, 128);
	mpfr_init2(P1, 128);
	mpfr_init2(P2, 128);
	mpfr_init2(P3, 128);
	mpfr_init2(P4, 128);
	mpfr_init2(P5, 128);
	mpfr_init2(P6, 128);
	mpfr_init2(P7, 128);
	mpfr_init2(P8, 128);
	mpfr_init2(P9, 128);
	mpfr_init2(P10, 128);
	mpfr_init2(P11, 128);
	calib();
	return 0;
}

void BMP3::calib() {
	u8 cal_data[21];
	for(int i = 0; i < 21; i++) i2c_read(this->addr, 0x31+i, &cal_data[i]);
	
	unsigned short	T1 = cal_data[1] << 8 | cal_data[0];
	unsigned short	T2 = cal_data[3] << 8 | cal_data[2];
	char			T3 = cal_data[4];

	mpfr_set_ui(this->T1, T1, MPFR_RNDD);
	mpfr_div_d(this->T1, this->T1, pow(2, -8), MPFR_RNDD);
	mpfr_set_ui(this->T2, T2, MPFR_RNDD);
	mpfr_div_d(this->T2, this->T2, pow(2, 30), MPFR_RNDD);
	mpfr_set_si(this->T3, T3, MPFR_RNDD);
	mpfr_div_d(this->T3, this->T3, pow(2, 48), MPFR_RNDD);
}

void BMP3::read() {
	u8 status = 0;
	i2c_write(this->addr, 0x1B, 0x13);
	i2c_write(this->addr, 0x1B, 0x13);
	while((status & 0x60) != 0x60) {
		i2c_read(this->addr, 0x03, &status);
	}
	
	u8 data[6];
	for(int i = 0; i < 6; i++) i2c_read(this->addr, 0x04+i, &data[i]);
	
	int		adc_t = data[5] << 16 | data[4] << 8 | data[3],
			adc_p = data[2] << 16 | data[1] << 8 | data[0];
	
	mpfr_t pd1, pd2, temp_test;
	mpfr_init2(pd1, 128);
	mpfr_init2(pd2, 128);
	mpfr_init2(temp_test, 128);
	mpfr_set_si(pd1, adc_t, MPFR_RNDD);
	mpfr_sub(pd1, pd1, this->T1, MPFR_RNDD);
	mpfr_set(pd2, pd1, MPFR_RNDD);
	mpfr_mul(pd2, pd2, this->T2, MPFR_RNDD);
	mpfr_set(temp_test, pd1, MPFR_RNDD);
	mpfr_mul(temp_test, temp_test, pd1, MPFR_RNDD);
	mpfr_mul(temp_test, temp_test, this->T3, MPFR_RNDD);
	mpfr_add(temp_test, temp_test, pd2, MPFR_RNDD);

	mpfr_out_str(stdout, 10, 0, temp_test, MPFR_RNDD);
//	temp = pd2 + (pd1 * pd1) * this->T3;i
}

int main() {
	BMP3 sense;
	sense.init(BMP_ADDR);
	float temp = 0;
	sense.read();
//	printf("%f\n", temp);
//	printf("%f %f %f\n", sense.T1, sense.T2, sense.T3);
}
