#include <machine/patmos.h>
#include <stdio.h>
#include <string.h>
#include <math.h>


#define AUTO_TAKEOFF  1
#define ALT_HOLD      2
#define AUTO_THROTTLE 3
#define CLIMB         4

typedef unsigned char BYTE;

_iodev_ptr_t uart1_ptr = (_iodev_ptr_t) 0xF0080000;
_iodev_ptr_t uart2_ptr = (_iodev_ptr_t) 0xF00E0000;
_iodev_ptr_t uart3_ptr = (_iodev_ptr_t) 0xF00F0000;

BYTE out_elev_H, out_elev_L, out_ail_H, out_ail_L, out_rud_H, out_rud_L, out_thr_H, out_thr_L;

int mode = AUTO_TAKEOFF;


float pitch = 0;
float roll = 0;
float heading = 0;
float dt = 0.05;
float longitude, latitude;

// Data Fusion parameters 

float acc_x = 0;
float acc_y = 0;
float acc_z = 0;
float P = 0;
float Q = 0;

float ias_sen = 0;
float p_ias_sen = 0;
float alt_ft_msl = 0;
float alt_ft_agl = 0; 

float theta_cal = 0.0;
float phi_cal = 0.0; 

float P0_1_1 = 0;
float P0_2_1 = 0;
float P0_1_2 = 0;
float P0_2_2 = 0;
float P1_1_1 = 0;
float P1_2_1 = 0;
float P1_1_2 = 0;
float P1_2_2 = 0;

float gyro_noise = 0.001;
float gyro_bias = 0.03;
float Sensor_accuracy = 0.03;

float Kalman_gain_0_1 = 0;
float Kalman_gain_0_2 = 0;
float Kalman_gain_1_1 = 0;
float Kalman_gain_1_2 = 0;

float bias_1 = 0;
float bias_2 = 0;

float c99max (float a, float lowest){
	if (a< lowest){
		a = lowest;
	}
	return a; 
}


void Estimator(float ax, float ay, float az, float p, float q){

    ax = (ax - (ias_sen - p_ias_sen)/dt);
	p_ias_sen = ias_sen;

	float theta_acc = (float)(atan2(ax, sqrt(az*az + ay*ay))*57.324);
	float phi_acc = (float)(atan2(ay,az)*57.324);
	
	float temp1 = p - bias_1;
    theta_cal += dt * temp1;

	float temp2 = q- bias_2;
	phi_cal += dt * temp2;

	P0_1_1 += dt * (dt* P0_2_2 - P0_1_2 - P0_2_1 + gyro_noise);
	P0_1_2 -= dt * P0_2_2;
	P0_2_1 -= dt * P0_2_2;
	P0_2_2 += gyro_bias * dt;

	P1_1_1 += dt * (dt* P1_2_2 - P1_1_2 - P1_2_1 + gyro_noise);
	P1_1_2 -= dt * P1_2_2;
	P1_2_1 -= dt * P1_2_2;
	P1_2_2 += gyro_bias * dt;

	float temp3 = c99max(P0_1_1 + Sensor_accuracy, 0.00001);
	float temp4 = c99max(P1_1_1 + Sensor_accuracy, 0.00001);

	Kalman_gain_0_1 = c99max(P0_1_1 / temp3, .000001);
	Kalman_gain_0_2 = c99max(P0_1_2 / temp3, .000001);
	Kalman_gain_1_1 = c99max(P1_1_1 / temp4, .000001);
	Kalman_gain_1_2 = c99max(P1_1_2 / temp4, .000001);

	theta_cal += Kalman_gain_0_1 * (theta_acc - theta_cal);
	phi_cal += Kalman_gain_1_1 * (phi_acc - phi_cal);
	bias_1 += Kalman_gain_0_2 * (theta_acc - theta_cal);	
	bias_2 += Kalman_gain_1_2 * (phi_acc - phi_cal);

	P0_1_1 -= Kalman_gain_0_1 * P0_1_1;
    P0_1_2 -= Kalman_gain_0_1 * P0_1_2;
    P0_2_1 -= Kalman_gain_0_2 * P0_2_1;
    P0_2_2 -= Kalman_gain_0_2 * P0_2_2;
	P1_1_1 -= Kalman_gain_1_1 * P1_1_1;
    P1_1_2 -= Kalman_gain_1_1 * P1_1_2;
    P1_2_1 -= Kalman_gain_1_2 * P1_2_1;
    P1_2_2 -= Kalman_gain_1_2 * P1_2_2;

}


int receiving = 0;
int bit_1 = 0;
int bit_2 = 0;
int bit_3 = 0;

int elev, ail, rudder, throttle, n_wheel; 

int all_received = 0;

int total_bytes = 52;

int i = 0;
unsigned char raw_bytes_from_sim[52];

float p_control_accu_err = 0;
float p_control_d_err = 0;
float cruise_control_accu_err = 0;
float cruise_control_d_err = 0; 


float pitch_control_prop_gain = 2;
float pitch_control_diff_gain = 2;
float pitch_control_int_gain = 0.001;

float cruise_control_prop_gain = 0.020;
float cruise_control_diff_gain = 0.02;
float cruise_control_int_gain = 0.0001;


void longitudinal_control(int d_pitch) {
    rudder = 0;
	float err = d_pitch - pitch;
	p_control_accu_err += err;
	float der = err - p_control_d_err;
	p_control_d_err = err;
	elev = (int) (err * pitch_control_prop_gain + (p_control_accu_err *pitch_control_int_gain) + der * pitch_control_diff_gain);
	//printf("err: %f, derr: %f, ierr: %f, elev: %d \n", err, der, p_control_accu_err, elev );

}

float min_max(float max, float min, float val){
	if (val > max){
		val = max;
	}
	else if(val < min){
		val = min;
	}
	
	return val;
}

void cruise_control(int altitude) {
    rudder = 0;
	throttle = 60;
	float err = altitude - alt_ft_msl;
	cruise_control_accu_err += err;
	float der = err - cruise_control_d_err;

	longitudinal_control(min_max(15,-15,(err * cruise_control_prop_gain )));
	
}

void delay(int a) {

	for (int i = 0; i < a * 10000; i++) {
	
	}
}


void inline write(_iodev_ptr_t uart_base_ptr, unsigned char c) {
  while ((*uart_base_ptr & 0x01) == 0);
  *(uart_base_ptr+1) = c;
}

char inline read(_iodev_ptr_t uart_base_ptr) {
  while ((*uart_base_ptr & 0x02) == 0);
  return *(uart_base_ptr+1);
}

int safe_byte(float a) {
	int temp = (int) a; 
	temp = temp + 100;

	if (temp > 200) {
		temp = 200;
	}
	else if (temp < 0) {
		temp = 0;
	}

	return temp;
}

float roll_p = 1;
float roll_d = 30;
float roll_i = 0.0001;
float r_control_accu_err = 0.0;
float r_control_d_err = 0.0;


void lateral_control(float d_roll) {

    float err = d_roll - roll;
	r_control_accu_err += err;
	float der = err - r_control_d_err;
	r_control_d_err = err;
	ail = (int) (err * roll_p + (r_control_accu_err *roll_i) + der * roll_d);

}

void HDG_control (float d_hdg){
 float err = d_hdg - heading;
   if(err >= 180){
	   lateral_control(min_max(10,-10,-err*1));
   }
   else{
	   lateral_control(min_max(10,-10,err*1));
   }
}


int main() {
    printf("Patmos Started!!");


	for(;;){

		unsigned char serial =  read(uart2_ptr);

		if (receiving == 1) {
			i += 1;
			
		}
		if (serial == 255 && receiving == 0) {
			bit_1 = 1;
		}
		else if (serial == 254 && receiving == 0 && bit_1 == 1){
			bit_2 = 1;
		}
		else if (serial == 253 && receiving == 0 && bit_2 == 1) {
			bit_3 = 1;
		}
		else if (serial == 252 && receiving == 0 && bit_3 == 1) {
			receiving = 1;
			bit_1 = 0;
			bit_2 = 0;
			bit_3 = 0;
		}
		else {
			bit_1 = 0;
			bit_2 = 0;
			bit_3 = 0;
		}

		raw_bytes_from_sim[i] = serial;

		if (i == total_bytes) {
				receiving = 0;
				i = 0;
				bit_1 = 0;
			    bit_2 = 0;
			    bit_3 = 0;
				all_received = 1;
		}


        if(all_received == 1){
		unsigned char b[] = { raw_bytes_from_sim[4], raw_bytes_from_sim[3],raw_bytes_from_sim[2],raw_bytes_from_sim[1] };
		memcpy(&pitch, &b, sizeof(pitch));

		unsigned char c[] = { raw_bytes_from_sim[8], raw_bytes_from_sim[7],raw_bytes_from_sim[6],raw_bytes_from_sim[5] };
		memcpy(&roll, &c, sizeof(roll));

		unsigned char d[] = { raw_bytes_from_sim[12], raw_bytes_from_sim[11],raw_bytes_from_sim[10],raw_bytes_from_sim[9] };
		memcpy(&heading, &d, sizeof(heading));

		unsigned char e[] = { raw_bytes_from_sim[16], raw_bytes_from_sim[15],raw_bytes_from_sim[14],raw_bytes_from_sim[13] };
		memcpy(&ias_sen, &e, sizeof(ias_sen));

		unsigned char f[] = { raw_bytes_from_sim[20], raw_bytes_from_sim[19],raw_bytes_from_sim[18],raw_bytes_from_sim[17] };
		memcpy(&alt_ft_msl, &f, sizeof(alt_ft_msl));

		unsigned char g[] = { raw_bytes_from_sim[24], raw_bytes_from_sim[23],raw_bytes_from_sim[22],raw_bytes_from_sim[21] };
		memcpy(&alt_ft_agl, &g, sizeof(alt_ft_agl));

		unsigned char h[] = { raw_bytes_from_sim[28], raw_bytes_from_sim[27],raw_bytes_from_sim[26],raw_bytes_from_sim[25] };
		memcpy(&acc_x, &h, sizeof(acc_x));

		
		unsigned char i[] = { raw_bytes_from_sim[32], raw_bytes_from_sim[31],raw_bytes_from_sim[30],raw_bytes_from_sim[29] };
		memcpy(&acc_y, &i, sizeof(acc_y));

		unsigned char j[] = { raw_bytes_from_sim[36], raw_bytes_from_sim[35],raw_bytes_from_sim[34],raw_bytes_from_sim[33] };
		memcpy(&acc_z, &j, sizeof(acc_z));

		unsigned char k[] = { raw_bytes_from_sim[40], raw_bytes_from_sim[39],raw_bytes_from_sim[38],raw_bytes_from_sim[37] };
		memcpy(&P, &k, sizeof(P));

		unsigned char l[] = { raw_bytes_from_sim[44], raw_bytes_from_sim[43],raw_bytes_from_sim[42],raw_bytes_from_sim[41] };
		memcpy(&Q, &l, sizeof(Q));

		unsigned char m[] = { raw_bytes_from_sim[48], raw_bytes_from_sim[47],raw_bytes_from_sim[46],raw_bytes_from_sim[45] };
		memcpy(&latitude, &m, sizeof(latitude));

		unsigned char n[] = { raw_bytes_from_sim[52], raw_bytes_from_sim[51],raw_bytes_from_sim[50],raw_bytes_from_sim[49] };
		memcpy(&longitude, &n, sizeof(longitude));

		

        Estimator( acc_x,  acc_y,  acc_z,  P, Q);




		/*pitch_control_prop_gain = raw_bytes_from_sim[25];
		pitch_control_diff_gain = raw_bytes_from_sim[26];
		pitch_control_int_gain = raw_bytes_from_sim[27];

		pitch_control_prop_gain = pitch_control_prop_gain  / 10;
		pitch_control_diff_gain = pitch_control_diff_gain / 100;
		pitch_control_int_gain = pitch_control_int_gain / 10000;*/

		printf("Pitch: %f, Roll: %f, IAS: %f, accx: %f, accy: %f, accz: %f, P: %f Q: %f  lat: %f  lon: %f  thetac: %f  phic: %f\n",  
				pitch, roll, ias_sen, acc_x, acc_y, acc_z, P, Q, latitude, longitude, theta_cal, phi_cal);

		pitch = theta_cal;
		roll = phi_cal;

		//printf("%d %d %d %d \n",raw_bytes_from_sim[36], raw_bytes_from_sim[35], raw_bytes_from_sim[34], raw_bytes_from_sim[33]);

		
		}
		

		all_received = 0;



		if (mode == AUTO_TAKEOFF) {
			n_wheel = 0;
			throttle = 100;
			rudder = 0;
			elev = 05;
			ail = 0;
			if (ias_sen > 170) {
				mode = CLIMB;
			}
		}
		else if (mode == CLIMB) {
			if (alt_ft_msl <= 3000) {
				longitudinal_control(15);
				lateral_control(0);
			}
			else {
				mode = ALT_HOLD;
			}
		}
		else if (mode == ALT_HOLD) {
			HDG_control(150);
			cruise_control(4000);
		}

		


		elev = safe_byte(elev);
		ail = safe_byte(ail);
		rudder = safe_byte(rudder);
		throttle = safe_byte(throttle);

		/*out_elev_L = (elev & 0xFF);
		out_elev_H = ((elev >> 8) & 0xFF);

		out_ail_L = (ail & 0xFF);
		out_ail_H = ((ail >> 8) & 0xFF);

		out_rud_L = (rudder & 0xFF);
		out_rud_H = ((rudder >> 8) & 0xFF);

		out_thr_L = (throttle & 0xFF);
		out_thr_H = ((throttle >> 8) & 0xFF);*/

		BYTE send_sequence [] = { 255,254,253,252, elev, ail, rudder, throttle};

		//printf("Elev: %d,aileron: %d, rudder:  %d throttle: %d \n", elev, ail, rudder, throttle );


		for (int send_i = 0; send_i < 8; send_i ++){

             write(uart2_ptr,send_sequence[send_i]);
		}
		
		if(ias_sen < 20){
			mode = AUTO_TAKEOFF;
		}
	}
}
