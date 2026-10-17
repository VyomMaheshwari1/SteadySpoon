from pathlib import Path
import re,subprocess,os,shutil,tempfile
s=(Path(__file__).resolve().parents[1] / 'Core/Src/main.c').read_text();prefix=re.sub(r'^#include.*$','',s[:s.index('void SystemClock_Config(void);')],flags=re.M)
stub='\n#include <stdint.h>\n#include <string.h>\n#include <math.h>\n#include <assert.h>\n#include <stdio.h>\ntypedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;\ntypedef struct { uint32_t ErrorCode; } I2C_HandleTypeDef;\ntypedef struct { uint32_t ccr; } TIM_HandleTypeDef;\n#define TIM_CHANNEL_3 3\n#define I2C_MEMADD_SIZE_8BIT 1\n#define __HAL_TIM_SET_COMPARE(h,ch,p) ((h)->ccr=(p))\nstatic uint32_t tick=100;\nstatic HAL_StatusTypeDef mock_status=HAL_OK;\nstatic uint8_t raw[14];\nstatic uint32_t HAL_GetTick(void) { return tick; }\nstatic void HAL_Delay(uint32_t ms) { tick+=ms; }\nstatic int32_t osKernelLock(void) { return 0; }\nstatic int32_t osKernelRestoreLock(int32_t lock) { return lock; }\nstatic HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h,int addr,int reg,int sz,uint8_t *out,int n,int timeout)\n{ (void)h;(void)addr;(void)reg;(void)sz;(void)timeout; tick+=2; if(mock_status==HAL_OK) memcpy(out,raw,n); return mock_status; }\n';stub=stub[:stub.index('static void HAL_Delay')]+'''static int32_t osKernelLock(void){return 0;}
static int32_t osKernelRestoreLock(int32_t x){return x;}
#include <stdlib.h>
''';stub=stub.replace('static HAL_StatusTypeDef mock_status=HAL_OK;','').replace('static uint8_t raw[14];','')
def fn(n):
 m=re.search(r'(?m)^static void '+n+r'\([^;]*?\)\s*\{',s);return s[m.start():s.index('\n}',m.end())+2]
t=r'''
static void sample(void){
 tick+=10;mpu2.ok=mpu2.filter_initialized=1;mpu2.gyro_samples=200;mpu2.reference_samples=100;
 mpu2.read_status=HAL_OK;mpu2.read_count++;mpu2.last_update_ms=tick;
 mpu2.roll_filtered_deg=0;mpu2.roll_reference_deg=0;mpu2.accel_x_g=0;mpu2.accel_z_g=-1;
 mpu2.gyro_x_dps=mpu2.gyro_y_dps=mpu2.gyro_z_dps=0;
 mpu1=mpu2;
}
int main(int argc,char **argv){
 int mode=argc>1?atoi(argv[1]):0;
 if(mode==0){for(int i=0;i<1000;i++){sample();servo2_hold_center_test=i%2;Servo2_Stabilization_Update();assert(servo2_pulse_us==1500&&servo2_test_state==0&&servo2_integral_us==0);}return 0;}
 if(mode==8)tick=0xfffff000U;
 sample();if(mode==11)mpu1.roll_filtered_deg=4;servo2_test_request=1;Servo2_Stabilization_Update();assert(servo2_test_state==1);
 if(mode==11){assert(servo2_pulse_us==1500);for(int i=0;i<50;i++){sample();mpu1.roll_filtered_deg=5;Servo2_Stabilization_Update();}assert(servo2_pulse_us==1513);return 0;}
 if(mode==12){for(int i=0;i<100;i++){sample();mpu1.pitch_filtered_deg=i%2?10.0f:-10.0f;Servo2_Stabilization_Update();assert(!servo2_test_fault&&servo2_pulse_us==1500);}return 0;}
 if(mode==1||mode==8){unsigned last=1500,low=1500,high=1500;for(int i=0;i<3100;i++){sample();mpu1.roll_filtered_deg=i<500?6:-6;Servo2_Stabilization_Update();assert(!servo2_test_fault);assert(abs((int)servo2_pulse_us-(int)last)<=1);last=servo2_pulse_us;if(last<low)low=last;if(last>high)high=last;assert(servo2_integral_us==0);}assert(low==1435&&high==1565&&last==1500&&servo2_test_state==6);servo2_test_request=1;sample();Servo2_Stabilization_Update();assert(servo2_test_request==0&&servo2_test_state==6);return 0;}
 for(int i=0;i<100;i++){sample();mpu1.roll_filtered_deg=3;Servo2_Stabilization_Update();}unsigned last=servo2_pulse_us;assert(last==1539);
 sample();
 if(mode==2)mpu1.roll_filtered_deg=9;
 if(mode==3)mpu2.last_update_ms=tick-21;
 if(mode==4)mpu2.gyro_x_dps=101;
 if(mode==5)tick+=30;
 if(mode==6)servo2_test_request=0;
 if(mode==7)mpu2.roll_filtered_deg=NAN;
 if(mode==9)mpu2.roll_filtered_deg=9;
 if(mode==10)servo2_test_request=2;
 if(mode==13)mpu1.pitch_filtered_deg=10.1f;
 Servo2_Stabilization_Update();assert(servo2_test_state==7&&servo2_test_fault&&servo2_inhibit&&servo2_pulse_us==last);
 sample();servo2_test_request=1;Servo2_Stabilization_Update();assert(servo2_test_state==7&&servo2_test_request==0&&servo2_pulse_us==last);
 puts("PASS abort freezes command and prevents rearming");
}
'''

compiler = os.environ.get('CC') or shutil.which('gcc') or shutil.which('cc')
if not compiler and Path('C:/msys64/ucrt64/bin/gcc.exe').exists():
    compiler = 'C:/msys64/ucrt64/bin/gcc.exe'
if not compiler:
    raise SystemExit('Install a host C compiler or set CC to its executable.')
with tempfile.TemporaryDirectory(prefix='servo2-test-') as folder:
    p = Path(folder) / 'test.c'
    exe = Path(folder) / ('test.exe' if os.name == 'nt' else 'test')
    p.write_text(stub+prefix+fn('Servo2_SetPulse')+fn('Servo2_ResponseTest_Update')+fn('Servo2_Stabilization_Update')+t)
    subprocess.run([compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror',str(p),'-o',str(exe),'-lm'],check=True)
    for case in range(14):
        subprocess.run([str(exe),str(case)],check=True)
print('PASS all 14 feedforward bench cases')
