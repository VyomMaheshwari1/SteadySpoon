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
static void sample(float angle,float rate){
 tick+=10;mpu2.ok=mpu2.filter_initialized=1;mpu2.gyro_samples=200;mpu2.reference_samples=100;
 mpu2.read_status=HAL_OK;mpu2.read_count++;mpu2.last_update_ms=tick;
 mpu2.roll_filtered_deg=angle;mpu2.roll_reference_deg=0;mpu2.accel_x_g=0;mpu2.accel_z_g=-1;
 mpu2.gyro_x_dps=-rate;mpu2.gyro_y_dps=mpu2.gyro_z_dps=0;
}
int main(int argc,char **argv){
 int mode=argc>1?atoi(argv[1]):0;assert(servo2_hold_center_test==1);
 if(mode==0){for(int i=0;i<1000;i++){sample(i%2?45:-45,i%2?490:-490);Servo2_Stabilization_Update();assert(servo2_pulse_us==1500&&!servo2_control_fault);}return 0;}
 servo2_hold_center_test=0;
 if(mode==1){unsigned last=1500;for(int i=0;i<500;i++){sample(10,0);Servo2_Stabilization_Update();assert(!servo2_control_fault&&abs((int)servo2_pulse_us-(int)last)<=6);last=servo2_pulse_us;assert(fabsf(servo2_integral_us)<=150);}assert(servo2_integral_us< -40&&servo2_pulse_us<1460);}
 if(mode==2){sample(0,50);Servo2_Stabilization_Update();assert(servo2_d_term_us<0&&servo2_pulse_us<1500);for(int i=0;i<100;i++){sample(0,0);Servo2_Stabilization_Update();}assert(servo2_pulse_us==1500);}
 if(mode==3){sample(0,451);Servo2_Stabilization_Update();assert(servo2_control_fault==3&&servo2_pulse_us==1500);servo2_hold_center_test=1;sample(0,0);Servo2_Stabilization_Update();assert(servo2_control_fault==3);}
 if(mode==4){for(int i=0;i<25;i++){sample(0,(i/4)%2?150:-150);Servo2_Stabilization_Update();}assert(servo2_control_fault==4);unsigned last=servo2_pulse_us;sample(20,0);Servo2_Stabilization_Update();assert(servo2_pulse_us==last);}
 if(mode==5){sample(0,0);Servo2_Stabilization_Update();tick+=100;sample(0,0);Servo2_Stabilization_Update();assert(servo2_control_fault==2);}
 if(mode==6){sample(0,0);mpu2.last_update_ms=tick-21;Servo2_Stabilization_Update();assert(servo2_control_fault==1);}
 if(mode==7){for(int i=0;i<2000;i++){sample(200,0);Servo2_Stabilization_Update();assert(servo2_pulse_us>=1150&&servo2_pulse_us<=1850);assert(fabsf(servo2_integral_us)<.01f);}assert(servo2_pulse_us==1150&&servo2_saturated);}
 if(mode==8){tick=0xffffffe0U;for(int i=0;i<10;i++){sample(-10,0);Servo2_Stabilization_Update();assert(!servo2_control_fault&&servo2_pulse_us>=1500);}}
 if(mode==9){sample(10,0);Servo2_Stabilization_Update();float integral=servo2_integral_us;unsigned pulse=servo2_pulse_us;tick+=10;Servo2_Stabilization_Update();assert(servo2_integral_us==integral&&servo2_pulse_us==pulse);}
 if(mode==10){sample(NAN,0);Servo2_Stabilization_Update();assert(servo2_control_fault==1);}
 puts("PASS controller case");
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
    p.write_text(stub+prefix+fn('Servo2_SetPulse')+fn('Servo2_Stabilization_Update')+t)
    subprocess.run([compiler,'-std=c11','-O2','-Wall','-Wextra','-Werror',str(p),'-o',str(exe),'-lm'],check=True)
    for case in range(11):
        subprocess.run([str(exe),str(case)],check=True)
print('PASS all 11 Servo 2 controller cases')
