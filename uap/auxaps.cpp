//#######################################################################################//   
//UAP module, Made by Artlav in 2011
//Auxillary autopilots
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE
//#######################################################################################//
#include "sal.h"
#include "iap.h"
#include "auxaps.h"
#include "uapsys.h"
//#######################################################################################//
ap_liftoff::ap_liftoff(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Lift-off\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("mode",VT_INT,&mode,0,0,3);
 add_var("target",VT_CHAR,target,0,0,255);
 add_var("heading",VT_DBL,&tgt_heading,VF_DEG,0,360*RAD);
 add_var("altitude",VT_DBL,&tgt_alt,0,0,1e100);
 add_var("pitch_tgt",VT_DBL,&pitch_tgt,VF_DEG,0,360*RAD);
 add_var("pitch_duration",VT_DBL,&pitch_duration,0,0,1e3);
 add_var("off_duration",VT_DBL,&off_duration,0,0,1e3);
 add_var("stage",VT_INT,&stage,VF_STATE,0,1);
 add_var("time_to_end",VT_DBL,&time_to_end,VF_OUT,0,1e3);
 add_var("mode_is",VT_CHAR,&mode_is,VF_OUT,0,255);

 pitch_tgt=20*RAD;
 use_grp=xTHGROUP_MAIN;
 tgt_heading=90*RAD;
 pitch_duration=110;
 off_duration=10;
 tgt_alt=modv(get_major_gravity_local_acc_vector())*50; //10 seconds to fall from
 max_time_accel=10;
 target[0]=0;
 use_target=0;
 mode=2;
 info();
 reset();
}
//#######################################################################################//
void ap_liftoff::reset()
{
 iautopilot::reset();
 time_to_end=0;
 stage=0;
}
//#######################################################################################//
void ap_liftoff::info()
{
 switch(mode){
  case 0:set_char_var("mode_is","Hover, align heading");break;
  case 1:set_char_var("mode_is","Take off, align heading");break;
  case 2:set_char_var("mode_is","Take off, roll, pitch");break;
  case 3:set_char_var("mode_is","Take off, roll, pitch-hover");break;
  default:set_char_var("mode_is","WTF?");
 }
}
//#######################################################################################//
void ap_liftoff::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 if(mode<2)if(sal->get_ves_altitude(us)>tgt_alt)stop();
 
 init();
}
//#######################################################################################//
void ap_liftoff::init()
{
 is_loaded=true;
 tgt_obj=0;
 use_target=true;
 if(target[0]!=0)tgt_obj=sal->object_by_name(target);
 if(!tgt_obj)use_target=false;
 rt=sal->sim_time();
}
//#######################################################################################//
void ap_liftoff::stop()
{
 iautopilot::stop();
 
 is_running=false;
 is_started=false;
 is_finished=true;
 //if(mode==0)
 stop_all_engines();
 //us->ActivateNavmode(NAVMODE_KILLROT);
}
//#######################################################################################//
void ap_liftoff::step(double simt,double simdt)
{
 if(!is_running)return;
 if(!is_loaded)init();

 OBJHANDLE ref;
 ref=sal->get_gravity_ref(us);
 if(use_target)tgt_heading=sal->calculate_azimuth(us,sal->get_obj_size(ref)+sal->get_obj_altitude(tgt_obj,ref),sal->get_obj_equ_inclination(tgt_obj,ref)); 
 
 if(tgt_heading<0)tgt_heading=2*PI+tgt_heading;
  
 if(mode==0){
  if((sal->get_ves_altitude(us)>tgt_alt)&&(absd(get_roll_heading(use_grp)-tgt_heading)<1*RAD)){stop();return;}
  VECTOR3 lv,ga;
  
  ga=get_major_gravity_local_acc_vector();
  lv=sal->get_ves_horizon_airspeed_vector(us);
  double vspd,dalt,xalt,ts;
  vspd=lv.y;
  dalt=tgt_alt-sal->get_ves_altitude(us);
  
  ts=vspd/modv(ga);
  xalt=vspd*ts-modv(ga)*ts*ts/2;
  if((xalt>dalt)&&(vspd>0)){
   stop_all_engines();
  }else{
   give_thgrp_accel(modv(ga)*2,-ga,use_grp);
  }
 
  att_guide(simdt,90*RAD,0,tgt_heading,use_grp);
  att_control_exp(simdt,get_pitch_horizon(use_grp),get_yaw_vertical(use_grp),get_roll_heading(use_grp),90*RAD,0,tgt_heading,7,use_grp);
 }
 
 if(mode==1){
  set_thgrp_level(1,use_grp);
  if((sal->get_ves_altitude(us)>tgt_alt)&&(absd(get_roll_heading(use_grp)-tgt_heading)<1*RAD)){stop();return;}
  att_guide(simdt,90*RAD,0,tgt_heading,use_grp);
  att_control_exp(simdt,get_pitch_horizon(use_grp),get_yaw_vertical(use_grp),get_roll_heading(use_grp),90*RAD,0,tgt_heading,7,use_grp);
 }
 
 if(mode==2){
  set_thgrp_level(1,use_grp);
  if(stage==0){
   sprintf(state_string,"Going up");
   if((simt-rt)<off_duration)return;
   if((simt-rt)>off_duration+100){stop();return;}
   double hdg=get_roll_heading(use_grp);
   if(get_pitch_horizon(use_grp)<45*RAD)hdg=get_heading(use_grp);
   if(((simt-rt)>off_duration)&&(absd(hdg-tgt_heading)<1*RAD)){
    stage=1;
    pitch_start=get_pitch_horizon(use_grp);
    rt=simt;
    att_guide_not_set=1;
    return;
   }
   att_guide(simdt,90*RAD-1*RAD,0,tgt_heading,use_grp);
   att_control_exp(simdt,get_pitch_horizon(use_grp),get_yaw_vertical(use_grp),hdg,90*RAD,0,tgt_heading,7,use_grp);
  }else if(stage==1){
   sprintf(state_string,"Pitching");
   if((simt-rt)>pitch_duration){stop();return;}
   double tgt_pitch=linterp(0,pitch_start,pitch_duration,pitch_tgt,simt-rt);
   time_to_end=pitch_duration-(simt-rt);
   att_guide(simdt,tgt_pitch,tgt_heading,0,use_grp);
   att_control_exp(simdt,get_pitch_horizon(use_grp),get_heading(use_grp),get_bank(use_grp),tgt_pitch,tgt_heading,0,7,use_grp);
  }
 }
 if(mode==3){
  if((sal->get_ves_altitude(us)>tgt_alt)&&(absd(get_roll_heading(use_grp)-tgt_heading)<1*RAD)&&(absd(get_pitch_horizon(use_grp)-pitch_tgt)<1*RAD)){stop();return;}
  VECTOR3 lv,ga;
  
  ga=get_major_gravity_local_acc_vector();
  lv=sal->get_ves_horizon_airspeed_vector(us);
  double vspd,dalt,xalt,ts;
  vspd=lv.y;
  dalt=tgt_alt-sal->get_ves_altitude(us);
  
  ts=vspd/modv(ga);
  xalt=vspd*ts-modv(ga)*ts*ts/2;
  if((xalt>dalt)&&(vspd>0)){
   stop_all_engines();
  }else{
   give_thgrp_accel(modv(ga)*2,-ga,xTHGROUP_MAIN+xTHGROUP_HOVER);
  }
 
  att_guide(simdt,90*RAD,0,tgt_heading,use_grp);
  att_control_exp(simdt,get_pitch(use_grp),get_yaw_vertical(use_grp),get_roll_heading(use_grp),pitch_tgt,0,tgt_heading,7,use_grp);
 }
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
ap_attitude::ap_attitude(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Attitude\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("mode",VT_INT,&mode,0,0,2);
 add_var("mode_is",VT_CHAR,&mode_is,VF_OUT,0,255);
 
 mode=0;
 max_time_accel=10;
 use_grp=xTHGROUP_MAIN;
 info();
}
//#######################################################################################//
void ap_attitude::info()
{
 switch(mode){
  case 0:set_char_var("mode_is","Prograde");break;
  case 1:set_char_var("mode_is","Retrograde");break;
  case 2:set_char_var("mode_is","Killrot");break;
  default:set_char_var("mode_is","WTF?");
 }
}
//#######################################################################################//
void ap_attitude::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
}
//#######################################################################################//
void ap_attitude::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_attitude::step(double simt,double simdt)
{
 if(!is_running)return;
 VECTOR3 ang_vel,pyr;
 ang_vel=get_angular_vel_grp(use_grp);

 if(mode==0){
  sprintf(state_string,"Orienting");
  pyr=get_pitchyawroll(sal->global_2_local(us,sal->get_relative_vel(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)),sal->global_2_local(us,sal->get_relative_pos(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)));
  if((absd(pyr.x-0)<1*RAD)&&(absd(pyr.y-0)<1*RAD)&&(absd(pyr.z-0)<1*RAD)){
   if((absd(ang_vel.x)<0.05)&&(absd(ang_vel.y)<0.05)&&(absd(ang_vel.z)<0.05)){stop();return;}
  }
  att_guide(simdt,0,0,0,use_grp);
  att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,use_grp);
 }else if(mode==1){
  sprintf(state_string,"Orienting");
  pyr=get_pitchyawroll(-sal->global_2_local(us,sal->get_relative_vel(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)),sal->global_2_local(us,sal->get_relative_pos(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)));
  if((absd(pyr.x-0)<1*RAD)&&(absd(pyr.y-0)<1*RAD)&&(absd(pyr.z-0)<1*RAD)){
   if((absd(ang_vel.x)<0.05)&&(absd(ang_vel.y)<0.05)&&(absd(ang_vel.z)<0.05)){stop();return;}
  }
  att_guide(simdt,0,0,0,use_grp);
  att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,use_grp);
 }else if(mode==2){
  sprintf(state_string,"Despinning");
  if((absd(ang_vel.x)<0.001)&&(absd(ang_vel.y)<0.001)&&(absd(ang_vel.z)<0.001)){stop();return;}
  att_killrot(7,use_grp);
 }
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
ap_maneuvre::ap_maneuvre(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Maneuvre\0");
 add_var("engine"     ,VT_INT ,&use_grp,VF_INP,1,15);
 add_var("mode"       ,VT_INT ,&mode   ,VF_INP,0,1);  
 add_var("target"     ,VT_CHAR,tgt_nam ,VF_INP,0,255);
 add_var("orientation",VT_INT ,&dvdir  ,VF_INP,0,3);
 add_var("dv"         ,VT_DBL ,&dv     ,VF_INP,0,1e10);
 add_var("dt"         ,VT_DBL ,&dt     ,VF_INP,0,1e10);
 add_var("level"      ,VT_DBL ,&level  ,VF_INP,0,1);
 add_var("mode_is"    ,VT_CHAR,&mode_is,VF_OUT,0,255);
 add_var("dir_is"     ,VT_CHAR,&dir_is ,VF_OUT,0,255);
          
 sprintf(tgt_nam,"\0");
 use_grp=xTHGROUP_MAIN;
 mode=0;
 dv=100;
 dt=10;
 dvdir=0;
 level=1;
 max_time_accel=10;
 use_grp=xTHGROUP_MAIN;
 info();
}
//#######################################################################################//
void ap_maneuvre::info()
{
 switch(mode){
  case 0:set_char_var("mode_is","Timed burn");break;
  case 1:set_char_var("mode_is","Dv burn");break;
  default:set_char_var("mode_is","WTF?");
 }
 switch(dvdir){
  case 0:set_char_var("dir_is","Prograde");break;
  case 1:set_char_var("dir_is","Retrograde");break;
  case 2:set_char_var("dir_is","From target");break;
  case 3:set_char_var("dir_is","Towards target");break;
  default:set_char_var("dir_is","N/A");
 }
}
//#######################################################################################//
void ap_maneuvre::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}      
//#######################################################################################//
void ap_maneuvre::init()
{
 is_loaded=true;
 
 stop_all_engines();
 
 tgt_ves=sal->vessel_by_name(tgt_nam);
 if((dvdir==2)||(dvdir==3))if(!tgt_ves){fail_error("No such target vessel");return;}
}
//#######################################################################################//
void ap_maneuvre::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_maneuvre::step(double simt,double simdt)
{
 if(!is_running)return; 
 if(!is_loaded)init();
 VECTOR3 ang_vel,pyr,d,rv,rp;                 
 int can_burn=0;
 double acc,lv;    

 ang_vel=get_angular_vel_grp(use_grp);
                      

 sprintf(state_string,"Orienting)");
 if(dvdir==0){
  pyr=get_pitchyawroll(sal->global_2_local(us,sal->get_relative_vel(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)),sal->global_2_local(us,sal->get_relative_pos(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)));
  if((absd(pyr.x-0)<1*RAD)&&(absd(pyr.y-0)<1*RAD)&&(absd(pyr.z-0)<1*RAD)){
   if((absd(ang_vel.x)<0.05)&&(absd(ang_vel.y)<0.05)&&(absd(ang_vel.z)<0.05))can_burn=1;
  }
  att_guide(simdt,0,0,0,1);
  att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,1);
 }else if(dvdir==1){
  pyr=get_pitchyawroll(-sal->global_2_local(us,sal->get_relative_vel(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)),sal->global_2_local(us,sal->get_relative_pos(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)));
  if((absd(pyr.x-0)<1*RAD)&&(absd(pyr.y-0)<1*RAD)&&(absd(pyr.z-0)<1*RAD)){
   if((absd(ang_vel.x)<0.05)&&(absd(ang_vel.y)<0.05)&&(absd(ang_vel.z)<0.05))can_burn=1;
  }
  att_guide(simdt,0,0,0,1);
  att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,1);
 }else if((dvdir==2)||(dvdir==3)){   
  rv=sal->get_obj_relative_vel(sal->get_ves_obj(us),sal->get_ves_obj(tgt_ves));
  rp=sal->get_obj_relative_pos(sal->get_ves_obj(us),sal->get_ves_obj(tgt_ves));
  d=sal->global_2_local(us,rp+sal->get_global_pos(us));
  if(dvdir==3)d=-d;
  pyr=get_pitchyawroll(d,tvec(0,0,0));
  if((absd(pyr.x-0)<1*RAD)&&(absd(pyr.y-0)<1*RAD)&&(absd(pyr.z-0)<1*RAD)){
   if((absd(ang_vel.x)<0.05)&&(absd(ang_vel.y)<0.05)&&(absd(ang_vel.z)<0.05))can_burn=1;
  }
  att_guide(simdt,0,0,0,1);
  att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,1);
 }

 if(can_burn){  
  sprintf(state_string,"Burning");
  if(mode==0){  
   dt=dt-simdt; 
   if(dt<=0){stop();return;}  
   set_thgrp_level(level,use_grp);
  }else if(mode==1){      
   if(dv<=0){stop();return;}  
   acc=modv(get_group_thrust_vector(use_grp,true))/sal->get_ves_mass(us);
   if(level*acc*simdt>dv)lv=dv/simdt;else lv=level*acc;
   dv=dv-lv*simdt;    
   set_thgrp_level(lv/acc,use_grp);
  }
 }else set_thgrp_level(0,use_grp);
}
//#######################################################################################//
//#######################################################################################//
#ifdef USE_ORBITER
//#######################################################################################//
#pragma comment(lib, "winmm.lib")
//#######################################################################################//
//описание заголовка файла WAV
#pragma pack(push,1)
struct WAVHEADER
{
 char    sigRIFF[4];     // должно быть равно "RIFF"
 DWORD   sizeRIFFch;     // размер чанка RIFF
 char    sigWAVE[4];     // должно быть равно "WAVE"
 char    sigFMT[4];      // должно быть равно "fmt "
 DWORD   sizeFMTch;      // размер чанка FMT
 WORD    wFormatTag;     // категория формата, для PCM = 1
 WORD    wChannels;      // кол-во каналов: 1-моно 2-стерео
 DWORD   dwSamplesPerSec;// кол-во сэмплов в сек.
 DWORD   dwAvgBytesPerSec;// среднее число байт в сек
 WORD    wBlockAlign;    // выравнивание данных в дата-чанке
 WORD    wBitPerSample;  // бит в сэмпле
 char    sigDATA[4];     // должно быть равно "data"
 DWORD   sizeDATAch;     // размер data-чанка
};
#pragma pack (pop)
//#######################################################################################//
void do_beep(int type)
{
 DWORD numSamples,size,i,j,freq,lng;
 void *buff;
 WAVHEADER *head;
 BYTE *samples;
 DWORD cnt;
 
 cnt=100;
 
 switch(type){
  case 1:{
   freq=900;
   lng=1000;
   numSamples=(44100/freq);

   size=sizeof(WAVHEADER)+cnt*numSamples;
   buff=malloc(size);
   samples=((BYTE*)(buff))+sizeof(WAVHEADER);
   
   for(j=0;j<cnt;j++)for(i=0;i<numSamples;i++)samples[j*numSamples+i]=BYTE(128*(1+sin(2*PI*( double(i)/(double(numSamples)*((double(j)/100))*0.5+0.5)  ))));
   break;
  }
  case 2:{
   freq=900;
   lng=1000;
   numSamples=(44100/freq);

   size=sizeof(WAVHEADER)+cnt*numSamples;
   buff=malloc(size);
   samples=((BYTE*)(buff))+sizeof(WAVHEADER);
   
   for(j=0;j<cnt;j++)for(i=0;i<numSamples;i++)samples[j*numSamples+i]=BYTE(128*(1+sin(2*PI*( double(i)/double(numSamples)  ))));
   break;
  }
  case 3:{
   freq=3000;
   lng=500;
   numSamples=(44100/freq);

   size=sizeof(WAVHEADER)+cnt*numSamples;
   buff=malloc(size);
   samples=((BYTE*)(buff))+sizeof(WAVHEADER);
   
   for(j=0;j<cnt;j++)for(i=0;i<numSamples;i++)samples[j*numSamples+i]=BYTE(128*(1+sin(2*PI*( double(i)/double(numSamples)  ))));
   break;
  }
 }
 
 head=(WAVHEADER*)buff;
 strcpy(head->sigRIFF,"RIFF");
 strcpy(head->sigWAVE,"WAVE");
 strcpy(head->sigFMT,"fmt ");
 strcpy(head->sigDATA,"data");
 head->sizeRIFFch=size-8;
 head->sizeFMTch=16;
 head->wFormatTag=1;
 head->wChannels=1;
 head->dwSamplesPerSec=44100;
 head->dwAvgBytesPerSec=44100;
 head->wBlockAlign=1;
 head->wBitPerSample=8;
 head->sizeDATAch=cnt*numSamples;
 
 //FILE* ouf=fopen("snd.wav","w");
 //fwrite(buff,1,size,ouf);
 //fclose(ouf);
 
 PlaySound((const char*)buff,0,SND_ASYNC|SND_LOOP|SND_MEMORY);
 Sleep(lng);
 PlaySound(0,0,SND_ASYNC);
}
//#######################################################################################//
DWORD WINAPI snd_th(int tp)
{
 do_beep(tp);
	return NULL;
}
//#######################################################################################//
void make_snd(int tp)
{
	DWORD id;
	HANDLE h=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)snd_th,(void*)tp,0,&id);
}
//#######################################################################################//
#endif
//#######################################################################################//
ap_tools::ap_tools(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Tools\0");
 add_var("type",VT_INT,&typ,0,0,9);
 add_var("key",VT_INT,&key,0,0,255);
 add_var("param",VT_INT,&param,0,1,3);
 add_var("wait",VT_DBL,&dt,0,0,1e100);
 add_var("target",VT_CHAR,tgt_nam,0,0,255);
 add_var("time_to_end",VT_DBL,&time_to_end,VF_OUT,0,1e100);
 add_var("mode_is",VT_CHAR,&mode_is,VF_OUT,0,255);
 
 typ=1;
 key=0;
 param=1;
 dt=5;
 w_seq=-1;
 sprintf(tgt_nam,"\0");
 time_to_end=0;
 ct=sal->sim_time();
 max_time_accel=10;
 info();
 reset();
}
//#######################################################################################//
void ap_tools::reset()
{
 iautopilot::reset();
 dt=0;
}
//#######################################################################################//
void ap_tools::info()
{
 switch(typ){
  case 0:set_char_var("mode_is","Key press");break;
  case 1:set_char_var("mode_is","Wait for timer");break;
  case 2:set_char_var("mode_is","Wait apoapsis");break;
  case 3:set_char_var("mode_is","Wait periapsis");break;
  case 4:set_char_var("mode_is","Trigger start");break;
  case 5:set_char_var("mode_is","Wait for target");break;
  case 6:set_char_var("mode_is","Undock");break;
  case 7:set_char_var("mode_is","Make sound");break;  
  case 8:set_char_var("mode_is","Wait less distance");break;
  case 9:set_char_var("mode_is","Wait more distance");break;
  default:set_char_var("mode_is","WTF?");
 }
}
//#######################################################################################//
void ap_tools::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_tools::init()
{
 is_loaded=true;
 switch(typ){
  case 0:{
   VESSEL* v=sal->vessel_by_name(tgt_nam);
   if(!v)v=us;
   sal->ves_key_press(v,key);stop();
   break;
  }
  case 1:ct=sal->sim_time();break;
  case 2:prev_alt=sal->get_ves_altitude(us);stg=0;break;
  case 3:prev_alt=sal->get_ves_altitude(us);stg=0;break;
  case 4:{
   VESSEL* v=sal->vessel_by_name(tgt_nam);
   if(!v){fail_error("No target vessel");return;}
   int n=get_seq_by_vessel(v);
   if(n==-1){stop();return;}
   uap_start_seq(n);
   stop();
   break;
  }
  case 5:{
   VESSEL* v=sal->vessel_by_name(tgt_nam);
   if(!v){fail_error("No target vessel");return;}
   w_seq=get_seq_by_vessel(v);
   if(w_seq==-1){stop();return;}
   if(!seqs[w_seq].run)stop();
   break;
  }
  #ifdef USE_ORBITER
  case 6:us->Undock(ALLDOCKS);stop();break;
  case 7:make_snd(param);stop();break;  
  #endif
  case 8:
  case 9:{
   tgt_obj=sal->object_by_name(tgt_nam);
   if(!tgt_obj){fail_error("No target object");return;}
   break;
  }
  default:stop();
 }
}
//#######################################################################################//
void ap_tools::step(double simt,double simdt)
{
 double alt;
 if(!is_loaded)init();
 
 sprintf(state_string,"Waiting");
 switch(typ){
  case 1:if(simt-ct>dt)stop();time_to_end=dt-(simt-ct);max_time_accel=10000;if(time_to_end<1000)max_time_accel=100;break;
  case 2:max_time_accel=1000;alt=sal->get_ves_altitude(us);if(alt<prev_alt){if(stg==1)stop();}else stg=1;prev_alt=alt;break;
  case 3:max_time_accel=1000;alt=sal->get_ves_altitude(us);if(alt>prev_alt){if(stg==1)stop();}else stg=1;prev_alt=alt;break;
  case 5:max_time_accel=10000;if(!seqs[w_seq].run)stop();break;
  case 8:
  case 9:{
   max_time_accel=1000;
   double dist=modv(sal->get_obj_relative_pos(sal->get_ves_obj(us),tgt_obj));

   if(typ==8)if(dist<=dt){stop();return;}
   if(typ==9)if(dist>=dt){stop();return;}
   break;
  }
  default:stop();
 }
}
//#######################################################################################//
iautopilot *mk_maneuvre(SAL *sal_in,VESSEL *v){return new ap_maneuvre(sal_in,v);}
iautopilot *mk_attitude(SAL *sal_in,VESSEL *v){return new ap_attitude(sal_in,v);}
iautopilot *mk_liftoff(SAL *sal_in,VESSEL *v){return new ap_liftoff(sal_in,v);}
iautopilot *mk_tools(SAL *sal_in,VESSEL *v){return new ap_tools(sal_in,v);}
//#######################################################################################//