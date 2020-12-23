//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Aerial autopilots
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE
//#######################################################################################//
#include "sal.h"
#include "iap.h"
#include "air.h"
//#######################################################################################//
ap_runway_off::ap_runway_off(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Runway take-off\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("thrust",VT_DBL,&thrust,0,0,1);
 add_var("altitude",VT_DBL,&altitude,0,0,1e6);
 add_var("target",VT_CHAR,target,0,0,255);
 add_var("heading",VT_DBL,&heading,VF_DEG,0,360*RAD);
 add_var("v1",VT_DBL,&v1,0,0,1e6);
 add_var("gear_key",VT_INT,&gear_key,0,1,255);
 
 use_grp=xTHGROUP_MAIN;
 altitude=1000;
 heading=90*RAD;
 v1=170;
 gear_key=0;
 target[0]=0;
 use_target=0;
 thrust=1;
 max_time_accel=10;
 
 reset();
}
//#######################################################################################//
void ap_runway_off::reset()
{
 iautopilot::reset();
 stage=0;
}
//#######################################################################################//
void ap_runway_off::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_runway_off::init()
{
 //Set it
 stage=0;
 
 tgt_obj=0;
 use_target=true;
 if(target[0]!=0)tgt_obj=sal->object_by_name(target);
 if(!tgt_obj)use_target=false;
 
 is_loaded=true;
 sal->af_control_set(us,false);
}
//#######################################################################################//
void ap_runway_off::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_runway_off::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  double v,alt,hdg,tgt_p,tgt_y,tgt_r;
  OBJHANDLE ref;
  ref=sal->get_gravity_ref(us);
  if(use_target)heading=sal->calculate_azimuth(us,sal->get_obj_size(ref)+sal->get_obj_altitude(tgt_obj,ref),sal->get_obj_equ_inclination(tgt_obj,ref)); 
 
  
  v=modv(sal->get_ves_airspeed_vector(us));
  alt=sal->get_ves_altitude(us);
  hdg=get_heading(use_grp);
  
  if((alt>altitude)&&(absd(hdg-heading)<1*RAD)){stop();return;}
  
  tgt_p=get_pitch(use_grp);
  tgt_y=hdg;
  tgt_r=get_bank(use_grp);
   
  if(stage==0){
   sprintf(state_string,"Accelerating\0");
   set_thgrp_level(1,use_grp);
   if(alt>100){
    asf_zero();
    stage=1;
    if(gear_key!=0)sal->ves_key_press(us,gear_key);
    return;
   }
   
   if(v>v1){
    sprintf(state_string,"Rotating\0");
    tgt_p=45*RAD;
   }else{
    tgt_p=0;
   }
   
   asf_guide(simdt,tgt_p,tgt_y,tgt_r,use_grp);
   asf_control(simdt,tgt_p,tgt_y,tgt_r,use_grp);
  }
  if(stage==1){
   sprintf(state_string,"Turning");
   set_thgrp_level(thrust,use_grp);
   tgt_p=10*RAD;
   tgt_y=0;
   
   double c=hdg-heading;
   while(c<-PI)c+=2*PI;
   while(c> PI)c-=2*PI;
   //sprintf(oapiDebugString(),"c=%f",c/RAD);
   c=c/(10*RAD);
   
   if(c>1)c=1;
   if(c<-1)c=-1;
   tgt_r=45*c*RAD;
   
 
   asf_guide(simdt,tgt_p,0,tgt_r,use_grp);
   asf_control_exp(simdt,get_pitch(use_grp),0,get_bank(use_grp),tgt_p,0,tgt_r,5,use_grp);
  }
 }
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
ap_air_hold::ap_air_hold(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Aerial state hold\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("onoff",VT_INT,&onoff,0,0,7);
 add_var("altitude",VT_DBL,&altitude,0,0,1e6);
 add_var("heading",VT_DBL,&heading,VF_DEG,0,360*RAD);
 add_var("velocity",VT_DBL,&velocity,0,0,1e6);
 add_var("alt_rate",VT_DBL,&alt_rate,0,-1e6,1e6);
 add_var("hdg_rate",VT_DBL,&hdg_rate,VF_DEG,-360*RAD,360*RAD);
 add_var("vel_rate",VT_DBL,&vel_rate,0,-1e6,1e6);
 
 use_grp=xTHGROUP_MAIN;
 altitude=1000;
 heading=90*RAD;
 velocity=200;
 onoff=7;
 alt_rate=0;
 hdg_rate=0;
 vel_rate=0;
 
 max_time_accel=10;
}
//#######################################################################################//
void ap_air_hold::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_air_hold::init()
{
 is_loaded=true;
 set_thgrp_level(1,use_grp);
 sal->af_control_set(us,false);
 sprintf(state_string,"Flying");
}
//#######################################################################################//
void ap_air_hold::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_air_hold::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  VECTOR3 av;
  double v,alt,hdg,tgt_p,tgt_y,tgt_r;
  
  if(hdg_rate!=0)heading+=hdg_rate*simdt;
  if(alt_rate!=0)altitude+=alt_rate*simdt;
  if(vel_rate!=0)velocity+=vel_rate*simdt;
  if(heading>2*PI)heading=heading-2*PI;
  if(heading<0)heading=heading+2*PI;
  
  
  av=sal->get_ves_airspeed_vector(us);
  v=1-(modv(av)-velocity)/5;
  if(v>1)v=1;
  if(v<0)v=0;
  if(onoff&4)set_thgrp_level(v,use_grp);else set_thgrp_level(1,use_grp);
  
  alt=sal->get_ves_altitude(us);
  hdg=get_heading(use_grp);
  
  double c=hdg-heading;
  while(c<-PI)c+=2*PI;
  while(c> PI)c-=2*PI;
  c=c/(10*RAD);
  
  if(c>1)c=1;
  if(c<-1)c=-1;
  tgt_r=45*c*RAD;
  tgt_y=0;
  
  c=-(sal->get_ves_altitude(us)-altitude)/500;
  if(c>1)c=1;
  if(c<-1)c=-1;
  tgt_p=10*RAD*c;
  
  asf_guide(simdt,tgt_p,0,tgt_r,use_grp);
  asf_control_exp(simdt,get_pitch(use_grp),0,get_bank(use_grp),tgt_p,0,tgt_r,(onoff&1?5:0),use_grp);

 }
}
//#######################################################################################//
iautopilot *mk_runway_off(SAL *sal_in,VESSEL *v){return new ap_runway_off(sal_in,v);}
iautopilot *mk_air_hold(SAL *sal_in,VESSEL *v){return new ap_air_hold(sal_in,v);}
//#######################################################################################//